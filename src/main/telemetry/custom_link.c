/*
 * This file is part of Cleanflight and Betaflight.
 *
 * Cleanflight and Betaflight are free software: you can redistribute
 * this software and/or modify this software under the terms of the
 * GNU General Public License as published by the Free Software
 * Foundation, either version 3 of the License, or (at your option)
 * any later version.
 *
 * Cleanflight and Betaflight is distributed in the hope that it
 * will be useful, but WITHOUT ANY WARRANTY; without even the implied
 * warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this software.
 *
 * If not, see <http://www.gnu.org/licenses/>.
 */

#include <stdbool.h>
#include <stdint.h>
#include <math.h>
#include <string.h>

#include "platform.h"

#include "build/debug.h"

#if defined(USE_CUSTOM_LINK)

#include "common/axis.h"
#include "common/maths.h"
#include "common/time.h"
#include "common/utils.h"

#include "drivers/dshot.h"
#include "drivers/motor.h"
#include "drivers/serial.h"
#include "drivers/system.h"
#include "drivers/time.h"

#include "scheduler/scheduler.h"

#include "cli/cli.h"

#include "fc/core.h"
#include "fc/rc_modes.h"
#include "fc/runtime_config.h"

#include "flight/failsafe.h"
#include "flight/imu.h"
#include "flight/mixer.h"
#include "flight/pid.h"
#include "flight/pid_init.h"
#include "flight/position_estimator.h"

#include "io/gps.h"
#include "io/serial.h"

#include "config/config.h"

#include "pg/custom_link.h"

#include "rx/rx.h"
#include "rx/rx_bind.h"

#include "sensors/acceleration.h"
#include "sensors/barometer.h"
#include "sensors/battery.h"
#include "sensors/gyro.h"

#ifdef SIMULATOR
// Simulator sensor injection (0x21/0x22/0x23) feeds the same virtual sensor
// drivers the UDP FDM bridge uses; these headers exist on every target but
// the symbols only link on SITL builds, so all references stay inside
// #ifdef SIMULATOR / USE_VIRTUAL_* guards.
#include <stdio.h>

#include "drivers/accgyro/accgyro_virtual.h"
#include "drivers/barometer/barometer_virtual.h"
#include "drivers/compass/compass_virtual.h"
#include "io/gps_virtual.h"
#endif

#include "telemetry/custom_link.h"
#include "telemetry/custom_link_protocol.h"

// Completed uplink frames buffered from ISR/parser to the 200 Hz task. A
// simulator host injects 0x21/0x22/0x23 alongside 0x20 (~400 frames/s total,
// bursty against the task period), so SITL builds get extra headroom.
#ifdef SIMULATOR
#define CL_RX_RING_LEN          8
#else
#define CL_RX_RING_LEN          4
#endif

typedef struct {
    clFrame_t frame;
    timeUs_t t2Us;                     // ISR timestamp of frame completion (time sync T2)
} clRxRingEntry_t;

static serialPort_t *customLinkPort;
static bool linkEnabled;

static clParser_t parser;
static clControlState_t controlState;

static uint8_t downlinkSeq;            // one shared sequence counter for all downlink frames

// written by the 200 Hz task, read from the PID loop and mixer
static volatile bool controlActive;
static volatile float cachedRate[XYZ_AXIS_COUNT];      // deg/s
static volatile float cachedThrottle;                  // 0.0 .. 1.0

// single-producer (RX ISR) single-consumer (200 Hz task) ring
static clRxRingEntry_t rxRing[CL_RX_RING_LEN];
static volatile uint8_t rxRingHead;
static volatile uint8_t rxRingTail;

// Uplink observability (MSP_DEBUG, DEBUG_CUSTOM_LINK): a feeder-side stall
// shows env stopping while imu keeps counting; an FC-side loss shows frames
// or bytes flattening while the host insists it is streaming. Counters are
// u16-truncated on export - liveness is the signal, not the absolute value.
// Byte parsing runs from the serial RX callback (IRQ context on hardware
// UARTs) or from the 200 Hz task's buffered drain (SITL TCP ports) depending
// on the port driver, and the export below races both, so every access is
// atomic; relaxed ordering suffices because these are statistics only.
#define CL_STAT_INC(stat) __atomic_fetch_add(&(stat), 1, __ATOMIC_RELAXED)
static uint32_t clStatFrames;      // parser-completed uplink frames, all types
static uint32_t clStatEnv;         // 0x22 HOST_ENV frames processed
static uint32_t clStatImu;         // 0x21 HOST_IMU frames processed
static uint32_t clStatRc;          // 0x23 HOST_RC frames processed
static uint32_t clStatRingDrops;   // uplink frames dropped on a full rxRing
static uint32_t clStatBytes;       // bytes pulled from the serial port

// pending time sync response, task context only
static bool syncPending;
static uint64_t syncT1;
static uint32_t syncT2Us;

// CLI read/write channel (0x24 up / 0x15 down). One in-flight response at a
// time; a new request replaces it (last request wins). GET/SET responses are
// snapshotted into buf once and streamed out; INFO regenerates each window
// through the offset-aware helper because its text (lookup value lists) can
// exceed any reasonable static buffer.
#define CL_CLI_GET_BUF_LEN      288     // worst-case "name = value" + NUL

typedef struct {
    bool active;
    bool fromBuffer;                          // buf snapshot vs INFO regeneration
    uint8_t status;
    uint16_t reqId;
    uint16_t offset;
    uint16_t totalLen;
    char name[CL_CLI_MAX_ARGS + 1];           // cached INFO setting name
    char buf[CL_CLI_GET_BUF_LEN];
} clCliStream_t;

static clCliStream_t cliStream;

// True when the settings engine (table scan, PG copy, sprintf) ran during the
// current taskFast period. That work is bounded, tens-of-microsecond bursts -
// unlike serial-write stalls - so its cost is let into the scheduler's
// execution-time estimate; only the stall jitter keeps the ignore below.
static bool cliEngineWorked;

static void customLinkFrameHandler(const clFrame_t *frame, timeUs_t frameCompleteUs, void *ctx)
{
    UNUSED(ctx);

    CL_STAT_INC(clStatFrames);

    const uint8_t next = (rxRingHead + 1) % CL_RX_RING_LEN;
    if (next == rxRingTail) {
        CL_STAT_INC(clStatRingDrops);  // ring full: drop the frame
        return;
    }
    rxRing[rxRingHead].frame = *frame;
    rxRing[rxRingHead].t2Us = frameCompleteUs;
    rxRingHead = next;
}

// Receive ISR callback, called per byte while the port is on the IRQ RX path
static void customLinkRxCallback(uint16_t c, void *data)
{
    UNUSED(data);
    clParserProcessByte(&parser, (uint8_t)c, microsISR(), customLinkFrameHandler, NULL);
}

// Bytes that were buffered instead of reaching the ISR callback (transports
// without per-byte callbacks, e.g. the SITL TCP serial) are parsed here with a
// poll-time timestamp. On real UARTs the rx ring stays empty and this is a
// no-op, preserving ISR-precision T2 timestamps.
static void customLinkDrainBufferedRx(void)
{
    uint32_t drained = 0;
    while (serialRxBytesWaiting(customLinkPort) > 0) {
        const uint8_t c = serialRead(customLinkPort);
        clParserProcessByte(&parser, c, micros(), customLinkFrameHandler, NULL);
        drained++;
    }
    __atomic_fetch_add(&clStatBytes, drained, __ATOMIC_RELAXED);
}

// Send that only consumes a sequence number once the frame is actually on the
// wire, so a deferred retry never reads as a lost telemetry frame. Used by the
// CLI response streamer, which re-attempts the same chunk next task period.
static bool customLinkTrySendFrame(uint8_t msgId, const void *payload, uint8_t payloadLen)
{
    uint8_t buf[CUSTOM_LINK_FRAME_MAX];
    const uint8_t len = clEncodeFrame(msgId, downlinkSeq, payload, payloadLen, buf, sizeof(buf));
    if (len == 0) {
        return false;
    }
    // serialWriteBuf busy-waits for buffer space on a stalled transport
    // (no TCP client, slow host); a wedged task would freeze the whole
    // cooperative scheduler. Degrade to dropping the frame instead.
    if (serialTxBytesFree(customLinkPort) < (uint32_t)len) {
        return false;
    }
    serialWriteBuf(customLinkPort, buf, len);
    downlinkSeq++;
    return true;
}

static void customLinkSendFrame(uint8_t msgId, const void *payload, uint8_t payloadLen)
{
    // telemetry frames consume a sequence number even when dropped: the host
    // loss detector must see the gap (this preserves the original semantics)
    uint8_t buf[CUSTOM_LINK_FRAME_MAX];
    const uint8_t len = clEncodeFrame(msgId, downlinkSeq++, payload, payloadLen, buf, sizeof(buf));
    if (len == 0) {
        return;
    }
    if (serialTxBytesFree(customLinkPort) < (uint32_t)len) {
        return;
    }
    serialWriteBuf(customLinkPort, buf, len);
}

static void customLinkCliStartBuffered(uint16_t reqId, uint8_t status, const char *text, uint16_t len)
{
    cliStream.active = true;
    cliStream.fromBuffer = true;
    cliStream.status = status;
    cliStream.reqId = reqId;
    cliStream.offset = 0;
    cliStream.totalLen = len;
    memcpy(cliStream.buf, text, len);
}

// Variant for responses composed directly into cliStream.buf (the GET/SET
// readback path): keeps the 200 Hz task clear of a second sizeable stack
// buffer on top of the CLI engine's own parsing buffers.
static void customLinkCliStartSnapshot(uint16_t reqId, uint8_t status, uint16_t len)
{
    cliStream.active = true;
    cliStream.fromBuffer = true;
    cliStream.status = status;
    cliStream.reqId = reqId;
    cliStream.offset = 0;
    cliStream.totalLen = len;
}

#ifdef USE_CLI
static void customLinkCliStartInfo(uint16_t reqId, const char *name)
{
    cliStream.active = true;
    cliStream.fromBuffer = false;
    cliStream.status = CL_CLI_STATUS_OK;
    cliStream.reqId = reqId;
    cliStream.offset = 0;
    cliStream.totalLen = 0;                   // latched from the first window
    strncpy(cliStream.name, name, sizeof(cliStream.name) - 1);
    cliStream.name[sizeof(cliStream.name) - 1] = '\0';
}
#endif // USE_CLI

// Streams at most one response chunk per task period (5 ms) so CLI traffic
// never displaces telemetry frames; a full TX buffer defers the chunk to the
// next period without consuming a sequence number. Both chunk sources share
// the same header/send/advance epilogue - only the way resp.data is filled
// differs (snapshot memcpy vs offset-windowed regeneration).
static void customLinkCliPump(void)
{
    if (!cliStream.active) {
        return;
    }

    clPayloadCliResp_t resp;
    uint8_t payloadLen;
    if (!cliStream.fromBuffer) {
#if defined(USE_CLI)
        int totalLen = 0;
        const int written = cliGetSettingInfoByName(cliStream.name, cliStream.offset,
            resp.data, CL_CLI_MAX_DATA, &totalLen);
        if (written < 0 || totalLen <= 0 ||
            (written == 0 && cliStream.offset != 0)) {
            // the setting vanished mid-stream (cannot happen with a static
            // settings table, but never wedge the streamer on it)
            static const char infoFailed[] = "INFO FAILED";
            customLinkCliStartBuffered(cliStream.reqId, CL_CLI_STATUS_ERR, infoFailed, sizeof(infoFailed) - 1);
            return;
        }
        if (cliStream.totalLen == 0) {
            cliStream.totalLen = (uint16_t)totalLen;
        }
        cliEngineWorked = true;
        payloadLen = clCliResponseHeader(&resp, cliStream.reqId, cliStream.totalLen,
            cliStream.offset, CL_CLI_STATUS_OK, (uint8_t)written);
#else
        return;
#endif
    } else {
        // header sizing clamps n to the remaining bytes of the snapshot
        payloadLen = clCliResponseHeader(&resp, cliStream.reqId, cliStream.totalLen,
            cliStream.offset, cliStream.status, CL_CLI_MAX_DATA);
        memcpy(resp.data, cliStream.buf + cliStream.offset, payloadLen - 8);
    }

    if (customLinkTrySendFrame(CUSTOM_LINK_MSG_FC_CLI, &resp, payloadLen)) {
        cliStream.offset += payloadLen - 8;
        cliStream.active = cliStream.offset < cliStream.totalLen;
    }
}

// Executes a validated 0x24 request against the settings table. Runs in task
// context exactly like the MSP handler for MSP2_CLI_SETTING; writes are
// RAM-only and range-checked by the shared CLI engine.
static void customLinkProcessCliRequest(const clFrame_t *frame)
{
    clCliRequest_t req;
    if (!clCliParseRequest(frame, &req)) {
        static const char badRequest[] = "BAD REQUEST";
        customLinkCliStartBuffered(req.reqId, CL_CLI_STATUS_ERR, badRequest, sizeof(badRequest) - 1);
        return;
    }

#if defined(USE_CLI)
    cliEngineWorked = true;
    switch (req.cmd) {
    case CL_CLI_CMD_GET: {
        const int written = cliGetSettingByName(req.args, cliStream.buf, sizeof(cliStream.buf));
        if (written < 0) {
            static const char notFound[] = "NOT FOUND";
            customLinkCliStartBuffered(req.reqId, CL_CLI_STATUS_ERR, notFound, sizeof(notFound) - 1);
        } else {
            customLinkCliStartSnapshot(req.reqId, CL_CLI_STATUS_OK, (uint16_t)written);
        }
        break;
    }

    case CL_CLI_CMD_SET: {
        if (cliSetSettingByName(req.args)) {
            // read back the new value for a request/response contract like
            // MSP2_CLI_SETTING: strip "= value", trim, query by bare name
            char nameOnly[CUSTOM_LINK_MAX_PAYLOAD];
            strncpy(nameOnly, req.args, sizeof(nameOnly) - 1);
            nameOnly[sizeof(nameOnly) - 1] = '\0';
            char *eq = strchr(nameOnly, '=');
            if (eq != NULL) {
                while (eq > nameOnly && *(eq - 1) == ' ') {
                    eq--;
                }
                *eq = '\0';
                const int written = cliGetSettingByName(nameOnly, cliStream.buf, sizeof(cliStream.buf));
                if (written >= 0) {
                    customLinkCliStartSnapshot(req.reqId, CL_CLI_STATUS_OK, (uint16_t)written);
                    break;
                }
            }
            // set succeeded but the readback failed: acknowledge with the request
            customLinkCliStartBuffered(req.reqId, CL_CLI_STATUS_OK, req.args, (uint16_t)strlen(req.args));
        } else {
            static const char setFailed[] = "SET FAILED";
            customLinkCliStartBuffered(req.reqId, CL_CLI_STATUS_ERR, setFailed, sizeof(setFailed) - 1);
        }
        break;
    }

    case CL_CLI_CMD_INFO: {
        // minimal probe window: unknown names return -1 before stream state
        // is touched; the probe also reports the full response length
        char probe[1];
        int totalLen = 0;
        if (req.args[0] == '\0' ||
            cliGetSettingInfoByName(req.args, 0, probe, sizeof(probe), &totalLen) < 0) {
            static const char notFound[] = "NOT FOUND";
            customLinkCliStartBuffered(req.reqId, CL_CLI_STATUS_ERR, notFound, sizeof(notFound) - 1);
        } else {
            customLinkCliStartInfo(req.reqId, req.args);
        }
        break;
    }

    case CL_CLI_CMD_BIND_RX: {
        // Same engine as the serial CLI "bind_rx": put the receiver (CRSF/
        // SRXL2/SPI) into pairing mode. Bounded, no settings-engine work.
        // Builds without any bind-capable RX provider (e.g. SITL) answer
        // NOT SUPPORTED rather than pulling in dead code.
#if defined(USE_RX_BIND)
        if (startRxBind()) {
            static const char binding[] = "binding...";
            customLinkCliStartBuffered(req.reqId, CL_CLI_STATUS_OK, binding, sizeof(binding) - 1);
        } else {
            static const char notSupported[] = "NOT SUPPORTED";
            customLinkCliStartBuffered(req.reqId, CL_CLI_STATUS_ERR, notSupported, sizeof(notSupported) - 1);
        }
#else
        static const char notSupported[] = "NOT SUPPORTED";
        customLinkCliStartBuffered(req.reqId, CL_CLI_STATUS_ERR, notSupported, sizeof(notSupported) - 1);
#endif
        break;
    }

    case CL_CLI_CMD_SAVE: {
        // CLI "save": persist the RAM config (writeEEPROM, exactly what
        // MSP_EEPROM_WRITE does) and, unless args = "noreboot", reboot like
        // the serial CLI. Refused while armed - a reboot mid-flight is
        // catastrophic and a flash write can stall the control loop.
        if (ARMING_FLAG(ARMED)) {
            static const char armedErr[] = "ARMED";
            customLinkCliStartBuffered(req.reqId, CL_CLI_STATUS_ERR, armedErr, sizeof(armedErr) - 1);
            break;
        }
        case CL_CLI_CMD_APPLY_PID: {
            const uint8_t profile = getCurrentPidProfileIndex();
            if (strcmp(req.args, "angle") == 0) {
#ifdef USE_ACC
                static const char capability[] = "ANGLE OFFBOARD 1";
#else
                static const char capability[] = "ANGLE NOT SUPPORTED";
#endif
                customLinkCliStartBuffered(req.reqId, CL_CLI_STATUS_OK, capability, sizeof(capability) - 1);
                break;
            }
            if (strcmp(req.args, "check") != 0) {
                if (strlen(req.args) != 1 || req.args[0] != '0' + profile) {
                    static const char wrongProfile[] = "PID PROFILE MISMATCH";
                    customLinkCliStartBuffered(req.reqId, CL_CLI_STATUS_ERR, wrongProfile, sizeof(wrongProfile) - 1);
                    break;
                }
                if (!clPidApplyIsSafe(ARMING_FLAG(ARMED), FLIGHT_MODE(OFFBOARD_MODE),
                    customLinkIsControlFresh(), cachedThrottle, cachedRate, gyro.gyroADCf)) {
                    static const char busy[] = "PID APPLY REQUIRES ZERO OUTPUT";
                    customLinkCliStartBuffered(req.reqId, CL_CLI_STATUS_ERR, busy, sizeof(busy) - 1);
                    break;
                }
                pidInitConfig(currentPidProfile);
                pidResetIterm();
            }
            char response[] = "PID PROFILE 0";
            response[sizeof(response) - 2] = '0' + profile;
            customLinkCliStartBuffered(req.reqId, CL_CLI_STATUS_OK, response, sizeof(response) - 1);
            break;
        }
        static const char saving[] = "saving...";
        customLinkCliStartBuffered(req.reqId, CL_CLI_STATUS_OK, saving, sizeof(saving) - 1);
        // put the single-chunk reply on the wire NOW: the EEPROM write and
        // especially the reboot must not race the response
        customLinkCliPump();
        writeEEPROM();
        if (strcasecmp(req.args, "noreboot") != 0) {
            waitForSerialPortToFinishTransmitting(customLinkPort);
            motorShutdown();
            systemReset();
        }
        break;
    }
    }
#else
    // no settings table on this build: fail loudly so a host never mistakes
    // silence for success
    static const char noCli[] = "NO CLI";
    customLinkCliStartBuffered(req.reqId, CL_CLI_STATUS_ERR, noCli, sizeof(noCli) - 1);
#endif
}


static int16_t clScaleFloat(float v, float scale)
{
    return constrain((int32_t)lrintf(v * scale), INT16_MIN, INT16_MAX);
}

#ifdef SIMULATOR
// Simulator sensor injection. A simulator host replaces the real sensors by
// streaming 0x21/0x22/0x23 frames; they reach the firmware through the same
// virtual sensor drivers as the UDP FDM bridge (sitl.c), so scale/sign
// conventions are kept identical to that path: 1 dps = 16.4 gyro counts,
// 1 g = acc_1G counts, level accelerometer = +1000 mg on Z.

#define CL_SIM_GYRO_SCALE       16.4f   // sitl.c GYRO_SCALE
#define CL_SIM_QUAT_SCALE       16384.0f

static void customLinkInjectImu(const clPayloadHostImu_t *imu)
{
#if defined(USE_VIRTUAL_GYRO)
    if (virtualGyroDev) {
        int16_t gyroAdc[XYZ_AXIS_COUNT];
        for (int axis = 0; axis < XYZ_AXIS_COUNT; axis++) {
            const float dps = imu->gyro[axis] * 0.1f;
            gyroAdc[axis] = constrain((int32_t)lrintf(dps * CL_SIM_GYRO_SCALE), INT16_MIN, INT16_MAX);
        }
        virtualGyroSet(virtualGyroDev, gyroAdc[X], gyroAdc[Y], gyroAdc[Z]);
    }
#endif
#if defined(USE_VIRTUAL_ACC)
    if (virtualAccDev) {
        int16_t accAdc[XYZ_AXIS_COUNT];
        for (int axis = 0; axis < XYZ_AXIS_COUNT; axis++) {
            const float g = imu->acc[axis] / 1000.0f;
            accAdc[axis] = constrain((int32_t)lrintf(g * acc.dev.acc_1G), INT16_MIN, INT16_MAX);
        }
        virtualAccSet(virtualAccDev, accAdc[X], accAdc[Y], accAdc[Z]);
    }
#endif

    const float qw = imu->quat[0] / CL_SIM_QUAT_SCALE;
    const float qx = imu->quat[1] / CL_SIM_QUAT_SCALE;
    const float qy = imu->quat[2] / CL_SIM_QUAT_SCALE;
    const float qz = imu->quat[3] / CL_SIM_QUAT_SCALE;

#if defined(USE_VIRTUAL_MAG)
    {
        // Synthetic earth field, NWU, 4096 counts (same model as sitl.c):
        // 60 deg inclination, 2 deg east declination so every body component
        // is generically nonzero. v_body = R(q)^T * v_world.
        static const float fieldN = 2046.8f;
        static const float fieldW = -71.5f;
        static const float fieldU = -3547.2f;
        const float r00 = 1.0f - 2.0f * (qy * qy + qz * qz);
        const float r01 = 2.0f * (qx * qy - qw * qz);
        const float r02 = 2.0f * (qx * qz + qw * qy);
        const float r10 = 2.0f * (qx * qy + qw * qz);
        const float r11 = 1.0f - 2.0f * (qx * qx + qz * qz);
        const float r12 = 2.0f * (qy * qz - qw * qx);
        const float r20 = 2.0f * (qx * qz - qw * qy);
        const float r21 = 2.0f * (qy * qz + qw * qx);
        const float r22 = 1.0f - 2.0f * (qx * qx + qy * qy);
        const float magX = r00 * fieldN + r10 * fieldW + r20 * fieldU;
        const float magY = r01 * fieldN + r11 * fieldW + r21 * fieldU;
        const float magZ = r02 * fieldN + r12 * fieldW + r22 * fieldU;
        virtualMagSet(lrintf(magX), lrintf(magY), lrintf(magZ));
    }
#endif

#if !defined(USE_IMU_CALC)
    // Direct-attitude builds (SITL_ATTITUDE_DIRECT); by default SITL runs the
    // real estimator on the injected gyro/acc/mag instead.
    imuSetAttitudeQuat(qw, qx, qy, qz);
#else
    UNUSED(qw);
    UNUSED(qx);
    UNUSED(qy);
    UNUSED(qz);
#endif
}

static void customLinkInjectEnv(const clPayloadHostEnv_t *env)
{
#if defined(USE_VIRTUAL_BARO)
    if (env->baro_pa != 0) {
        virtualBaroSet((int32_t)env->baro_pa, env->temp_cdeg);
    }
#endif
#if defined(USE_VIRTUAL_GPS)
    if (env->fix != 0 && env->sats != 0) {
        const double latitude = (double)env->lat_e7 / 1e7;
        const double longitude = (double)env->lon_e7 / 1e7;
        const double altitude = (double)env->alt_msl_cm / 100.0;
        const double velN = (double)env->vel_ned[0] / 100.0;
        const double velE = (double)env->vel_ned[1] / 100.0;
        const double velD = (double)env->vel_ned[2] / 100.0;
        const double speed = sqrt(velN * velN + velE * velE);
        const double speed3D = sqrt(velN * velN + velE * velE + velD * velD);
        const double course = (double)env->course_cdeg / 100.0;
        setVirtualGPS(latitude, longitude, altitude, speed, speed3D, course, velN, velE, velD);
        setVirtualGPSSatellites(env->sats);
    }
#endif
}

// Last accepted 0x23 RC injection (millis): feeds only the 0x11 link-quality
// report; deliberately NOT the rxUdpBridgeRcFresh() arbitration input.
static uint32_t lastHostRcMs;

static void customLinkInjectRc(const clPayloadHostRc_t *rc)
{
#if ENABLE_RX_UDP
    // RC arbitration: a live UDP RC bridge (host joystick tool on the
    // simulator's port 9004) owns the channels while it streams, so two
    // sources cannot fight over the shared latch. After pilot takeover,
    // fallback HOST_RC must not undo it when the bridge goes quiet.
    // NOTE: do not mark rxUdpBridgeRcFresh() from here - that freshness IS
    // the arbitration input and would self-block subsequent 0x23 frames.
    if (rxUdpBridgeRcFresh() || controlState.apiBlocked) {
        return;
    }
    // Same entry point as the UDP RC bridge: latches the channel array and
    // marks one RC frame complete for the rx task. Copied out of the packed
    // payload to avoid taking the address of a packed member. lastHostRcMs
    // only feeds the 0x11 link-quality report (accepted 0x23 frames count
    // as a live simulator RC link too).
    uint16_t channels[ARRAYLEN(rc->rc)];
    memcpy(channels, rc->rc, sizeof(channels));
    rxUpdateUdpChannels(channels, ARRAYLEN(channels));
    lastHostRcMs = millis();
#else
    UNUSED(rc);
#endif
}
#endif // SIMULATOR

static void customLinkProcessUplinkFrame(const clFrame_t *frame, timeUs_t t2Us)
{
    switch (frame->msgId) {
    case CUSTOM_LINK_MSG_HOST_CONTROL: {
        const clEvent_e event = clControlApplyFrame(&controlState, frame, millis());
        for (int axis = 0; axis < XYZ_AXIS_COUNT; axis++) {
            cachedRate[axis] = controlState.rateX10[axis] * 0.1f;
        }
        cachedThrottle = (controlState.throttle - 1000) * 0.001f;

        // After pilot takeover, stale host disarm requests have no authority.
        if (event == CL_EVENT_DISARM_REQUEST &&
            (customLinkIsModeRequested() || FLIGHT_MODE(OFFBOARD_MODE))) {
            disarm(DISARM_REASON_OFFBOARD);
        }
        break;
    }

    case CUSTOM_LINK_MSG_HOST_TIMESYNC: {
        clPayloadTimesyncReq_t req;
        if (frame->len == sizeof(req)) {
            memcpy(&req, frame->payload, sizeof(req));
            syncPending = true;
            syncT1 = req.t1;
            syncT2Us = (uint32_t)t2Us;
        }
        break;
    }

    case CUSTOM_LINK_MSG_HOST_CLI:
        customLinkProcessCliRequest(frame);
        break;

#ifdef SIMULATOR
    case CUSTOM_LINK_MSG_HOST_IMU: {
        clPayloadHostImu_t imu;
        if (frame->len == sizeof(imu)) {
            memcpy(&imu, frame->payload, sizeof(imu));
            customLinkInjectImu(&imu);
            CL_STAT_INC(clStatImu);
        }
        break;
    }

    case CUSTOM_LINK_MSG_HOST_ENV: {
        clPayloadHostEnv_t env;
        if (frame->len == sizeof(env)) {
            memcpy(&env, frame->payload, sizeof(env));
            customLinkInjectEnv(&env);
            CL_STAT_INC(clStatEnv);
        }
        break;
    }

    case CUSTOM_LINK_MSG_HOST_RC: {
        clPayloadHostRc_t rc;
        if (frame->len == sizeof(rc)) {
            memcpy(&rc, frame->payload, sizeof(rc));
            customLinkInjectRc(&rc);
            CL_STAT_INC(clStatRc);
        }
        break;
    }
#endif // SIMULATOR

    default:
        break;
    }
}

void customLinkInit(void)
{
    clControlReset(&controlState);

    const serialPortConfig_t *portConfig = findSerialPortConfig(FUNCTION_CUSTOM_LINK);
    if (!portConfig) {
        return;
    }

    baudRate_e baudIndex = portConfig->telemetry_baudrateIndex;
    if (baudIndex == BAUD_AUTO) {
        baudIndex = BAUD_921600;
    }
    const uint32_t baud = baudRates[baudIndex];

    // inter-byte resync timeout: twice the on-wire time of a maximum frame
    timeUs_t interByteTimeoutUs = (timeUs_t)(((uint64_t)CUSTOM_LINK_FRAME_MAX * 10 * 2 * 1000000) / baud);
    interByteTimeoutUs = MAX(interByteTimeoutUs, (timeUs_t)2000);
#ifdef SIMULATOR
    // the SITL transport is polled at task rate, not per byte
    interByteTimeoutUs = MAX(interByteTimeoutUs, (timeUs_t)20000);
#endif
    clParserInit(&parser, interByteTimeoutUs);

    customLinkPort = openSerialPort(portConfig->identifier, FUNCTION_CUSTOM_LINK,
                                    customLinkRxCallback, NULL, baud,
                                    MODE_RXTX, SERIAL_NOT_INVERTED);
    linkEnabled = (customLinkPort != NULL);
}

bool customLinkIsEnabled(void)
{
    return linkEnabled;
}

bool customLinkIsControlFresh(void)
{
    return clControlIsFresh(&controlState, millis(), customLinkConfig()->watchdog_ms);
}

bool customLinkHasControl(void)
{
    return controlActive && FLIGHT_MODE(OFFBOARD_MODE);
}

void customLinkUpdateRcModes(bool pilotModesValid)
{
    uint32_t pilotModes = 0;
    for (boxId_e mode = BOXANGLE; mode < BOXOFFBOARD; mode++) {
        if (IS_RC_MODE_ACTIVE(mode)) {
            pilotModes |= 1U << mode;
        }
    }
    clControlUpdateAuthority(&controlState, pilotModes, pilotModesValid,
        IS_RC_MODE_ACTIVE(BOXOFFBOARD), IS_RC_MODE_ACTIVE(BOXGPSRESCUE),
        IS_RC_MODE_ACTIVE(BOXARM), ARMING_FLAG(ARMED), customLinkIsControlFresh());
}

bool customLinkIsModeRequested(void)
{
    return clControlModeRequested(&controlState);
}

bool customLinkHostArmActive(void)
{
    return linkEnabled && clControlHostArmActive(&controlState,
        ARMING_FLAG(ARMED), customLinkIsControlFresh());
}

float customLinkGetRateSetpoint(int axis)
{
    float rate = cachedRate[axis];

    // Host rate setpoint clamp (deg/s). Applied ahead of the PID loop so the
    // envelope holds regardless of what the host sends; 0 disables.
    const uint16_t rateLimit = customLinkConfig()->rate_limit_dps;
    if (rateLimit != 0) {
        rate = constrainf(rate, -(float)rateLimit, (float)rateLimit);
    }

    // Roll/pitch attitude envelope: once the euler angle leaves the limit,
    // commands that would bank further in the same direction are cut to zero
    // (recovery-direction commands still pass). Betaflight attitude signs are
    // aligned with their rate axes (roll: right positive, pitch: nose-down
    // positive), so the direction comparison is direct. 0 disables.
    const uint16_t angleLimit = customLinkConfig()->angle_limit_deg;
    if (axis < FD_YAW && angleLimit != 0) {
        const float angle = (axis == FD_ROLL ? attitude.values.roll : attitude.values.pitch) * 0.1f;
        if ((angle > (float)angleLimit && rate > 0.0f) ||
            (angle < -(float)angleLimit && rate < 0.0f)) {
            rate = 0.0f;
        }
    }

    return rate;
}

bool customLinkAngleRequested(void)
{
    return controlState.modeReq == 3 && customLinkIsControlFresh() && customLinkIsModeRequested();
}

float customLinkGetAngleSetpoint(int axis)
{
    return constrainf(cachedRate[axis], -10.0f, 10.0f);
}

float customLinkGetThrottle(void)
{
    return cachedThrottle;
}

void customLinkTaskFast(timeUs_t currentTimeUs)
{
    if (!linkEnabled) {
        return;
    }

    customLinkDrainBufferedRx();

    // drain uplink frames completed in the ISR
    while (rxRingTail != rxRingHead) {
        const clRxRingEntry_t entry = rxRing[rxRingTail];
        rxRingTail = (rxRingTail + 1) % CL_RX_RING_LEN;
        customLinkProcessUplinkFrame(&entry.frame, entry.t2Us);
    }

    // uplink observability export (DEBUG_CUSTOM_LINK): truncated to u16.
    // Relaxed atomic loads - an aligned u32 store is single-copy atomic
    // anyway, this keeps the cross-context contract explicit.
    DEBUG_SET(DEBUG_CUSTOM_LINK, 0, (int16_t)__atomic_load_n(&clStatFrames, __ATOMIC_RELAXED));
    DEBUG_SET(DEBUG_CUSTOM_LINK, 1, (int16_t)__atomic_load_n(&clStatEnv, __ATOMIC_RELAXED));
    DEBUG_SET(DEBUG_CUSTOM_LINK, 2, (int16_t)__atomic_load_n(&clStatImu, __ATOMIC_RELAXED));
    DEBUG_SET(DEBUG_CUSTOM_LINK, 3, (int16_t)__atomic_load_n(&clStatRc, __ATOMIC_RELAXED));
    DEBUG_SET(DEBUG_CUSTOM_LINK, 4, (int16_t)__atomic_load_n(&clStatRingDrops, __ATOMIC_RELAXED));
    DEBUG_SET(DEBUG_CUSTOM_LINK, 5, (int16_t)__atomic_load_n(&clStatBytes, __ATOMIC_RELAXED));

    // authority is recomputed every task period so a watchdog expiry stops
    // the PID/mixer override immediately, ahead of the next processRxModes
    controlActive = FLIGHT_MODE(OFFBOARD_MODE) && customLinkIsControlFresh();

    // time sync response - T3 is captured as late as possible, right before TX
    if (syncPending) {
        syncPending = false;
        clPayloadTimesyncResp_t resp;
        resp.t1 = syncT1;
        resp.t2_isr_us = syncT2Us;
        resp.t3_tx_us = (uint32_t)micros();
        customLinkSendFrame(CUSTOM_LINK_MSG_FC_TIMESYNC, &resp, sizeof(resp));
    }

    // 200 Hz IMU and attitude stream
    clPayloadFast_t fast;
    memset(&fast, 0, sizeof(fast));
    fast.ts_us = (uint32_t)currentTimeUs;
    for (int axis = 0; axis < XYZ_AXIS_COUNT; axis++) {
        fast.gyro[axis] = clScaleFloat(gyro.gyroADCf[axis], 10.0f);                 // 0.1 deg/s
        fast.acc[axis] = clScaleFloat(acc.accADC.v[axis] * 1000.0f / acc.dev.acc_1G, 1.0f); // 1 mg
    }
    fast.attitude[0] = clScaleFloat(attitude.values.roll, 10.0f);                   // 0.01 deg
    fast.attitude[1] = clScaleFloat(attitude.values.pitch, 10.0f);
    // Betaflight attitude.values.yaw 是罗盘航向 (0..360, 顺时针为正)。协议输出
    // 改为右手系数学角：绕机体 +Z(Up) 逆时针为正，0 = 磁北，±180 deg 折返，
    // 同时满足 i16 (±0.01 deg) 编码范围。
    int32_t yawCd = -(int32_t)attitude.values.yaw * 10;                             // -36000..0
    while (yawCd > 18000) {
        yawCd -= 36000;
    }
    while (yawCd < -18000) {
        yawCd += 36000;
    }
    fast.attitude[2] = constrain(yawCd, INT16_MIN, INT16_MAX);
    customLinkSendFrame(CUSTOM_LINK_MSG_FC_FAST, &fast, sizeof(fast));

#ifdef USE_MOTOR
    // Mixer motor outputs for the simulator (0x13). External values in the us
    // domain, exactly what MSP_MOTOR reports, so a host normalizes the same
    // way for both protocols. Opt-in via custom_link_motors_stream (default
    // on for SITL builds, off for real hardware).
    if (customLinkConfig()->motors_stream && motorIsEnabled()) {
        clPayloadMotors_t motors;
        memset(&motors, 0, sizeof(motors));
        motors.count = getMotorCount();
        if (motors.count > ARRAYLEN(motors.motor)) {
            motors.count = ARRAYLEN(motors.motor);
        }
        for (uint8_t i = 0; i < motors.count; i++) {
            const uint16_t value = motorIsMotorEnabled(i) ? motorConvertToExternal(motor[i]) : 0;
            motors.motor[i] = value;
        }
        customLinkSendFrame(CUSTOM_LINK_MSG_FC_MOTORS, &motors, sizeof(motors));
    }
#endif

    // CLI read/write response chunks (0x15), paced one per task period
    customLinkCliPump();

    // 串口写路径的耗时抖动不应抬调度器对本任务的时间预算需求
    //（8 kHz gyro 节奏下凑不出大预算窗口，会饥饿降频）。例外：CLI 设置
    // 引擎的耗时（查表扫描/PG 拷贝，几十 µs 量级突发）必须如实计入预算，
    // 否则调度器以为本任务近乎零耗时、贴着 gyro 死线排入，而统计看不见。
    if (!cliEngineWorked) {
        schedulerIgnoreTaskExecTime();
    }
    cliEngineWorked = false;
}

// 温度来源优先级：气压计内部温度（DPS310 等用于补偿的实测值）> IMU 温度 >
// ISA 估计（海平面 20 degC，每上升 100 m 降 0.6 degC）。读不到以 0 为无效判据
// （实测环境不会恰好 0.00 degC；无温度寄存器支持的驱动保持为 0）。
static int32_t customLinkTemperatureCdeg(void)
{
    if (baro.temperature != 0) {
        return baro.temperature;                              // already centidegrees
    }
    const int16_t imuTemp = gyroGetTemperature();            // whole degrees C
    if (imuTemp != 0) {
        return (int32_t)imuTemp * 100;
    }
    return (int32_t)lrintf(2000.0f - getBaroAltitude() * 0.006f);
}

void customLinkTaskMed(timeUs_t currentTimeUs)
{
    if (!linkEnabled) {
        return;
    }

    clPayloadMedium_t med;
    memset(&med, 0, sizeof(med));
    med.ts_us = (uint32_t)currentTimeUs;
    med.baro_pa = (uint32_t)baro.pressure;
    med.baro_alt_cm = (int32_t)lrintf(getBaroAltitude());
    med.temp_cdeg = constrain(customLinkTemperatureCdeg(), INT16_MIN, INT16_MAX); // 0.01 degC
    for (int chan = 0; chan < 16 && chan < MAX_SUPPORTED_RC_CHANNEL_COUNT; chan++) {
        med.rc[chan] = (uint16_t)lrintf(rcData[chan]);
    }
#if ENABLE_RX_UDP
    // UDP 桥 RC 无真实射频：按帧新鲜度上报链路质量（新鲜=100，停流=0），
    // feeder 停流时下游在 failsafe 触发前就能看到质量归零；被接受的 0x23
    // 注入帧同样算活跃链路。真机走 rx 的 RSSI 源（getRssiPercent 0..100）。
    med.rssi_pct = (rxUdpBridgeRcFresh() || millis() - lastHostRcMs < 100) ? 100 : 0;
#else
    med.rssi_pct = rxGetLinkQualityPercent() & 0xFF;
#endif
    customLinkSendFrame(CUSTOM_LINK_MSG_FC_MEDIUM, &med, sizeof(med));

#ifdef USE_MOTOR
    // ESC motor RPM telemetry (0x16). Bidirectional DShot (Bluejay-class ESCs)
    // reports eRPM per motor; the FC converts it to mechanical rpm via the
    // motor_poles setting so hosts never need the pole count. With telemetry
    // off the valid mask stays zero and the frame still flows so hosts see a
    // stable cadence.
    if (motorIsEnabled()) {
        clPayloadEscTelemetry_t esc;
        memset(&esc, 0, sizeof(esc));
        esc.ts_us = (uint32_t)currentTimeUs;
        esc.count = getMotorCount();
        if (esc.count > ARRAYLEN(esc.rpm)) {
            esc.count = ARRAYLEN(esc.rpm);
        }
        for (uint8_t i = 0; i < esc.count; i++) {
#if defined(USE_DSHOT_TELEMETRY)
            if (isDshotMotorTelemetryActive(i)) {
                esc.valid |= 1 << i;
                const uint32_t rpm = (uint32_t)lrintf(erpmToRpm(getDshotErpm(i)));
                esc.rpm[i] = rpm > UINT16_MAX ? UINT16_MAX : (uint16_t)rpm;
            }
#endif
        }
        customLinkSendFrame(CUSTOM_LINK_MSG_FC_ESC_TELEMETRY, &esc, sizeof(esc));
    }
#endif

#ifdef USE_GPS
    positionEstimate3d_t estimate;
    uint16_t originId;
    if (positionEstimatorGetArmOdometry(&estimate, &originId)) {
        clPayloadOdometry_t odom = {0};
        odom.ts_us = (uint32_t)currentTimeUs;
        odom.flags = CL_ODOM_VALID_FLAGS;
        odom.origin_id = originId;
        odom.arm_yaw_cdeg = positionEstimatorGetArmYawCdeg();
        bool valid = true;
        for (int axis = 0; axis < XYZ_AXIS_COUNT; axis++) {
            const float position = estimate.position.v[axis];
            const float velocity = estimate.velocity.v[axis];
            valid &= isfinite(position) && fabsf(position) < INT32_MAX &&
                isfinite(velocity) && fabsf(velocity) <= INT16_MAX &&
                isfinite(gyro.gyroADCf[axis]) &&
                isfinite(estimate.positionVariance.v[axis]) && estimate.positionVariance.v[axis] >= 0.0f &&
                isfinite(estimate.velocityVariance.v[axis]) && estimate.velocityVariance.v[axis] >= 0.0f;
            if (!valid) {
                break;
            }
            odom.position_cm[axis] = lrintf(position);
            odom.velocity_cms[axis] = lrintf(velocity);
            odom.attitude_cdeg[axis] = attitude.raw[axis] * 10;
            odom.gyro_dps_x10[axis] = constrain(lrintf(gyro.gyroADCf[axis] * 10.0f), INT16_MIN, INT16_MAX);
            odom.position_variance[axis] = estimate.positionVariance.v[axis] * 0.0001f;
            odom.velocity_variance[axis] = estimate.velocityVariance.v[axis] * 0.0001f;
        }
        // Keep odometry attitude identical to 0x10, not native clockwise compass heading.
        int32_t yawCd = -(int32_t)attitude.values.yaw * 10;
        if (yawCd < -18000) {
            yawCd += 36000;
        }
        odom.attitude_cdeg[FD_YAW] = yawCd;
        if (valid) {
            customLinkSendFrame(CUSTOM_LINK_MSG_FC_ODOMETRY, &odom, sizeof(odom));
        }
    }
#endif

    schedulerIgnoreTaskExecTime();
}

void customLinkTaskSlow(timeUs_t currentTimeUs)
{
    if (!linkEnabled) {
        return;
    }

    clPayloadSlow_t slow;
    memset(&slow, 0, sizeof(slow));
    slow.ts_us = (uint32_t)currentTimeUs;
    slow.vbat_mv = getBatteryVoltage() * 10;                                        // 0.01 V -> mV
    slow.current_ma = getAmperage() * 10;                                           // 0.01 A -> mA
    const int32_t mAh = getMAhDrawn();
    slow.mah = mAh > INT16_MAX ? INT16_MAX : (int16_t)mAh;
#ifdef USE_GPS
    slow.fix = gpsHas3DFix() ? 1 : 0;
#endif
    slow.sats = gpsSol.numSat;
    slow.lat_e7 = gpsSol.llh.lat;
    slow.lon_e7 = gpsSol.llh.lon;
    slow.alt_msl_cm = gpsSol.llh.altCm;
    slow.gspeed_cms = gpsSol.groundSpeed;
    slow.course_cdeg = gpsSol.groundCourse * 10;                                    // 0.1 deg -> 0.01 deg
    slow.mode_flags = flightModeFlags;
    slow.status = (ARMING_FLAG(ARMED) ? CL_STATUS_ARMED : 0) |
                  (failsafeIsActive() ? CL_STATUS_FAILSAFE : 0);
    customLinkSendFrame(CUSTOM_LINK_MSG_FC_SLOW, &slow, sizeof(slow));

#ifdef SIMULATOR
    // Pure custom-link SITL sessions never run updateState()'s 1 Hz status
    // print (no UDP FDM arrives), so surface arming blockers here instead.
    static uint32_t lastArmingDebugMs;
    const uint32_t nowMs = millis();
    if (cmp32(nowMs, lastArmingDebugMs) >= 1000 && ARMING_FLAG(ARMED) == 0) {
        lastArmingDebugMs = nowMs;
        const armingDisableFlags_e flags = getArmingDisableFlags();
        if (flags != 0) {
            printf("[SITL] t=%dms Arming disabled:", (int)millis());
            for (unsigned i = 0; i < ARMING_DISABLE_FLAGS_COUNT; i++) {
                const armingDisableFlags_e flag = (1 << i);
                if (flags & flag) {
                    printf(" %s", getArmingDisableFlagName(flag));
                }
            }
            printf("\n");
        }
    }
#endif

    schedulerIgnoreTaskExecTime();
}

#endif // USE_CUSTOM_LINK
