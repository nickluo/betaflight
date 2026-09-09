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

#if defined(USE_CUSTOM_LINK)

#include "common/axis.h"
#include "common/maths.h"
#include "common/time.h"
#include "common/utils.h"

#include "drivers/serial.h"
#include "drivers/time.h"

#include "scheduler/scheduler.h"

#include "fc/core.h"
#include "fc/rc_modes.h"
#include "fc/runtime_config.h"

#include "flight/failsafe.h"
#include "flight/imu.h"

#include "io/gps.h"
#include "io/serial.h"

#include "pg/custom_link.h"

#include "rx/rx.h"

#include "sensors/acceleration.h"
#include "sensors/barometer.h"
#include "sensors/battery.h"
#include "sensors/gyro.h"

#include "telemetry/custom_link.h"
#include "telemetry/custom_link_protocol.h"

#define CL_RX_RING_LEN          4       // completed uplink frames buffered from ISR to task

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

// pending time sync response, task context only
static bool syncPending;
static uint64_t syncT1;
static uint32_t syncT2Us;

static void customLinkFrameHandler(const clFrame_t *frame, timeUs_t frameCompleteUs, void *ctx)
{
    UNUSED(ctx);

    const uint8_t next = (rxRingHead + 1) % CL_RX_RING_LEN;
    if (next == rxRingTail) {
        return;                        // ring full: drop the frame
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
    while (serialRxBytesWaiting(customLinkPort) > 0) {
        const uint8_t c = serialRead(customLinkPort);
        clParserProcessByte(&parser, c, micros(), customLinkFrameHandler, NULL);
    }
}

static void customLinkSendFrame(uint8_t msgId, const void *payload, uint8_t payloadLen)
{
    uint8_t buf[CUSTOM_LINK_FRAME_MAX];
    const uint8_t len = clEncodeFrame(msgId, downlinkSeq++, payload, payloadLen, buf, sizeof(buf));
    if (len == 0) {
        return;
    }
    // serialWriteBuf busy-waits for buffer space on a stalled transport
    // (no TCP client, slow host); a wedged task would freeze the whole
    // cooperative scheduler. Degrade to dropping the frame instead.
    if (serialTxBytesFree(customLinkPort) < (uint32_t)len) {
        return;
    }
    serialWriteBuf(customLinkPort, buf, len);
}

static int16_t clScaleFloat(float v, float scale)
{
    return constrain((int32_t)lrintf(v * scale), INT16_MIN, INT16_MAX);
}

static void customLinkProcessUplinkFrame(const clFrame_t *frame, timeUs_t t2Us)
{
    switch (frame->msgId) {
    case CUSTOM_LINK_MSG_HOST_CONTROL: {
        const clEvent_e event = clControlApplyFrame(&controlState, frame, millis());
        for (int axis = 0; axis < XYZ_AXIS_COUNT; axis++) {
            cachedRate[axis] = controlState.rateX10[axis] * 0.1f;
        }
        cachedThrottle = (controlState.throttle - 1000) * 0.001f;

        // Host arming authority is conditional: arm requests only take effect
        // while the pilot holds the BOXOFFBOARD switch and still pass every
        // tryArm() safety check. Disarm is honored for as long as the host
        // holds authority (switch on or mode engaged).
        if (event == CL_EVENT_DISARM_REQUEST &&
            (IS_RC_MODE_ACTIVE(BOXOFFBOARD) || FLIGHT_MODE(OFFBOARD_MODE))) {
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
    return controlActive;
}

bool customLinkHostArmActive(void)
{
    // Level semantics: a fresh stream holding arm=1 counts as an armed
    // request equivalent to the pilot's ARM switch (processRcStickPositions
    // disarms whenever the ARM box is inactive, which would otherwise fight
    // and flap against host-initiated arming).
    return linkEnabled && controlState.arm && customLinkIsControlFresh();
}

float customLinkGetRateSetpoint(int axis)
{
    return cachedRate[axis];
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

    // 串口写路径的耗时抖动不应抬调度器对本任务的时间预算需求
    //（8 kHz gyro 节奏下凑不出大预算窗口，会饥饿降频）。
    schedulerIgnoreTaskExecTime();
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
    customLinkSendFrame(CUSTOM_LINK_MSG_FC_MEDIUM, &med, sizeof(med));

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
    slow.fix = STATE(GPS_FIX) ? 1 : 0;
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

    schedulerIgnoreTaskExecTime();
}

#endif // USE_CUSTOM_LINK
