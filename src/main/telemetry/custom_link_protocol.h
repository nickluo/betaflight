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

#pragma once

#if defined(USE_CUSTOM_LINK)

#include <stdbool.h>
#include <stdint.h>

#include "common/time.h"

//
// Custom high-frequency bidirectional link to a companion computer (host).
//
// Frame format (all multi-byte fields little-endian):
//
//   0xEB 0x90 | MsgID u8 | Len u8 | SeqNum u8 | Payload (Len bytes) | CRC16 lo hi
//
// CRC-16-CCITT (poly 0x1021, MSB-first), init 0 (CRC-16/XMODEM), computed over
// MsgID + Len + SeqNum + Payload and appended little-endian.
//
// Downlink SeqNum is a single FC-side counter shared by all message ids so the
// host can detect the loss of any downlink frame. Uplink frames are sequenced
// by the command_seq field inside the control payload.
//
// All payload structs below are packed and must only be memcpy'd to/from the
// frame payload; big-endian hosts must deserialize field by field.
//

#define CUSTOM_LINK_SYNC1            0xEB
#define CUSTOM_LINK_SYNC2            0x90

#define CUSTOM_LINK_FRAME_OVERHEAD   7                     // sync(2) + msgId + len + seq + crc16
#define CUSTOM_LINK_MAX_PAYLOAD      64
#define CUSTOM_LINK_FRAME_MAX        (CUSTOM_LINK_FRAME_OVERHEAD + CUSTOM_LINK_MAX_PAYLOAD)

typedef enum {
    CUSTOM_LINK_MSG_FC_FAST        = 0x10,   // 200 Hz gyro/acc/attitude
    CUSTOM_LINK_MSG_FC_MEDIUM      = 0x11,   // 100 Hz baro/temperature/rc channels
    CUSTOM_LINK_MSG_FC_SLOW        = 0x12,   // 10 Hz battery/gps/flight mode
    CUSTOM_LINK_MSG_HOST_CONTROL   = 0x20,   // 100-200 Hz control setpoints
    CUSTOM_LINK_MSG_HOST_TIMESYNC  = 0x30,   // time sync request (u64 T1)
    CUSTOM_LINK_MSG_FC_TIMESYNC    = 0x31,   // time sync response (T1, T2, T3)
} customLinkMsgId_e;

// 0x10 - 200 Hz IMU and attitude (22 bytes)
typedef struct __attribute__((packed)) {
    uint32_t ts_us;             // FC micros() timestamp
    int16_t gyro[3];            // filtered gyro, 0.1 deg/s
    int16_t acc[3];             // accelerometer, 1 mg
    int16_t attitude[3];        // roll/pitch/yaw euler angles, 0.01 deg
} clPayloadFast_t;

// 0x11 - 100 Hz barometer, temperature and RC channels (46 bytes)
typedef struct __attribute__((packed)) {
    uint32_t ts_us;             // FC micros() timestamp
    uint32_t baro_pa;           // absolute pressure, Pa
    int32_t baro_alt_cm;        // barometric altitude, cm
    int16_t imu_temp_cdeg;      // IMU temperature, 0.01 degC
    uint16_t rc[16];            // RC channel pulse widths, us (1000-2000)
} clPayloadMedium_t;

// 0x12 - 10 Hz power, GPS and survey status (36 bytes)
typedef struct __attribute__((packed)) {
    uint32_t ts_us;             // FC micros() timestamp
    uint16_t vbat_mv;           // battery voltage, mV
    int32_t current_ma;         // battery current, mA
    int16_t mah;                // consumed charge, mAh (saturated at 32767)
    uint8_t fix;                // 0 = no fix, 1 = fix (3D for ublox; 2D is not exposed)
    uint8_t sats;               // satellites in use
    int32_t lat_e7;             // latitude, deg * 1e7
    int32_t lon_e7;             // longitude, deg * 1e7
    int32_t alt_msl_cm;         // altitude above mean sea level, cm
    uint16_t gspeed_cms;        // ground speed, cm/s
    uint16_t course_cdeg;       // ground course, 0.01 deg
    uint32_t mode_flags;        // flightModeFlags (see flightModeFlags_e)
    uint16_t status;            // bit0 = armed, bit1 = failsafe active
} clPayloadSlow_t;

#define CL_STATUS_ARMED              0x0001
#define CL_STATUS_FAILSAFE           0x0002

// 0x20 - 100-200 Hz control command (20 bytes)
typedef struct __attribute__((packed)) {
    uint64_t host_ts_us;        // host monotonic timestamp at send time
    uint16_t cmd_seq;           // command sequence number
    uint8_t arm;                // 0 = request disarm, 1 = request arm
    uint8_t mode_req;           // 0 = manual/rate, 1 = angle, 2 = offboard override
    uint16_t throttle;          // throttle, us-style 1000-2000 (clamped)
    int16_t rate_x10[3];        // roll/pitch/yaw rate targets, 0.1 deg/s
} clPayloadControl_t;

// 0x30 - time sync request (8 bytes)
typedef struct __attribute__((packed)) {
    uint64_t t1;                // host timestamp when the request was sent
} clPayloadTimesyncReq_t;

// 0x31 - time sync response (16 bytes)
typedef struct __attribute__((packed)) {
    uint64_t t1;                // T1 echoed from the request
    uint32_t t2_isr_us;         // FC micros() captured in the RX ISR when the request completed
    uint32_t t3_tx_us;          // FC micros() captured immediately before the response was sent
} clPayloadTimesyncResp_t;

typedef struct {
    uint8_t msgId;
    uint8_t len;
    uint8_t seq;
    uint8_t payload[CUSTOM_LINK_MAX_PAYLOAD];
} clFrame_t;

//
// Encoder
//

// Computes the CRC of a frame header+payload the way clEncodeFrame appends it.
uint16_t clCrc(const clFrame_t *frame);

// Serializes one frame into out; returns the total frame size or 0 on error
// (payload too large or output buffer too small).
uint8_t clEncodeFrame(uint8_t msgId, uint8_t seq, const void *payload, uint8_t payloadLen, uint8_t *out, uint8_t outSize);

//
// Parser state machine
//
// Feed one received byte at a time together with a timestamp. Resynchronizes
// by preamble scanning, drops frames with an impossible length or a bad CRC,
// and resets on an inter-byte timeout. Valid frames are handed to handler with
// the timestamp of the byte that completed them (this is T2 for time sync).
//

typedef void (*clFrameHandler_t)(const clFrame_t *frame, timeUs_t frameCompleteUs, void *ctx);

typedef enum {
    CL_PARSER_SYNC1 = 0,
    CL_PARSER_SYNC2,
    CL_PARSER_HDR,
    CL_PARSER_PAYLOAD,
    CL_PARSER_CRC_LO,
    CL_PARSER_CRC_HI,
} clParserState_e;

typedef struct {
    uint8_t state;
    uint8_t hdrPos;
    uint8_t payloadPos;
    timeUs_t lastByteUs;
    timeUs_t interByteTimeoutUs;
    uint16_t crc;
    clFrame_t frame;
} clParser_t;

void clParserInit(clParser_t *parser, timeUs_t interByteTimeoutUs);
void clParserProcessByte(clParser_t *parser, uint8_t c, timeUs_t nowUs, clFrameHandler_t handler, void *ctx);

//
// Uplink control authority (pure logic, time injected for testability)
//

typedef enum {
    CL_EVENT_NONE = 0,
    CL_EVENT_ARM_REQUEST,       // host arm bit went 0 -> 1
    CL_EVENT_DISARM_REQUEST,    // host arm bit went 1 -> 0
} clEvent_e;

typedef struct {
    uint64_t lastHostTsUs;
    uint32_t lastFrameMs;
    uint16_t lastCmdSeq;
    uint8_t arm;
    uint16_t throttle;          // clamped to 1000-2000
    int16_t rateX10[3];
    bool haveFrame;
} clControlState_t;

void clControlReset(clControlState_t *state);

// Applies a (validated) host control frame. Only CUSTOM_LINK_MSG_HOST_CONTROL
// frames are accepted; returns the arm edge event, if any.
clEvent_e clControlApplyFrame(clControlState_t *state, const clFrame_t *frame, uint32_t nowMs);

// True while the most recent control frame is younger than watchdogMs.
bool clControlIsFresh(const clControlState_t *state, uint32_t nowMs, uint16_t watchdogMs);

#endif // USE_CUSTOM_LINK
