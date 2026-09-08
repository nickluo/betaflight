/*
 * This file is part of Cleanflight.
 *
 * Cleanflight is free software: you can redistribute this software and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation, either version 3 of the
 * License, or (at your option) any later version.
 *
 * Cleanflight is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
 * General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this software.
 *
 * If not, see <http://www.gnu.org/licenses/>.
 */

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include <string.h>

extern "C" {
    #include <platform.h>

    #include "common/axis.h"
    #include "common/crc.h"
    #include "common/time.h"

    #include "telemetry/custom_link_protocol.h"
}

#include "unittest_macros.h"
#include "gtest/gtest.h"

typedef struct {
    unsigned frameCount;
    clFrame_t lastFrame;
    timeUs_t lastFrameCompleteUs;
} testRxCapture_t;

static testRxCapture_t testRx;

static void testCaptureFrame(const clFrame_t *frame, timeUs_t frameCompleteUs, void *ctx)
{
    UNUSED(ctx);
    testRx.frameCount++;
    testRx.lastFrame = *frame;
    testRx.lastFrameCompleteUs = frameCompleteUs;
}

static clParser_t parser;

static void resetParser(timeUs_t interByteTimeoutUs)
{
    memset(&testRx, 0, sizeof(testRx));
    clParserInit(&parser, interByteTimeoutUs);
}

// Feed bytes one by one, advancing the clock by one step per byte, and return
// the timestamp at which the last byte was processed.
static timeUs_t feedBytes(const uint8_t *buf, uint8_t len, timeUs_t startUs, timeUs_t stepUs)
{
    timeUs_t t = startUs;
    for (uint8_t i = 0; i < len; i++) {
        clParserProcessByte(&parser, buf[i], t, testCaptureFrame, NULL);
        if (i + 1 < len) {
            t += stepUs;
        }
    }
    return t;
}

TEST(CustomLinkCrcTest, TestXmodemKnownAnswer)
{
    // CRC-16/XMODEM check value for the ASCII string "123456789"
    const char *data = "123456789";
    EXPECT_EQ(crc16_ccitt_update(0, data, 9), 0x31C3);
}

TEST(CustomLinkEncodeTest, TestFrameLayout)
{
    clPayloadTimesyncReq_t req = { .t1 = 0xDEADBEEFCAFE1234ULL };
    uint8_t buf[CUSTOM_LINK_FRAME_MAX];
    const uint8_t len = clEncodeFrame(CUSTOM_LINK_MSG_HOST_TIMESYNC, 42, &req, sizeof(req), buf, sizeof(buf));

    EXPECT_EQ(len, CUSTOM_LINK_FRAME_OVERHEAD + sizeof(req));
    EXPECT_EQ(buf[0], 0xEB);
    EXPECT_EQ(buf[1], 0x90);
    EXPECT_EQ(buf[2], CUSTOM_LINK_MSG_HOST_TIMESYNC);
    EXPECT_EQ(buf[3], (uint8_t)sizeof(req));
    EXPECT_EQ(buf[4], 42);

    // payload is a little-endian u64
    uint64_t t1 = 0;
    memcpy(&t1, &buf[5], sizeof(t1));
    EXPECT_EQ(t1, 0xDEADBEEFCAFE1234ULL);

    // appended CRC is recomputed over msgId+len+seq+payload, low byte first
    clFrame_t frame = {};
    frame.msgId = buf[2];
    frame.len = buf[3];
    frame.seq = buf[4];
    memcpy(frame.payload, &buf[5], frame.len);
    const uint16_t crc = clCrc(&frame);
    EXPECT_EQ(buf[len - 2], (uint8_t)(crc & 0xFF));
    EXPECT_EQ(buf[len - 1], (uint8_t)(crc >> 8));
}

TEST(CustomLinkEncodeTest, TestRejectsBadInput)
{
    uint8_t buf[CUSTOM_LINK_FRAME_MAX];
    uint8_t payload[4] = { 1, 2, 3, 4 };

    EXPECT_EQ(clEncodeFrame(0x10, 0, payload, 0, buf, sizeof(buf)), 0);                 // zero length
    EXPECT_EQ(clEncodeFrame(0x10, 0, NULL, 4, buf, sizeof(buf)), 0);                    // null payload
    EXPECT_EQ(clEncodeFrame(0x10, 0, payload, 200, buf, sizeof(buf)), 0);               // too large
    EXPECT_EQ(clEncodeFrame(0x10, 0, payload, 4, buf, 8), 0);                           // output too small
}

TEST(CustomLinkRoundtripTest, TestAllMessageIds)
{
    resetParser(10000);

    clPayloadFast_t fast = {};
    fast.ts_us = 123456;
    fast.gyro[FD_ROLL] = -1234;
    fast.acc[FD_PITCH] = 999;
    fast.attitude[FD_YAW] = 18000;
    uint8_t buf[CUSTOM_LINK_FRAME_MAX];
    uint8_t len = clEncodeFrame(CUSTOM_LINK_MSG_FC_FAST, 7, &fast, sizeof(fast), buf, sizeof(buf));
    ASSERT_GT(len, 0);
    feedBytes(buf, len, 1000, 10);
    EXPECT_EQ(testRx.frameCount, 1u);
    EXPECT_EQ(testRx.lastFrame.msgId, CUSTOM_LINK_MSG_FC_FAST);
    EXPECT_EQ(testRx.lastFrame.seq, 7);
    EXPECT_EQ(testRx.lastFrame.len, (uint8_t)sizeof(fast));
    clPayloadFast_t fastOut;
    memcpy(&fastOut, testRx.lastFrame.payload, sizeof(fastOut));
    EXPECT_EQ(fastOut.ts_us, fast.ts_us);
    EXPECT_EQ(fastOut.gyro[FD_ROLL], fast.gyro[FD_ROLL]);
    EXPECT_EQ(fastOut.acc[FD_PITCH], fast.acc[FD_PITCH]);
    EXPECT_EQ(fastOut.attitude[FD_YAW], fast.attitude[FD_YAW]);

    clPayloadControl_t ctrl = {};
    ctrl.host_ts_us = 0x1122334455667788ULL;
    ctrl.cmd_seq = 5170;
    ctrl.arm = 1;
    ctrl.mode_req = 2;
    ctrl.throttle = 1500;
    ctrl.rate_x10[0] = -100;
    ctrl.rate_x10[1] = 200;
    ctrl.rate_x10[2] = -300;
    len = clEncodeFrame(CUSTOM_LINK_MSG_HOST_CONTROL, 8, &ctrl, sizeof(ctrl), buf, sizeof(buf));
    ASSERT_GT(len, 0);
    feedBytes(buf, len, 5000, 10);
    EXPECT_EQ(testRx.frameCount, 2u);
    EXPECT_EQ(testRx.lastFrame.msgId, CUSTOM_LINK_MSG_HOST_CONTROL);
    clPayloadControl_t ctrlOut;
    memcpy(&ctrlOut, testRx.lastFrame.payload, sizeof(ctrlOut));
    EXPECT_EQ(ctrlOut.host_ts_us, ctrl.host_ts_us);
    EXPECT_EQ(ctrlOut.cmd_seq, ctrl.cmd_seq);
    EXPECT_EQ(ctrlOut.arm, ctrl.arm);
    EXPECT_EQ(ctrlOut.throttle, ctrl.throttle);
    EXPECT_EQ(ctrlOut.rate_x10[2], ctrl.rate_x10[2]);

    clPayloadTimesyncResp_t resp = {};
    resp.t1 = 1;
    resp.t2_isr_us = 2;
    resp.t3_tx_us = 3;
    len = clEncodeFrame(CUSTOM_LINK_MSG_FC_TIMESYNC, 9, &resp, sizeof(resp), buf, sizeof(buf));
    ASSERT_GT(len, 0);
    feedBytes(buf, len, 9000, 10);
    EXPECT_EQ(testRx.frameCount, 3u);
    clPayloadTimesyncResp_t respOut;
    memcpy(&respOut, testRx.lastFrame.payload, sizeof(respOut));
    EXPECT_EQ(respOut.t3_tx_us, 3u);
}

TEST(CustomLinkParserTest, TestFrameCompleteTimestamp)
{
    resetParser(10000);

    clPayloadTimesyncReq_t req = { .t1 = 42 };
    uint8_t buf[CUSTOM_LINK_FRAME_MAX];
    const uint8_t len = clEncodeFrame(CUSTOM_LINK_MSG_HOST_TIMESYNC, 1, &req, sizeof(req), buf, sizeof(buf));
    ASSERT_GT(len, 0);

    const timeUs_t lastByteUs = feedBytes(buf, len, 7000, 1);
    EXPECT_EQ(testRx.frameCount, 1u);
    EXPECT_EQ((uint32_t)testRx.lastFrameCompleteUs, (uint32_t)lastByteUs); // timestamp of the byte that completed the frame
}

TEST(CustomLinkParserTest, TestCrcFailureDropsFrameAndResyncs)
{
    resetParser(10000);

    clPayloadControl_t ctrl = {};
    ctrl.throttle = 1500;
    uint8_t buf[CUSTOM_LINK_FRAME_MAX];
    uint8_t len = clEncodeFrame(CUSTOM_LINK_MSG_HOST_CONTROL, 1, &ctrl, sizeof(ctrl), buf, sizeof(buf));
    ASSERT_GT(len, 0);

    // corrupt one payload bit
    buf[5] ^= 0x01;

    timeUs_t t = 1000;
    t = feedBytes(buf, len, t, 10);
    EXPECT_EQ(testRx.frameCount, 0u);

    // a subsequent intact frame must parse
    len = clEncodeFrame(CUSTOM_LINK_MSG_HOST_CONTROL, 2, &ctrl, sizeof(ctrl), buf, sizeof(buf));
    t = feedBytes(buf, len, t + 100, 10);
    EXPECT_EQ(testRx.frameCount, 1u);
    EXPECT_EQ(testRx.lastFrame.seq, 2);
}

TEST(CustomLinkParserTest, TestResyncOnGarbage)
{
    resetParser(10000);

    // pseudo-random garbage, deterministically seeded, containing no EB 90 pair
    uint32_t lcg = 0x12345678;
    for (int i = 0; i < 200; i++) {
        lcg = lcg * 1664525u + 1013904223u;
        uint8_t c = (uint8_t)(lcg >> 16);
        if (c == 0xEB) {
            c = 0xEC;
        }
        clParserProcessByte(&parser, c, 1000 + i, testCaptureFrame, NULL);
    }
    EXPECT_EQ(testRx.frameCount, 0u);

    clPayloadSlow_t slow = {};
    slow.vbat_mv = 16400;
    uint8_t buf[CUSTOM_LINK_FRAME_MAX];
    const uint8_t len = clEncodeFrame(CUSTOM_LINK_MSG_FC_SLOW, 3, &slow, sizeof(slow), buf, sizeof(buf));
    ASSERT_GT(len, 0);
    feedBytes(buf, len, 100000, 10);
    EXPECT_EQ(testRx.frameCount, 1u);
    EXPECT_EQ(testRx.lastFrame.msgId, CUSTOM_LINK_MSG_FC_SLOW);
}

TEST(CustomLinkParserTest, TestGarbageStartingWithSyncByte)
{
    resetParser(10000);

    const uint8_t garbage[] = { 0xEB, 0xEB, 0x11, 0x90 };
    feedBytes(garbage, sizeof(garbage), 1000, 10);
    EXPECT_EQ(testRx.frameCount, 0u);

    clPayloadFast_t fast = {};
    uint8_t buf[CUSTOM_LINK_FRAME_MAX];
    const uint8_t len = clEncodeFrame(CUSTOM_LINK_MSG_FC_FAST, 4, &fast, sizeof(fast), buf, sizeof(buf));
    ASSERT_GT(len, 0);
    feedBytes(buf, len, 10000, 10);
    EXPECT_EQ(testRx.frameCount, 1u);
}

TEST(CustomLinkParserTest, TestInvalidLengthRejected)
{
    resetParser(10000);

    // hand-built frame headers with impossible lengths
    const uint8_t zeroLen[] = { 0xEB, 0x90, 0x20, 0x00, 0x01, 0xAA, 0xBB };
    feedBytes(zeroLen, sizeof(zeroLen), 1000, 10);
    EXPECT_EQ(testRx.frameCount, 0u);

    const uint8_t bigLen[] = { 0xEB, 0x90, 0x20, 0xC8, 0x01, 0xAA, 0xBB };
    feedBytes(bigLen, sizeof(bigLen), 5000, 10);
    EXPECT_EQ(testRx.frameCount, 0u);

    // parser must still be aligned for a real frame afterwards
    clPayloadControl_t ctrl = {};
    uint8_t buf[CUSTOM_LINK_FRAME_MAX];
    const uint8_t len = clEncodeFrame(CUSTOM_LINK_MSG_HOST_CONTROL, 5, &ctrl, sizeof(ctrl), buf, sizeof(buf));
    ASSERT_GT(len, 0);
    feedBytes(buf, len, 10000, 10);
    EXPECT_EQ(testRx.frameCount, 1u);
}

TEST(CustomLinkParserTest, TestInterByteTimeout)
{
    resetParser(1000);

    clPayloadControl_t ctrl = {};
    uint8_t buf[CUSTOM_LINK_FRAME_MAX];
    const uint8_t len = clEncodeFrame(CUSTOM_LINK_MSG_HOST_CONTROL, 1, &ctrl, sizeof(ctrl), buf, sizeof(buf));
    ASSERT_GT(len, 0);

    // stall mid-frame for longer than the timeout -> the rest is dropped
    timeUs_t t = 1000;
    t = feedBytes(buf, 6, t, 10);          // into the payload
    feedBytes(&buf[6], len - 6, t + 2000, 10);  // resumed too late
    EXPECT_EQ(testRx.frameCount, 0u);

    // boundary: a gap of exactly the timeout still holds the frame together
    resetParser(1000);
    t = feedBytes(buf, 6, 5000, 10);
    feedBytes(&buf[6], len - 6, t + 1000, 10);
    EXPECT_EQ(testRx.frameCount, 1u);
}

TEST(CustomLinkParserTest, TestSeqWrap)
{
    resetParser(10000);

    clPayloadFast_t fast = {};
    uint8_t buf[CUSTOM_LINK_FRAME_MAX];
    uint8_t seq = 254;
    for (int i = 0; i < 4; i++) {
        const uint8_t len = clEncodeFrame(CUSTOM_LINK_MSG_FC_FAST, seq, &fast, sizeof(fast), buf, sizeof(buf));
        ASSERT_GT(len, 0);
        feedBytes(buf, len, 1000 + i * 1000, 10);
        seq++;
    }
    EXPECT_EQ(testRx.frameCount, 4u);
    EXPECT_EQ((unsigned)testRx.lastFrame.seq, 1u);   // 254, 255, 0, 1
}

TEST(CustomLinkControlTest, TestArmEdgeDetection)
{
    clControlState_t state;
    clControlReset(&state);
    EXPECT_FALSE(clControlIsFresh(&state, 1000, 50));   // never had a frame

    clFrame_t frame = {};
    frame.msgId = CUSTOM_LINK_MSG_HOST_CONTROL;
    frame.len = sizeof(clPayloadControl_t);
    clPayloadControl_t *cmd = (clPayloadControl_t *)frame.payload;

    cmd->arm = 1;
    cmd->throttle = 1500;
    EXPECT_EQ(clControlApplyFrame(&state, &frame, 1000), CL_EVENT_ARM_REQUEST);
    EXPECT_TRUE(clControlIsFresh(&state, 1000, 50));
    EXPECT_TRUE(clControlIsFresh(&state, 1050, 50));     // exactly at the watchdog boundary: still fresh
    EXPECT_FALSE(clControlIsFresh(&state, 1051, 50));

    cmd->arm = 1;                                        // repeated arm: no new event
    EXPECT_EQ(clControlApplyFrame(&state, &frame, 1100), CL_EVENT_NONE);

    cmd->arm = 0;
    EXPECT_EQ(clControlApplyFrame(&state, &frame, 1200), CL_EVENT_DISARM_REQUEST);

    clControlReset(&state);
    cmd->arm = 1;                                        // after a reset the edge fires again
    EXPECT_EQ(clControlApplyFrame(&state, &frame, 1300), CL_EVENT_ARM_REQUEST);
    EXPECT_EQ(state.arm, 1u);
}

TEST(CustomLinkControlTest, TestThrottleClampAndRates)
{
    clControlState_t state;
    clControlReset(&state);

    clFrame_t frame = {};
    frame.msgId = CUSTOM_LINK_MSG_HOST_CONTROL;
    frame.len = sizeof(clPayloadControl_t);
    clPayloadControl_t *cmd = (clPayloadControl_t *)frame.payload;

    cmd->throttle = 900;
    cmd->rate_x10[FD_ROLL] = -1500;
    cmd->rate_x10[FD_PITCH] = 0;
    cmd->rate_x10[FD_YAW] = 32767;
    cmd->cmd_seq = 0xBEEF;
    cmd->host_ts_us = 0x0102030405060708ULL;
    clControlApplyFrame(&state, &frame, 500);
    EXPECT_EQ(state.throttle, 1000u);                    // clamped low

    cmd->throttle = 2100;
    clControlApplyFrame(&state, &frame, 600);
    EXPECT_EQ(state.throttle, 2000u);                    // clamped high

    cmd->throttle = 1750;
    clControlApplyFrame(&state, &frame, 700);
    EXPECT_EQ(state.throttle, 1750u);

    EXPECT_EQ(state.rateX10[FD_ROLL], -1500);
    EXPECT_EQ(state.rateX10[FD_PITCH], 0);
    EXPECT_EQ(state.rateX10[FD_YAW], 32767);
    EXPECT_EQ(state.lastCmdSeq, 0xBEEFu);
    EXPECT_EQ(state.lastHostTsUs, 0x0102030405060708ULL);
    EXPECT_FLOAT_EQ(state.rateX10[FD_ROLL] * 0.1f, -150.0f);  // consumer-side scaling
}

TEST(CustomLinkControlTest, TestNonControlFramesIgnored)
{
    clControlState_t state;
    clControlReset(&state);

    clFrame_t frame = {};
    frame.msgId = CUSTOM_LINK_MSG_HOST_TIMESYNC;          // wrong msgId
    frame.len = sizeof(clPayloadControl_t);
    clPayloadControl_t *cmd = (clPayloadControl_t *)frame.payload;
    cmd->arm = 1;
    EXPECT_EQ(clControlApplyFrame(&state, &frame, 1000), CL_EVENT_NONE);
    EXPECT_FALSE(state.haveFrame);

    frame.msgId = CUSTOM_LINK_MSG_HOST_CONTROL;
    frame.len = 4;                                        // wrong length
    EXPECT_EQ(clControlApplyFrame(&state, &frame, 1000), CL_EVENT_NONE);
    EXPECT_FALSE(state.haveFrame);
}

TEST(CustomLinkStructTest, TestPackedLayout)
{
    EXPECT_EQ(sizeof(clPayloadFast_t), 22u);
    EXPECT_EQ(sizeof(clPayloadMedium_t), 46u);
    EXPECT_EQ(sizeof(clPayloadSlow_t), 36u);
    EXPECT_EQ(sizeof(clPayloadControl_t), 20u);
    EXPECT_EQ(sizeof(clPayloadTimesyncReq_t), 8u);
    EXPECT_EQ(sizeof(clPayloadTimesyncResp_t), 16u);

    EXPECT_EQ(offsetof(clPayloadControl_t, host_ts_us), 0u);
    EXPECT_EQ(offsetof(clPayloadControl_t, cmd_seq), 8u);
    EXPECT_EQ(offsetof(clPayloadControl_t, arm), 10u);
    EXPECT_EQ(offsetof(clPayloadControl_t, mode_req), 11u);
    EXPECT_EQ(offsetof(clPayloadControl_t, throttle), 12u);
    EXPECT_EQ(offsetof(clPayloadControl_t, rate_x10), 14u);

    EXPECT_EQ(offsetof(clPayloadTimesyncResp_t, t2_isr_us), 8u);
    EXPECT_EQ(offsetof(clPayloadTimesyncResp_t, t3_tx_us), 12u);
}
