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
#include <string.h>

#include "platform.h"

#if defined(USE_CUSTOM_LINK)

#include "common/crc.h"

#include "telemetry/custom_link_protocol.h"

uint16_t clCrc(const clFrame_t *frame)
{
    // msgId, len and seq are contiguous at the head of clFrame_t
    uint16_t crc = crc16_ccitt_update(0, &frame->msgId, 3);
    return crc16_ccitt_update(crc, frame->payload, frame->len);
}

uint8_t clEncodeFrame(uint8_t msgId, uint8_t seq, const void *payload, uint8_t payloadLen, uint8_t *out, uint8_t outSize)
{
    if (payloadLen == 0 || payloadLen > CUSTOM_LINK_MAX_PAYLOAD || payload == NULL) {
        return 0;
    }
    const uint8_t frameSize = CUSTOM_LINK_FRAME_OVERHEAD + payloadLen;
    if (out == NULL || outSize < frameSize) {
        return 0;
    }

    clFrame_t frame = { 0 };
    frame.msgId = msgId;
    frame.len = payloadLen;
    frame.seq = seq;
    memcpy(frame.payload, payload, payloadLen);
    const uint16_t crc = clCrc(&frame);

    out[0] = CUSTOM_LINK_SYNC1;
    out[1] = CUSTOM_LINK_SYNC2;
    out[2] = msgId;
    out[3] = payloadLen;
    out[4] = seq;
    memcpy(&out[5], payload, payloadLen);
    out[frameSize - 2] = (uint8_t)(crc & 0xFF);
    out[frameSize - 1] = (uint8_t)(crc >> 8);

    return frameSize;
}

void clParserInit(clParser_t *parser, timeUs_t interByteTimeoutUs)
{
    memset(parser, 0, sizeof(*parser));
    parser->state = CL_PARSER_SYNC1;
    parser->interByteTimeoutUs = interByteTimeoutUs;
}

static void clParserReset(clParser_t *parser)
{
    parser->state = CL_PARSER_SYNC1;
    parser->hdrPos = 0;
    parser->payloadPos = 0;
}

void clParserProcessByte(clParser_t *parser, uint8_t c, timeUs_t nowUs, clFrameHandler_t handler, void *ctx)
{
    // A gap longer than the inter-byte timeout means this byte starts a new
    // frame; drop whatever partial frame was in progress.
    if (parser->state != CL_PARSER_SYNC1 && cmpTimeUs(nowUs, parser->lastByteUs) > (timeDelta_t)parser->interByteTimeoutUs) {
        clParserReset(parser);
    }
    parser->lastByteUs = nowUs;

    switch (parser->state) {
    case CL_PARSER_SYNC1:
        if (c == CUSTOM_LINK_SYNC1) {
            parser->state = CL_PARSER_SYNC2;
        }
        break;

    case CL_PARSER_SYNC2:
        if (c == CUSTOM_LINK_SYNC2) {
            parser->hdrPos = 0;
            parser->state = CL_PARSER_HDR;
        } else if (c != CUSTOM_LINK_SYNC1) {
            // still exactly one preamble byte seen; keep waiting for 0x90
            parser->state = CL_PARSER_SYNC1;
        }
        break;

    case CL_PARSER_HDR:
        (&parser->frame.msgId)[parser->hdrPos++] = c;
        if (parser->hdrPos >= 3) {
            if (parser->frame.len == 0 || parser->frame.len > CUSTOM_LINK_MAX_PAYLOAD) {
                clParserReset(parser);
            } else {
                parser->payloadPos = 0;
                parser->state = CL_PARSER_PAYLOAD;
            }
        }
        break;

    case CL_PARSER_PAYLOAD:
        parser->frame.payload[parser->payloadPos++] = c;
        if (parser->payloadPos >= parser->frame.len) {
            parser->crc = clCrc(&parser->frame);
            parser->state = CL_PARSER_CRC_LO;
        }
        break;

    case CL_PARSER_CRC_LO:
        if (c == (uint8_t)(parser->crc & 0xFF)) {
            parser->state = CL_PARSER_CRC_HI;
        } else {
            clParserReset(parser);
        }
        break;

    case CL_PARSER_CRC_HI:
        if (c == (uint8_t)(parser->crc >> 8)) {
            const clFrame_t completed = parser->frame;   // handler may reuse the parser
            clParserReset(parser);
            if (handler) {
                handler(&completed, nowUs, ctx);
            }
        } else {
            clParserReset(parser);
        }
        break;

    default:
        clParserReset(parser);
        break;
    }
}

void clControlReset(clControlState_t *state)
{
    memset(state, 0, sizeof(*state));
}

clEvent_e clControlApplyFrame(clControlState_t *state, const clFrame_t *frame, uint32_t nowMs)
{
    if (frame->msgId != CUSTOM_LINK_MSG_HOST_CONTROL || frame->len != sizeof(clPayloadControl_t)) {
        return CL_EVENT_NONE;
    }

    clPayloadControl_t cmd;
    memcpy(&cmd, frame->payload, sizeof(cmd));

    clEvent_e event = CL_EVENT_NONE;
    if (state->arm == 0 && cmd.arm != 0) {
        event = CL_EVENT_ARM_REQUEST;
    } else if (state->arm != 0 && cmd.arm == 0) {
        event = CL_EVENT_DISARM_REQUEST;
    }

    state->arm = cmd.arm ? 1 : 0;
    state->lastHostTsUs = cmd.host_ts_us;
    state->lastCmdSeq = cmd.cmd_seq;
    state->throttle = cmd.throttle < 1000 ? 1000 : (cmd.throttle > 2000 ? 2000 : cmd.throttle);
    for (unsigned i = 0; i < 3; i++) {
        state->rateX10[i] = cmd.rate_x10[i];
    }
    state->lastFrameMs = nowMs;
    state->haveFrame = true;

    return event;
}

bool clControlIsFresh(const clControlState_t *state, uint32_t nowMs, uint16_t watchdogMs)
{
    // unsigned subtraction tolerates millis() wraparound; fresh while the age
    // has not yet exceeded the watchdog window
    return state->haveFrame && (uint32_t)(nowMs - state->lastFrameMs) <= watchdogMs;
}

#endif // USE_CUSTOM_LINK
