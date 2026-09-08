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

#include "common/time.h"

// Custom high-frequency bidirectional link to a companion computer.
// Wire protocol and frame codec: telemetry/custom_link_protocol.h.
//
// Serial port is claimed exclusively through the FUNCTION_CUSTOM_LINK
// serial function (telemetry_baudrateIndex selects the baud, BAUD_AUTO
// means 921600). Downlink streams run as three scheduler tasks
// (200/100/10 Hz). Uplink control frames arm/disarm and, while the
// BOXOFFBOARD switch is active and the stream is fresh, drive the PID
// rate setpoints and throttle (OFFBOARD_MODE).

struct serialPort_s;

void customLinkInit(void);
bool customLinkIsEnabled(void);

// True while the most recent valid host control frame is younger than
// custom_link_watchdog_ms.
bool customLinkIsControlFresh(void);

// True while OFFBOARD_MODE is engaged and the host stream is fresh. This is
// the single condition the PID and mixer hot paths check.
bool customLinkHasControl(void);

// True while a fresh host stream holds arm=1. Treated like an active pilot
// ARM switch by the arming state machine (rc_controls.c).
bool customLinkHostArmActive(void);

// Host rate setpoint for an axis in deg/s (0.1 deg/s protocol units scaled).
float customLinkGetRateSetpoint(int axis);

// Host throttle in mixer units, 0.0 .. 1.0.
float customLinkGetThrottle(void);

// Scheduler task entry points (fc/tasks.c).
void customLinkTaskFast(timeUs_t currentTimeUs);    // 200 Hz: uplink, authority, 0x10
void customLinkTaskMed(timeUs_t currentTimeUs);     // 100 Hz: 0x11
void customLinkTaskSlow(timeUs_t currentTimeUs);    // 10 Hz: 0x12

#endif // USE_CUSTOM_LINK
