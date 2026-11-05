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

#include "pg/pg.h"

typedef struct customLinkConfig_s {
    uint16_t watchdog_ms;       // host control stream staleness limit before offboard reverts to RC
    uint16_t rate_limit_dps;    // host rate setpoint clamp per axis, deg/s; 0 = unlimited (rate mode / yaw)
    uint16_t angle_limit_deg;   // rate-mode roll/pitch attitude envelope, deg; 0 = unlimited.
                                // mode_req=3 angle targets are NOT gated by this - they are
                                // bounded by the pid profile's angle_limit in pidLevel.
    uint8_t motors_stream;      // stream 0x13 mixer motor outputs to the host; 0 = off, 1 = on
} customLinkConfig_t;

PG_DECLARE(customLinkConfig_t, customLinkConfig);
