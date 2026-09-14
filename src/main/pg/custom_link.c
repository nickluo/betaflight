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

#include "platform.h"

#include "pg/custom_link.h"
#include "pg/pg_ids.h"

#ifndef CUSTOM_LINK_WATCHDOG_MS
#define CUSTOM_LINK_WATCHDOG_MS  50   // 200 Hz stream: 10 consecutive lost frames
#endif

#ifndef CUSTOM_LINK_RATE_LIMIT_DPS
#define CUSTOM_LINK_RATE_LIMIT_DPS  0  // 0 = unlimited (host is fully trusted)
#endif

#ifndef CUSTOM_LINK_ANGLE_LIMIT_DEG
#define CUSTOM_LINK_ANGLE_LIMIT_DEG  0  // 0 = unlimited (host is fully trusted)
#endif

PG_REGISTER_WITH_RESET_TEMPLATE(customLinkConfig_t, customLinkConfig, PG_CUSTOM_LINK_CONFIG, 1);

PG_RESET_TEMPLATE(customLinkConfig_t, customLinkConfig,
    .watchdog_ms = CUSTOM_LINK_WATCHDOG_MS,
    .rate_limit_dps = CUSTOM_LINK_RATE_LIMIT_DPS,
    .angle_limit_deg = CUSTOM_LINK_ANGLE_LIMIT_DEG,
);
