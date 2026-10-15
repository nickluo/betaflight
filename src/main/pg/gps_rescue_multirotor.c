/*
 * This file is part of Cleanflight and Betaflight.
 *
 * Cleanflight and Betaflight are free software. You can redistribute
 * this software and/or modify this software under the terms of the
 * GNU General Public License as published by the Free Software
 * Foundation, either version 3 of the License, or (at your option)
 * any later version.
 *
 * Cleanflight and Betaflight are distributed in the hope that they
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

#ifndef USE_WING
#ifdef USE_GPS_RESCUE

#include "flight/gps_rescue.h"

#include "pg/pg.h"
#include "pg/pg_ids.h"

#include "gps_rescue.h"

PG_REGISTER_WITH_RESET_TEMPLATE(gpsRescueConfig_t, gpsRescueConfig, PG_GPS_RESCUE, 9);

PG_RESET_TEMPLATE(gpsRescueConfig_t, gpsRescueConfig,

    .minStartDistM = 15,
    .altitudeMode = GPS_RESCUE_ALT_MODE_FIXED,
    .initialClimbM = 10,
    .ascendRate = 200,          // cm/s, 2 m/s climb to the return altitude

    .returnAltitudeM = 8,
    .groundSpeedCmS = 300,      // cm/s, 3 m/s return cruise speed

    .descentDistanceM = 20,
    .descendRate = 200,         // cm/s, 2 m/s descent while above the return altitude
    .landingRate = 50,          // cm/s, 0.5 m/s final descent once below the return altitude
    .sanityChecks = RESCUE_SANITY_FS_ONLY,
    .minSats = 8,
    .allowArmingWithoutFix = false,

    .yawP = 20,
);

#endif // USE_GPS_RESCUE
#endif // !USE_WING
