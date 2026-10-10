/*
 * This file is part of Betaflight.
 *
 * Betaflight is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Betaflight is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Betaflight. If not, see <http://www.gnu.org/licenses/>.
 */

#include "platform.h"

#ifndef USE_WING

#include "math.h"

#ifdef USE_ALTITUDE_HOLD

#include "build/debug.h"
#include "common/maths.h"
#include "config/config.h"

#include "fc/rc.h"
#include "fc/runtime_config.h"

#include "flight/autopilot.h"
#include "flight/failsafe.h"
#include "flight/flight_plan_nav.h"
#include "flight/position.h"
#include "flight/position_nav.h"

#include "rx/rx.h"
#include "pg/autopilot.h"

#include "alt_hold.h"

static const float taskIntervalSeconds = HZ_TO_INTERVAL(ALTHOLD_TASK_RATE_HZ); // i.e. 0.01 s

typedef struct {
    bool isActive;
    float targetAltitudeCm;
    float maxClimbRate;
    float targetVelocity;
    float deadband;
    bool allowStickAdjustment;
    bool navVelocityOnly;
} altHoldState_t;

altHoldState_t altHold;

// A rescue in progress substitutes gpsRescueConfig()'s ascend/descend rate for
// the alt-hold climbRate cap; failsafe auto-landing and pilot alt-hold keep it.
static float altHoldMaxClimbRate(void)
{
    const float rescueRate = flightPlanNavGetRescueVerticalRateCmS();
    return (rescueRate > 0.0f) ? rescueRate : altHold.maxClimbRate;
}

static void altHoldReset(void)
{
    resetAltitudeControl();
    altHold.targetAltitudeCm = getAltitudeCmControl();
    altHold.targetVelocity = 0.0f;
    altHold.navVelocityOnly = false;
}

void altHoldInit(void)
{
    altHold.isActive = false;
    altHold.deadband = altHoldConfig()->deadband / 100.0f;
    altHold.allowStickAdjustment = altHoldConfig()->deadband;
    altHold.maxClimbRate = altHoldConfig()->climbRate * 10.0f; // 50 in CLI means 500cm/s
    altHoldReset();
}

static void altHoldProcessTransitions(void) {

    if (FLIGHT_MODE(ALT_HOLD_MODE)) {
        if (!altHold.isActive) {
            altHoldReset();
            autopilotCaptureHoverThrottleForAltHold();
            altHold.isActive = true;
        }
    } else {
        if (altHold.isActive) {
            resetAltitudeControl();  // Reset throttle output when exiting altitude hold
            autopilotClearAltHoldHoverThrottle();
        }
        altHold.isActive = false;
    }
}

static void altHoldUpdateTargetAltitude(void)
{
    // Outside the deadband, including full-low throttle, the pilot requests
    // vertical speed. Arming's low-throttle check does not apply to this loop.

    float stickFactor = 0.0f;

    if (altHold.allowStickAdjustment) {
        const float rcThrottle = rcCommand[THROTTLE];
        const float hoverPwm = (float)autopilotGetEffectiveHoverThrottlePwm();
        const float lowThreshold = hoverPwm - altHold.deadband * (hoverPwm - PWM_RANGE_MIN);
        const float highThreshold = hoverPwm + altHold.deadband * (PWM_RANGE_MAX - hoverPwm);

        if (rcThrottle < lowThreshold) {
            stickFactor = scaleRangef(rcThrottle, PWM_RANGE_MIN, lowThreshold, -1.0f, 0.0f);
        } else if (rcThrottle > highThreshold) {
            stickFactor = scaleRangef(rcThrottle, highThreshold, PWM_RANGE_MAX, 0.0f, 1.0f);
        }
    }
    // StickFactor is a multiplier for maxClimbRate, based on throttle position, that sets the ascend or descend velocity
    // is zero within the deadband, meaning no requested change in altitude
    // below the lower deadband limit, it is negative, reaching -1 (full descent) at zero throttle
    // conversely, as throttle moves above the upper deadband limit, StickFactor increased from 0 to 1 at full throttle
            
    // if failsafe is active, and we get here, we are in failsafe landing mode, it controls throttle.
    // a switch-invoked rescue that cannot stage or has aborted drives the same descent.
    if (failsafeIsActive() || flightPlanNavIsRescueDescentActive()) {
        // descend at up to 10 times faster when high
        // default landing timeout is now 60s; must to get the quad down within this limit
        // need a rapid descent when initiated high, and must slow down closer to ground
        // this code doubles descent rate at 20m, to max 10x (10m/s on defaults) at 200m
        // the deceleration may be a bit rocky if it starts very high up
        // constant (set) deceleration target in the last 2m
        stickFactor = -(0.9f + constrainf(getAltitudeCmControl() / 2000.0f, 0.1f, 9.0f));
    }

    const float maxClimbRate = altHoldMaxClimbRate();
    altHold.targetVelocity = stickFactor * maxClimbRate;
    // prevent pilot altitude adjustments from moving the target altitude so far away from current altitude
    // that it might be difficult to get back to a similar target altitude in a reasonable time.
    // the altitude target cannot be moved to a location that cannot be reached in 1s at maxClimbRate
    // this constrains the P and I response to user target changes, but not D of F responses
    if (fabsf(getAltitudeCmControl() - altHold.targetAltitudeCm) < maxClimbRate * 1.0f /* s */) {
        altHold.targetAltitudeCm += altHold.targetVelocity * taskIntervalSeconds;
    }
}

static void altHoldUpdate(void)
{
    const positionNavCommand_t *navCmd = positionNavHasActiveTarget() ? positionNavGetActiveCommand() : NULL;
    const bool navAltitude = navCmd && navCmd->includeAltitude;
    if (!navAltitude && altHoldConfig()->climbRate) {
        altHoldUpdateTargetAltitude();
    }

    float targetAltitudeCm = altHold.targetAltitudeCm;
    float targetAltitudeVelocity = altHold.targetVelocity;

    if (navAltitude) {
        targetAltitudeVelocity = positionNavTargetReached() ? 0.0f : positionNavGetTargetVelocityCmS().z;
        if (navCmd->altitudeVelocityOnly) {
            const float altitude = getAltitudeCmControl();
            if (!altHold.navVelocityOnly) {
                altHold.targetAltitudeCm = altitude;
            }
            targetAltitudeVelocity = constrainf(targetAltitudeVelocity, -altHoldMaxClimbRate(), altHoldMaxClimbRate());
            // The far-below landing target prevents navigation arrival; do not
            // feed it to altitude P. Track descent with at most one second of lag.
            const float errorLimitCm = fmaxf(fabsf(targetAltitudeVelocity), 1.0f);
            altHold.targetAltitudeCm = constrainf(
                altHold.targetAltitudeCm + targetAltitudeVelocity * taskIntervalSeconds,
                altitude - errorLimitCm, altitude + errorLimitCm);
            targetAltitudeCm = altHold.targetAltitudeCm;
        } else {
            targetAltitudeCm = navCmd->targetPosEfM.z * 100.0f;
            // A completion callback can clear the target before the altitude
            // task sees "reached"; retain every live target for the final hold.
            altHold.targetAltitudeCm = targetAltitudeCm;
        }
    }
    altHold.navVelocityOnly = navAltitude && navCmd->altitudeVelocityOnly;

    altitudeControl(targetAltitudeCm, taskIntervalSeconds, targetAltitudeVelocity, altHoldMaxClimbRate());
}

void updateAltHold(timeUs_t currentTimeUs) {
    UNUSED(currentTimeUs);

    // check for enabling Alt Hold, otherwise do as little as possible while inactive
    altHoldProcessTransitions();

    if (altHold.isActive) {
        altHoldUpdate();
    }
}

bool isAltHoldActive(void) {
    return altHold.isActive;
}

#endif // USE_ALTITUDE_HOLD
#endif // !USE_WING
