#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <math.h>

extern "C" {
#include "platform.h"
#include "build/debug.h"
#include "pg/pg_ids.h"

#include "common/maths.h"
#include "common/vector.h"

#include "drivers/time.h"

#include "fc/runtime_config.h"

#include "flight/imu.h"
#include "flight/position.h"
#include "flight/position_estimator.h"

// STATIC_UNIT_TESTED in position_estimator.c — gravity-removed earth-frame
// linear acceleration in ENU (cm/s^2).
void getLinearAccelENU(float *accelEast, float *accelNorth, float *accelUp);

#include "io/gps.h"
#include "io/gps_virtual.h"

#include "sensors/acceleration.h"
#include "sensors/barometer.h"
#include "sensors/rangefinder.h"
#include "sensors/sensors.h"

PG_REGISTER(positionConfig_t, positionConfig, PG_POSITION, 0);
}

#include "gtest/gtest.h"

extern "C" {
uint8_t armingFlags = 0;
uint8_t stateFlags = 0;
uint16_t flightModeFlags = 0;
uint8_t debugMode = 0;
int16_t debug[DEBUG16_VALUE_COUNT];

acc_t acc;
matrix33_t rMat;
attitudeEulerAngles_t attitude;
gpsSolutionData_t gpsSol;

static uint32_t enabledSensors = 0;
static bool rfHealthy = false;
static float rfAltCm = 0.0f;
static float baroAltCm = 0.0f;
static timeUs_t fakeMicros = 0;
static bool gps3DFix = true;

bool sensors(uint32_t mask) { return (enabledSensors & mask) != 0; }
bool rangefinderIsHealthy(void) { return rfHealthy; }
int32_t rangefinderGetLatestAltitude(void) { return lrintf(rfAltCm); }
float getBaroAltitude(void) { return baroAltCm; }

timeUs_t micros(void) { return fakeMicros; }
bool gpsHas3DFix(void) { return STATE(GPS_FIX) && gps3DFix; }

bool gpsHasNewData(uint16_t *gpsStamp)
{
    (*gpsStamp)++;
    return true;
}

void GPS_distance2d(const gpsLocation_t *from, const gpsLocation_t *to, vector2_t *dest)
{
    // Simplified (no Earth-scale projection): return integer-unit deltas so sign tests work.
    dest->x = (float)(to->lon - from->lon);
    dest->y = (float)(to->lat - from->lat);
}
}

static void stepEstimator(unsigned count = 1)
{
    for (unsigned i = 0; i < count; i++) {
        fakeMicros += 10000; // 100 Hz
        positionEstimatorUpdate();
    }
}

class PositionEstimatorTest : public ::testing::Test {
protected:
    void SetUp() override
    {
        memset(&acc, 0, sizeof(acc));
        memset(&rMat, 0, sizeof(rMat));
        memset(&attitude, 0, sizeof(attitude));
        memset(&gpsSol, 0, sizeof(gpsSol));
        memset(debug, 0, sizeof(debug));

        // Identity rotation and 1G reciprocal scale so zero accel stays zero.
        rMat.m[NWU_N][X] = 1.0f;
        rMat.m[NWU_W][Y] = 1.0f;
        rMat.m[NWU_U][Z] = 1.0f;
        acc.dev.acc_1G_rec = 1.0f;
        acc.accADC.z = 1.0f;

        enabledSensors = SENSOR_GPS | SENSOR_BARO | SENSOR_RANGEFINDER;
        rfHealthy = true;
        rfAltCm = 100.0f;
        baroAltCm = 100.0f;
        gpsSol.llh.altCm = 100.0f;
        gpsSol.dop.pdop = 100; // pDOP 1.0

        stateFlags = GPS_FIX;
        armingFlags = ARMED;
        fakeMicros = 0;
        gps3DFix = true;

        positionConfigMutable()->altitude_source = ALTITUDE_SOURCE_RANGEFINDER_PREFER;
        positionConfigMutable()->altitude_prefer_baro = 100;
        positionConfigMutable()->rangefinder_max_range_cm = 400;

        positionEstimatorInit();
    }
};

TEST_F(PositionEstimatorTest, ArmOdometryRequiresThreeDFixAtArm)
{
    positionEstimate3d_t odom;
    uint16_t originId;
    stepEstimator(2);
    EXPECT_FALSE(positionEstimatorGetArmOdometry(&odom, &originId));
    gps3DFix = false;
    positionEstimatorCaptureArmOrigin();
    stepEstimator(2);
    EXPECT_FALSE(positionEstimatorGetArmOdometry(&odom, &originId));
    gps3DFix = true;
    stepEstimator(2);
    EXPECT_FALSE(positionEstimatorGetArmOdometry(&odom, &originId));
    positionEstimatorCaptureArmOrigin();
    stepEstimator(2);
    ASSERT_TRUE(positionEstimatorGetArmOdometry(&odom, &originId));
    EXPECT_EQ(originId, 2);
    EXPECT_NEAR(odom.position.x, 0, 0.01);
    EXPECT_NEAR(odom.position.y, 0, 0.01);
    EXPECT_NEAR(odom.position.z, 0, 0.01);
}

TEST_F(PositionEstimatorTest, ArmHeadingIsCapturedOnceAndOnlyChangesOnRearm)
{
    attitude.values.yaw = 1350;
    positionEstimatorCaptureArmOrigin();
    stepEstimator(2);
    EXPECT_EQ(positionEstimatorGetArmYawCdeg(), -13500);
    attitude.values.yaw = 2700;
    stepEstimator(2);
    EXPECT_EQ(positionEstimatorGetArmYawCdeg(), -13500);
    DISABLE_STATE(GPS_FIX);
    stepEstimator(2);
    ENABLE_STATE(GPS_FIX);
    stepEstimator(2);
    EXPECT_EQ(positionEstimatorGetArmYawCdeg(), -13500);
    armingFlags = 0;
    stepEstimator(2);
    armingFlags = ARMED;
    positionEstimatorCaptureArmOrigin();
    EXPECT_EQ(positionEstimatorGetArmYawCdeg(), 9000);
}

TEST_F(PositionEstimatorTest, VirtualGpsUsesValidDopUnitsAndVelocity)
{
    setVirtualGPS(47.0, 8.0, 123.0, 2.0, 2.5, 90.0, 1.0, 2.0, -0.5);
    gpsSolutionData_t solution;
    getVirtualGPS(&solution);
    EXPECT_EQ(solution.dop.pdop, 100);
    EXPECT_EQ(solution.dop.hdop, 100);
    EXPECT_EQ(solution.dop.vdop, 100);
    EXPECT_EQ(solution.velned.velN, 100);
    EXPECT_EQ(solution.velned.velE, 200);
    EXPECT_EQ(solution.velned.velD, -50);
    EXPECT_EQ(solution.llh.altCm, 12300);
}

TEST_F(PositionEstimatorTest, NoisyArmBarometerSampleDoesNotBecomePermanentOdometryOffset)
{
    enabledSensors = SENSOR_GPS | SENSOR_BARO;
    positionConfigMutable()->altitude_source = ALTITUDE_SOURCE_BARO_ONLY;
    armingFlags = 0;
    baroAltCm = 10000.0f;
    stepEstimator(300);
    baroAltCm = 10100.0f; // One noisy sample immediately before ARM.
    stepEstimator();
    armingFlags = ARMED;
    positionEstimatorCaptureArmOrigin();
    positionEstimatorResetZ();
    baroAltCm = 10000.0f;
    stepEstimator(100);
    positionEstimate3d_t odom;
    uint16_t originId;
    ASSERT_TRUE(positionEstimatorGetArmOdometry(&odom, &originId));
    EXPECT_NEAR(odom.position.v[ENU_U], 0.0f, 3.0f);
}

TEST_F(PositionEstimatorTest, ArmOdometryPreservesOriginThroughGpsRecoveryAndRearms)
{
    gpsSol.llh.lon = 1000;
    gpsSol.llh.lat = 2000;
    gpsSol.llh.altCm = 300;
    fakeMicros = 1;
    positionEstimatorCaptureArmOrigin();
    positionEstimatorResetZ();
    stepEstimator(20);
    gpsSol.llh.lon += 500;
    gpsSol.llh.lat -= 300;
    gpsSol.llh.altCm += 200;
    baroAltCm += 200;
    rfAltCm += 200;
    stepEstimator(1000);
    positionEstimate3d_t before, after;
    uint16_t firstId, id;
    ASSERT_TRUE(positionEstimatorGetArmOdometry(&before, &firstId));
    EXPECT_NEAR(before.position.x, 500, 5);
    EXPECT_NEAR(before.position.y, -300, 5);
    EXPECT_NEAR(before.position.z, 200, 20);
    EXPECT_GT(before.positionVariance.x, 0);
    EXPECT_GT(before.velocityVariance.y, 0);
    DISABLE_STATE(GPS_FIX);
    stepEstimator(5);
    EXPECT_FALSE(positionEstimatorGetArmOdometry(&after, &id));
    ENABLE_STATE(GPS_FIX);
    stepEstimator(20);
    ASSERT_TRUE(positionEstimatorGetArmOdometry(&after, &id));
    EXPECT_EQ(id, firstId);
    EXPECT_NEAR(after.position.x, before.position.x, 5);
    EXPECT_NEAR(after.position.y, before.position.y, 5);
    fakeMicros += 60000;
    EXPECT_FALSE(positionEstimatorGetArmOdometry(&after, &id));
    stepEstimator();
    ASSERT_TRUE(positionEstimatorGetArmOdometry(&after, &id));
    armingFlags = 0;
    positionEstimatorResetZ(); // calculateEstimatedAltitude resets Z on the disarm edge.
    stepEstimator(5);
    EXPECT_FALSE(positionEstimatorGetArmOdometry(&after, &id));
    armingFlags = ARMED;
    positionEstimatorCaptureArmOrigin();
    positionEstimatorResetZ();
    stepEstimator(20);
    ASSERT_TRUE(positionEstimatorGetArmOdometry(&after, &id));
    EXPECT_EQ(id, firstId + 1);
    EXPECT_NEAR(after.position.x, 0, 0.01);
    EXPECT_NEAR(after.position.y, 0, 0.01);
    EXPECT_NEAR(after.position.z, 0, 0.01);
}

TEST_F(PositionEstimatorTest, RangefinderPreferFallsBackAndRecovers)
{
    // Establish offsets/baseline near zero.
    stepEstimator(10);
    const float baseline = positionEstimatorGetAltitudeCm();

    // Rangefinder unavailable: fallback should use baro/GPS updates.
    rfHealthy = false;
    baroAltCm = 150.0f;
    gpsSol.llh.altCm = 150.0f;
    stepEstimator(40);
    const float fallbackAltitude = positionEstimatorGetAltitudeCm();
    EXPECT_GT(fallbackAltitude, baseline + 10.0f);

    // Rangefinder becomes available again at a lower relative altitude.
    rfHealthy = true;
    rfAltCm = 120.0f; // +20cm relative to RF offset baseline
    stepEstimator(40);
    const float recoveredAltitude = positionEstimatorGetAltitudeCm();

    // In RANGEFINDER_PREFER, recovered altitude should be pulled down toward RF value.
    EXPECT_LT(recoveredAltitude, fallbackAltitude - 5.0f);
    EXPECT_TRUE(positionEstimatorIsValidZ());
}

// Regression test: GPS_distance2d must be called as (arm, current) so that a craft
// East of the arm position produces a positive kfEast (East) position estimate.
// The bug had the arguments reversed — (current, arm) — giving a negative East estimate.
TEST_F(PositionEstimatorTest, GPSPositionEastOfArmIsPositive)
{
    // Run a couple of steps so the arm location is captured from the initial gpsSol (lon=0).
    stepEstimator(2);

    // Place the craft East of the arm (positive longitude delta).
    gpsSol.llh.lon = 100000;

    // Let the Kalman filter converge toward the GPS position measurement.
    stepEstimator(30);

    // position is ENU; index by ENU_E so "East" is explicit rather than ".x".
    EXPECT_GT(positionEstimatorGetEstimate()->position.v[ENU_E], 0.0f);
}

// Build rMat from roll/pitch/yaw using the exact quaternion + rotation-matrix
// formulas from imu.c (imuComputeQuaternionFromRPY + imuComputeRotationMatrix),
// so the matrix carries the firmware's body->earth NWU (North-West-Up) convention.
// Yaw follows the attitude/compass convention: 0 = North, +90 = East increases the
// reported heading, so a craft heading East is built with yawDeg = -90.
static void setRMatFromEuler(float rollDeg, float pitchDeg, float yawDeg)
{
    const float hr = DEGREES_TO_RADIANS(rollDeg) * 0.5f;
    const float hp = DEGREES_TO_RADIANS(pitchDeg) * 0.5f;
    const float hy = DEGREES_TO_RADIANS(yawDeg) * 0.5f;

    const float cr = cosf(hr), sr = sinf(hr);
    const float cp = cosf(hp), sp = sinf(hp);
    const float cy = cosf(hy), sy = sinf(hy);

    const float q0 = cr * cp * cy + sr * sp * sy;
    const float q1 = sr * cp * cy - cr * sp * sy;
    const float q2 = cr * sp * cy + sr * cp * sy;
    const float q3 = cr * cp * sy - sr * sp * cy;

    const float xx = q1 * q1, yy = q2 * q2, zz = q3 * q3;
    const float xy = q1 * q2, xz = q1 * q3, yz = q2 * q3;
    const float wx = q0 * q1, wy = q0 * q2, wz = q0 * q3;

    rMat.m[NWU_N][X] = 1.0f - 2.0f * yy - 2.0f * zz;
    rMat.m[NWU_N][Y] = 2.0f * (xy - wz);
    rMat.m[NWU_N][Z] = 2.0f * (xz + wy);
    rMat.m[NWU_W][X] = 2.0f * (xy + wz);
    rMat.m[NWU_W][Y] = 1.0f - 2.0f * xx - 2.0f * zz;
    rMat.m[NWU_W][Z] = 2.0f * (yz - wx);
    rMat.m[NWU_U][X] = 2.0f * (xz - wy);
    rMat.m[NWU_U][Y] = 2.0f * (yz + wx);
    rMat.m[NWU_U][Z] = 1.0f - 2.0f * xx - 2.0f * yy;
}

// Regression test for the East-West / magnitude bug discussed in #15321 and #15339.
// getLinearAccelENU must use matrixVectorMul (body->earth NWU) with East = -West;
// the previously-used matrixTrnVectorMul applied the inverse rotation, giving the
// wrong magnitude and a spurious North term.
//
// Scenario: heading East, 30 deg right roll, 0.5 g of forward (East) thrust.
// Gravity projects onto body Y and Z because of the roll, so accBF = (0.5, 0.5, 0.866).
// In the correct NWU formulation the gravity cross-terms cancel exactly, leaving
// the full 0.5 g of thrust on East and nothing on North or Up.
TEST_F(PositionEstimatorTest, EastThrustProducesEastAccelENU)
{
    constexpr float GRAVITY_CMSS = 980.665f;

    // rMat produced via imuComputeRotationMatrix's convention for heading East + 30 deg roll.
    setRMatFromEuler(30.0f, 0.0f, -90.0f);

    // Forward (East) thrust of 0.5 g; gravity projects onto body Y and Z due to the roll.
    acc.dev.acc_1G_rec = 1.0f;
    acc.accADC.x = 0.5f;
    acc.accADC.y = 0.5f;    // sin(30) * 1g
    acc.accADC.z = 0.866f;  // cos(30) * 1g

    float accelEast, accelNorth, accelUp;
    getLinearAccelENU(&accelEast, &accelNorth, &accelUp);

    EXPECT_NEAR(accelEast,  0.5f * GRAVITY_CMSS, 0.01f * GRAVITY_CMSS);
    EXPECT_NEAR(accelNorth, 0.0f,                0.01f * GRAVITY_CMSS);
    EXPECT_NEAR(accelUp,    0.0f,                0.01f * GRAVITY_CMSS);
}

// Integration counterpart: with the same attitude/thrust, the IMU prediction alone
// (GPS held at the origin) must drive a positive East velocity estimate.
TEST_F(PositionEstimatorTest, EastThrustProducesPositiveEastVelocity)
{
    // Run a couple of steps so XY fusion is established with the arm at origin.
    stepEstimator(2);

    setRMatFromEuler(30.0f, 0.0f, -90.0f);

    acc.accADC.x = 0.5f;
    acc.accADC.y = 0.5f;    // sin(30) * 1g
    acc.accADC.z = 0.866f;  // cos(30) * 1g

    // GPS position and velocity remain zero so any non-zero velocity estimate is
    // driven purely by the IMU prediction.
    stepEstimator(100);

    // velocity is ENU; index by ENU_E so "East" is explicit rather than ".x".
    EXPECT_GT(positionEstimatorGetEstimate()->velocity.v[ENU_E], 0.0f);
}
