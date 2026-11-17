/*
 * This file is part of Gripora.
 *
 * Gripora is free software. You can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Gripora is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this software. If not, see <https://www.gnu.org/licenses/>.
 */

#include <math.h>
#include <stdbool.h>
#include <stdint.h>

#include "gtest/gtest.h"

extern "C" {
#include "platform.h"
#include "common/maths.h"
#include "pg/pg.h"
#include "pg/drive.h"
#include "flight/drive.h"

uint8_t armingFlags;

// driveUpdate() is not used here: the tests drive driveUpdateFrom() directly
float mixerGetInput(uint8_t) { return 0; }
bool isMotorRpmSourceActive(uint8_t) { return false; }
float getMotor1Speedf(void) { return 0; }
float getMotor2Speedf(void) { return 0; }
float getMotorRPMf(uint8_t) { return 0; }
bool isMotorProtocolDshot(void) { return false; }
}

static const float DT = 0.001f;

// Wheel RPM for a speed with the default 100 mm tyre
static float rpmFor(float kmh)
{
    return kmh / (M_PIf * 100 * 1e-6f * 60.0f);
}

class DriveTest : public ::testing::Test {
protected:
    driveEscMode_e mode = DRIVE_ESC_FWD_BRK_REV;
    bool rpm = false;
    float kmh = 0;

    void SetUp() override
    {
        pgResetAll();
        driveReset();
        // A disarmed cycle, as before every arming
        driveUpdateFrom(false, 0, rpm, 0, mode, DT);
    }

    void run(float throttle, float seconds)
    {
        const int n = lrintf(seconds / DT);
        for (int i = 0; i < n; i++)
            driveUpdateFrom(true, throttle, rpm, rpmFor(kmh), mode, DT);
    }

    void withRpm(float speed)
    {
        rpm = true;
        kmh = speed;
        // Let the speed filter and the stop timer settle
        driveUpdateFrom(false, 0, rpm, rpmFor(kmh), mode, DT);
        for (int i = 0; i < 300; i++)
            driveUpdateFrom(false, 0, rpm, rpmFor(kmh), mode, DT);
    }
};

TEST_F(DriveTest, WheelRpmToKmh)
{
    // 1000 RPM on a 100 mm tyre: 0.314 m per turn, 18.85 km/h
    EXPECT_NEAR(driveWheelRpmToKmh(1000, 100), 18.85f, 0.01f);
    EXPECT_NEAR(driveWheelRpmToKmh(-1000, 100), 18.85f, 0.01f);
    EXPECT_NEAR(driveWheelRpmToKmh(1000, 50), 9.425f, 0.01f);
}

TEST_F(DriveTest, AutoResolvesByProtocol)
{
    EXPECT_EQ(driveResolveEscMode(DRIVE_ESC_AUTO, true), DRIVE_ESC_FWD_REV);
    EXPECT_EQ(driveResolveEscMode(DRIVE_ESC_AUTO, false), DRIVE_ESC_FWD_BRK_REV);
    EXPECT_EQ(driveResolveEscMode(DRIVE_ESC_FWD_BRK, true), DRIVE_ESC_FWD_BRK);
    EXPECT_EQ(driveResolveEscMode(DRIVE_ESC_FWD_REV, false), DRIVE_ESC_FWD_REV);
    EXPECT_EQ(driveResolveEscMode(DRIVE_ESC_FWD_BRK_REV, true), DRIVE_ESC_FWD_BRK_REV);
    EXPECT_EQ(driveResolveEscMode(DRIVE_ESC_MODE_COUNT, true), DRIVE_ESC_FWD_REV);
}

TEST_F(DriveTest, DefaultsAndValidation)
{
    EXPECT_EQ(driveConfig()->esc_mode, DRIVE_ESC_AUTO);
    EXPECT_EQ(driveConfig()->wheel_diameter, 100);
    EXPECT_EQ(driveConfig()->gyro_min_speed, 20);
    EXPECT_EQ(driveConfig()->gyro_full_speed, 60);
    EXPECT_EQ(driveConfig()->gyro_reverse, 1);
    EXPECT_EQ(driveConfig()->gyro_reverse_gain, 50);

    driveConfigMutable()->esc_mode = 9;
    driveConfigMutable()->wheel_diameter = 5;
    driveConfigMutable()->gyro_min_speed = 80;
    driveConfigMutable()->gyro_full_speed = 40;
    driveConfigMutable()->gyro_reverse_gain = 200;
    driveConfigMutable()->gyro_min_speed = 0;
    validateAndFixDriveConfig();
    // the gyro must be faded out by the stop speed, where the direction can flip
    EXPECT_EQ(driveConfig()->gyro_min_speed, DRIVE_GYRO_MIN_SPEED_MIN);
    EXPECT_EQ(driveConfig()->esc_mode, DRIVE_ESC_AUTO);
    EXPECT_EQ(driveConfig()->wheel_diameter, DRIVE_WHEEL_DIAMETER_MIN);
    EXPECT_GT(driveConfig()->gyro_full_speed, driveConfig()->gyro_min_speed);
    EXPECT_EQ(driveConfig()->gyro_reverse_gain, 100);
}

TEST_F(DriveTest, DisarmedIsNeutralForward)
{
    driveUpdateFrom(false, -1.0f, false, 0, mode, DT);
    EXPECT_EQ(driveGetEscState(), DRIVE_STATE_NEUTRAL);
    EXPECT_EQ(driveGetDirection(), 1);

    // Reversing, then disarm and re-arm: forward again
    run(-1.0f, 0.1f);
    EXPECT_EQ(driveGetDirection(), -1);
    driveUpdateFrom(false, -1.0f, false, 0, mode, DT);
    EXPECT_EQ(driveGetDirection(), 1);
    EXPECT_FALSE(driveIsReversing());
    EXPECT_EQ(driveGetEscState(), DRIVE_STATE_NEUTRAL);
}

TEST_F(DriveTest, DeadbandIsNeutral)
{
    run(0.04f, 0.1f);
    EXPECT_EQ(driveGetEscState(), DRIVE_STATE_NEUTRAL);
    run(-0.04f, 0.1f);
    EXPECT_EQ(driveGetEscState(), DRIVE_STATE_NEUTRAL);
    EXPECT_EQ(driveGetDirection(), 1);
}

//// FWD_BRK_REV

TEST_F(DriveTest, FwdBrkRevNoRpm)
{
    run(0.5f, 0.5f);
    EXPECT_EQ(driveGetEscState(), DRIVE_STATE_FORWARD);
    EXPECT_EQ(driveGetDirection(), 1);

    // Below neutral after forward: brake, still forward
    run(-0.5f, 0.2f);
    EXPECT_EQ(driveGetEscState(), DRIVE_STATE_BRAKE);
    EXPECT_EQ(driveGetDirection(), 1);

    // Neutral dwell, then below again: reverse, at once without RPM
    run(0, 0.1f);
    EXPECT_EQ(driveGetEscState(), DRIVE_STATE_NEUTRAL);
    EXPECT_EQ(driveGetDirection(), 1);
    run(-0.5f, DT);
    EXPECT_EQ(driveGetEscState(), DRIVE_STATE_REVERSE);
    EXPECT_EQ(driveGetDirection(), -1);
    EXPECT_TRUE(driveIsReversing());
    EXPECT_FALSE(driveSpeedKnown());
    EXPECT_EQ(driveGetSpeedKmh(), 0);

    // Neutral and below again: still reverse; forward: forward at once
    run(0, 0.1f);
    run(-0.5f, 0.1f);
    EXPECT_EQ(driveGetEscState(), DRIVE_STATE_REVERSE);
    EXPECT_EQ(driveGetDirection(), -1);
    run(0.5f, DT);
    EXPECT_EQ(driveGetEscState(), DRIVE_STATE_FORWARD);
    EXPECT_EQ(driveGetDirection(), 1);
}

TEST_F(DriveTest, FwdBrkRevBrakeWithoutDwellStaysBrake)
{
    run(0.5f, 0.5f);
    run(-0.5f, 0.1f);
    EXPECT_EQ(driveGetEscState(), DRIVE_STATE_BRAKE);

    // A neutral blip shorter than the dwell: the brake resumes
    run(0, DRIVE_NEUTRAL_DWELL_S * 0.5f);
    run(-0.5f, 0.2f);
    EXPECT_EQ(driveGetEscState(), DRIVE_STATE_BRAKE);
    EXPECT_EQ(driveGetDirection(), 1);
}

TEST_F(DriveTest, FwdBrkRevFirstCommandReverses)
{
    // Nothing commanded since arming: below neutral reverses
    run(-0.5f, 0.01f);
    EXPECT_EQ(driveGetEscState(), DRIVE_STATE_REVERSE);
    EXPECT_EQ(driveGetDirection(), -1);
}

TEST_F(DriveTest, FwdBrkRevWithRpm)
{
    withRpm(20);
    run(0.5f, 0.5f);
    EXPECT_TRUE(driveSpeedKnown());
    EXPECT_NEAR(driveGetSpeedKmh(), 20, 0.1f);
    EXPECT_EQ(driveGetDirection(), 1);

    // Brake while rolling
    run(-0.5f, 0.2f);
    EXPECT_EQ(driveGetEscState(), DRIVE_STATE_BRAKE);
    EXPECT_EQ(driveGetDirection(), 1);

    // Neutral dwell and below again while still rolling forward: reverse commanded, but
    // the car is still travelling forward
    kmh = 10;
    run(0, 0.1f);
    run(-0.5f, 0.2f);
    EXPECT_EQ(driveGetEscState(), DRIVE_STATE_REVERSE);
    EXPECT_EQ(driveGetDirection(), 1);

    // Stopped: now reversing
    kmh = 0;
    run(-0.5f, 0.3f);
    EXPECT_TRUE(driveIsStopped());
    EXPECT_EQ(driveGetDirection(), -1);

    // And it stays so while moving backwards
    kmh = 8;
    run(-0.5f, 0.5f);
    EXPECT_FALSE(driveIsStopped());
    EXPECT_EQ(driveGetDirection(), -1);

    // Forward commanded while still rolling backwards: no change until stopped
    run(0.5f, 0.2f);
    EXPECT_EQ(driveGetEscState(), DRIVE_STATE_FORWARD);
    EXPECT_EQ(driveGetDirection(), -1);
    kmh = 0;
    run(0.5f, 0.3f);
    EXPECT_EQ(driveGetDirection(), 1);
}

TEST_F(DriveTest, FwdBrkRevWithRpmBrakeToStopKeepsDirection)
{
    withRpm(20);
    run(0.5f, 0.5f);
    kmh = 0;
    run(-0.5f, 0.5f);
    EXPECT_TRUE(driveIsStopped());
    EXPECT_EQ(driveGetEscState(), DRIVE_STATE_BRAKE);
    EXPECT_EQ(driveGetDirection(), 1);
}

TEST_F(DriveTest, FwdBrkRevWithRpmWheelsTurningUnderBrakeIsReverse)
{
    // An ESC that reverses from a standstill on the first push (the model expected a brake):
    // the wheels turning under a below-neutral command show it
    withRpm(20);
    run(0.5f, 0.5f);
    kmh = 0;
    run(-0.5f, 0.5f);
    EXPECT_EQ(driveGetDirection(), 1);
    kmh = 5;
    run(-0.5f, 0.1f);
    EXPECT_EQ(driveGetEscState(), DRIVE_STATE_REVERSE);
    EXPECT_EQ(driveGetDirection(), -1);
}

//// FWD_REV

TEST_F(DriveTest, FwdRevNoRpm)
{
    mode = DRIVE_ESC_FWD_REV;
    run(0.5f, 0.5f);
    EXPECT_EQ(driveGetDirection(), 1);
    run(-0.5f, DT);
    EXPECT_EQ(driveGetEscState(), DRIVE_STATE_REVERSE);
    EXPECT_EQ(driveGetDirection(), -1);
    run(0.5f, DT);
    EXPECT_EQ(driveGetEscState(), DRIVE_STATE_FORWARD);
    EXPECT_EQ(driveGetDirection(), 1);
}

TEST_F(DriveTest, FwdRevWithRpmBrakesUntilStopped)
{
    mode = DRIVE_ESC_FWD_REV;
    withRpm(20);
    run(0.5f, 0.5f);

    // Below neutral while rolling forward: braking (or coasting), still forward
    kmh = 10;
    run(-0.5f, 0.5f);
    EXPECT_EQ(driveGetEscState(), DRIVE_STATE_BRAKE);
    EXPECT_EQ(driveGetDirection(), 1);

    // Stopped: reverse
    kmh = 0;
    run(-0.5f, 0.3f);
    EXPECT_EQ(driveGetEscState(), DRIVE_STATE_REVERSE);
    EXPECT_EQ(driveGetDirection(), -1);

    // Forward while rolling backwards: braking, still reversing until stopped
    kmh = 10;
    run(-0.5f, 0.3f);
    run(0.5f, 0.3f);
    EXPECT_EQ(driveGetEscState(), DRIVE_STATE_BRAKE);
    EXPECT_EQ(driveGetDirection(), -1);
    kmh = 0;
    run(0.5f, 0.3f);
    EXPECT_EQ(driveGetEscState(), DRIVE_STATE_FORWARD);
    EXPECT_EQ(driveGetDirection(), 1);
}

TEST_F(DriveTest, FwdRevWithRpmQuickReversalIsCaught)
{
    // The ESC reverses as soon as it is slow enough, before the stop time has run out:
    // the wheels passing through zero still switch the direction
    mode = DRIVE_ESC_FWD_REV;
    withRpm(20);
    run(0.5f, 0.5f);
    kmh = 0;
    run(-0.5f, 0.02f);
    EXPECT_FALSE(driveIsStopped());
    kmh = 6;
    run(-0.5f, 0.1f);
    EXPECT_EQ(driveGetEscState(), DRIVE_STATE_REVERSE);
    EXPECT_EQ(driveGetDirection(), -1);
}

//// FWD_BRK

TEST_F(DriveTest, FwdBrkNeverReversesNoRpm)
{
    mode = DRIVE_ESC_FWD_BRK;
    run(0.5f, 0.5f);
    for (int i = 0; i < 3; i++) {
        run(-0.5f, 0.2f);
        EXPECT_EQ(driveGetEscState(), DRIVE_STATE_BRAKE);
        run(0, 0.2f);
    }
    run(-0.5f, 0.2f);
    EXPECT_EQ(driveGetDirection(), 1);
    EXPECT_FLOAT_EQ(driveGetCorrectionScale(), 1.0f);
}

TEST_F(DriveTest, FwdBrkNeverReversesWithRpm)
{
    mode = DRIVE_ESC_FWD_BRK;
    withRpm(20);
    run(0.5f, 0.5f);
    kmh = 0;
    run(-0.5f, 0.5f);
    run(0, 0.2f);
    run(-0.5f, 0.5f);
    EXPECT_EQ(driveGetEscState(), DRIVE_STATE_BRAKE);
    EXPECT_EQ(driveGetDirection(), 1);
    kmh = 5;
    run(-0.5f, 0.2f);
    EXPECT_EQ(driveGetDirection(), 1);
}

//// Correction scale

TEST_F(DriveTest, SpeedFactorRamp)
{
    // Defaults: 0 below 2 km/h, full from 6 km/h
    const float speeds[] = { 0, 1, 1.9f, 3, 4, 5, 6, 10 };
    const float scales[] = { 0, 0, 0, 0.25f, 0.5f, 0.75f, 1, 1 };

    for (unsigned i = 0; i < sizeof(speeds) / sizeof(speeds[0]); i++) {
        SetUp();
        withRpm(speeds[i]);
        run(0.5f, 0.5f);
        EXPECT_NEAR(driveGetCorrectionScale(), scales[i], 0.01f) << speeds[i] << " km/h";
    }
}

TEST_F(DriveTest, SpeedUnknownIsFullScale)
{
    run(0.5f, 0.5f);
    EXPECT_FLOAT_EQ(driveGetCorrectionScale(), 1.0f);
}

TEST_F(DriveTest, ReverseGain)
{
    // ON with the default 50 %
    run(-0.5f, 2.0f);
    EXPECT_EQ(driveGetDirection(), -1);
    EXPECT_NEAR(driveGetCorrectionScale(), 0.5f, 1e-4f);

    driveConfigMutable()->gyro_reverse_gain = 80;
    run(-0.5f, DT);
    EXPECT_NEAR(driveGetCorrectionScale(), 0.8f, 1e-4f);

    // OFF: no correction in reverse
    driveConfigMutable()->gyro_reverse = 0;
    run(-0.5f, DT);
    EXPECT_EQ(driveGetCorrectionScale(), 0);

    // Forward is untouched by either
    run(0.5f, 2.0f);
    EXPECT_NEAR(driveGetCorrectionScale(), 1.0f, 1e-4f);
}

TEST_F(DriveTest, ReverseGainWithRpmAtSpeed)
{
    withRpm(0);
    run(-0.5f, 0.3f);
    EXPECT_EQ(driveGetDirection(), -1);
    kmh = 10;
    run(-0.5f, 0.5f);
    EXPECT_NEAR(driveGetCorrectionScale(), 0.5f, 1e-3f);
}

TEST_F(DriveTest, DirectionChangeWithoutRpmFadesIn)
{
    mode = DRIVE_ESC_FWD_REV;
    driveConfigMutable()->gyro_reverse_gain = 100;

    run(0.5f, 2.0f);
    EXPECT_FLOAT_EQ(driveGetCorrectionScale(), 1.0f);

    // Reverse commanded: the ESC may still be coasting forward, so no correction at first
    run(-0.5f, DT);
    EXPECT_EQ(driveGetDirection(), -1);
    EXPECT_EQ(driveGetCorrectionScale(), 0);

    run(-0.5f, DRIVE_DIR_CHANGE_RAMP_S * 0.5f);
    EXPECT_NEAR(driveGetCorrectionScale(), 0.5f, 0.01f);

    run(-0.5f, DRIVE_DIR_CHANGE_RAMP_S * 0.5f + 0.01f);
    EXPECT_FLOAT_EQ(driveGetCorrectionScale(), 1.0f);

    // And the same back to forward
    run(0.5f, DT);
    EXPECT_EQ(driveGetCorrectionScale(), 0);
    run(0.5f, DRIVE_DIR_CHANGE_RAMP_S + 0.01f);
    EXPECT_FLOAT_EQ(driveGetCorrectionScale(), 1.0f);
}

TEST_F(DriveTest, DirectionChangeWithRpmHasNoFade)
{
    // With RPM the change happens at a standstill, where the speed factor is already zero
    driveConfigMutable()->gyro_min_speed = 0;
    driveConfigMutable()->gyro_full_speed = 1;
    driveConfigMutable()->gyro_reverse_gain = 100;
    withRpm(0);
    run(-0.5f, 0.3f);
    EXPECT_EQ(driveGetDirection(), -1);
    kmh = 10;
    run(-0.5f, 0.05f);
    EXPECT_FLOAT_EQ(driveGetCorrectionScale(), 1.0f);
}
