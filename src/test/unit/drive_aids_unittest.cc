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
#include "fc/rc_modes.h"
#include "pg/pg.h"
#include "pg/drive_aids.h"
#include "flight/drive.h"
#include "flight/drive_aids.h"

uint8_t armingFlags;

// driveAidsApply() is not used here: the tests drive driveAidsApplyFrom() directly
bool IS_RC_MODE_ACTIVE(boxId_e) { return false; }
bool failsafeIsActive(void) { return false; }
float getRcDeflection(int) { return 0; }
driveEscState_e driveGetEscState(void) { return DRIVE_STATE_NEUTRAL; }
bool driveIsStopped(void) { return false; }
uint8_t getCurrentPidProfileIndex(void) { return 0; }
}

static const float DT = 0.001f;

class DriveAidsTest : public ::testing::Test {
protected:
    driveAidsConfig_t *config;

    // Output range seen over a run, and the time spent below the command
    float minOut, maxOut;
    float releasedTime;

    void SetUp() override
    {
        pgResetAll();
        driveAidsReset();
        config = driveAidsConfigsMutable(0);
    }

    void useAbs(uint8_t mode, uint8_t freq, uint8_t floor, uint8_t hold, uint8_t trigger)
    {
        config->abs_mode = mode;
        config->abs_freq = freq;
        config->abs_floor = floor;
        config->abs_hold = hold;
        config->abs_trigger = trigger;
    }

    void run(float throttle, float seconds, float steer = 0, bool braking = true, bool enabled = true, bool launch = false)
    {
        minOut = 2;
        maxOut = -2;
        releasedTime = 0;
        const int n = lrintf(seconds / DT);
        for (int i = 0; i < n; i++) {
            const float out = driveAidsApplyFrom(enabled, throttle, steer, braking, launch, DT);
            minOut = fminf(minOut, out);
            maxOut = fmaxf(maxOut, out);
            if (fabsf(out) < fabsf(throttle) - 1e-4f)
                releasedTime += DT;
        }
    }
};

TEST_F(DriveAidsTest, OffByDefault)
{
    EXPECT_EQ(DRIVE_ABS_OFF, driveAidsConfigs(0)->abs_mode);

    run(-1.0f, 1.0f);
    EXPECT_FLOAT_EQ(-1.0f, minOut);
    EXPECT_FLOAT_EQ(-1.0f, maxOut);
    EXPECT_EQ(DRIVE_AIDS_FLAG_ENABLED, driveAidsGetFlags());
}

TEST_F(DriveAidsTest, ThrottleAndBrakeBelowTriggerPassThrough)
{
    useAbs(DRIVE_ABS_STATIC, 10, 40, 0, 30);

    run(0.8f, 1.0f);
    EXPECT_FLOAT_EQ(0.8f, minOut);
    EXPECT_FLOAT_EQ(0.8f, maxOut);

    run(-0.25f, 1.0f);
    EXPECT_FLOAT_EQ(-0.25f, minOut);
    EXPECT_FLOAT_EQ(-0.25f, maxOut);
    EXPECT_FALSE(driveAidsGetFlags() & DRIVE_AIDS_FLAG_ABS_ARMED);
}

TEST_F(DriveAidsTest, StaticPulsesBetweenBrakeAndFloor)
{
    // 10 Hz, no hold: 50 ms full, 50 ms at 40 % of the brake
    useAbs(DRIVE_ABS_STATIC, 10, 40, 0, 30);

    run(-1.0f, 1.0f);
    EXPECT_FLOAT_EQ(-1.0f, minOut);
    EXPECT_NEAR(-0.4f, maxOut, 1e-5f);
    EXPECT_NEAR(0.5f, releasedTime, 0.01f);
    EXPECT_TRUE(driveAidsGetFlags() & DRIVE_AIDS_FLAG_ABS_ARMED);
}

TEST_F(DriveAidsTest, FirstPartOfAPulseIsFullBrake)
{
    useAbs(DRIVE_ABS_STATIC, 10, 40, 0, 30);

    run(-1.0f, 0.049f);
    EXPECT_FLOAT_EQ(-1.0f, maxOut);
    EXPECT_FALSE(driveAidsGetFlags() & DRIVE_AIDS_FLAG_ABS_ACTIVE);

    run(-1.0f, 0.002f);
    EXPECT_TRUE(driveAidsGetFlags() & DRIVE_AIDS_FLAG_ABS_ACTIVE);
}

TEST_F(DriveAidsTest, HoldLengthensFullBrakeUpToTheLimit)
{
    // 10 Hz, 20 ms hold: 70 ms full, 30 ms released
    useAbs(DRIVE_ABS_STATIC, 10, 40, 20, 30);
    run(-1.0f, 1.0f);
    EXPECT_NEAR(0.3f, releasedTime, 0.01f);

    // 100 ms hold is past the limit: full brake for 85 % of the cycle
    config->abs_hold = 100;
    run(-1.0f, 1.0f);
    EXPECT_NEAR(0.15f, releasedTime, 0.01f);
}

TEST_F(DriveAidsTest, ReleaseNeverReachesNeutral)
{
    // Floor 0 % would release to neutral: a brake-then-reverse ESC would then reverse
    useAbs(DRIVE_ABS_STATIC, 10, 0, 0, 30);

    run(-0.6f, 1.0f);
    EXPECT_NEAR(-DRIVE_ABS_MIN_BRAKE, maxOut, 1e-5f);
    EXPECT_LT(maxOut, -DRIVE_THROTTLE_DEADBAND);
}

TEST_F(DriveAidsTest, LightBrakeBelowTheMinimumIsNotPulsed)
{
    useAbs(DRIVE_ABS_STATIC, 10, 0, 0, 5);

    run(-0.08f, 1.0f);
    EXPECT_FLOAT_EQ(-0.08f, minOut);
    EXPECT_FLOAT_EQ(-0.08f, maxOut);
}

TEST_F(DriveAidsTest, OnlyWhileBraking)
{
    // Reversing, or stopped: the command below neutral is not a brake on a rolling car
    useAbs(DRIVE_ABS_STATIC, 10, 40, 0, 30);

    run(-1.0f, 1.0f, 0, false);
    EXPECT_FLOAT_EQ(-1.0f, maxOut);
    EXPECT_FALSE(driveAidsGetFlags() & DRIVE_AIDS_FLAG_ABS_ARMED);
}

TEST_F(DriveAidsTest, DisabledPassesThrough)
{
    useAbs(DRIVE_ABS_STATIC, 10, 40, 0, 30);

    run(-1.0f, 1.0f, 0, true, false);
    EXPECT_FLOAT_EQ(-1.0f, maxOut);
    EXPECT_EQ(0, driveAidsGetFlags());
    EXPECT_FLOAT_EQ(-1.0f, driveAidsGetThrottleIn());
    EXPECT_FLOAT_EQ(-1.0f, driveAidsGetThrottleOut());
}

TEST_F(DriveAidsTest, CycleRestartsWithEachBrake)
{
    useAbs(DRIVE_ABS_STATIC, 10, 40, 0, 30);

    // Into the released part of a pulse, then off the brake and back on: full brake again
    run(-1.0f, 0.07f);
    run(0, 0.01f);
    run(-1.0f, 0.04f);
    EXPECT_FLOAT_EQ(-1.0f, maxOut);
}

TEST_F(DriveAidsTest, DynamicMovesRateAndFloorWithDepth)
{
    // 5 Hz at the trigger, 20 Hz at full brake
    useAbs(DRIVE_ABS_DYNAMIC, 5, 50, 0, 20);
    config->abs_dyn_freq = 20;

    // Full brake: 20 Hz, floor 50 % x 0.6 = 30 %
    run(-1.0f, 1.0f);
    EXPECT_NEAR(-0.3f, maxOut, 1e-5f);
    // 20 cycles, each released for half
    EXPECT_NEAR(0.5f, releasedTime, 0.01f);

    // Count releases: 20 per second
    driveAidsReset();
    int releases = 0;
    bool released = false;
    for (int i = 0; i < 1000; i++) {
        const float out = driveAidsApplyFrom(true, -1.0f, 0, true, false, DT);
        const bool now = out > -0.99f;
        if (now && !released)
            releases++;
        released = now;
    }
    EXPECT_EQ(20, releases);
}

TEST_F(DriveAidsTest, DynamicHoldShortensWithDepth)
{
    // 10 Hz, 40 ms hold: 90 ms full capped to 85 ms. Dynamic hold 0 ms at full brake.
    useAbs(DRIVE_ABS_DYNAMIC, 10, 40, 40, 20);
    config->abs_dyn_freq = 10;

    config->abs_dyn_hold = 0;
    run(-1.0f, 1.0f);
    EXPECT_NEAR(0.15f, releasedTime, 0.01f);

    config->abs_dyn_hold = 1;
    run(-1.0f, 1.0f);
    // 50 + 1 ms full, 49 ms released
    EXPECT_NEAR(0.49f, releasedTime, 0.01f);
}

TEST_F(DriveAidsTest, CornerAddsRateAndShortensHold)
{
    useAbs(DRIVE_ABS_STATIC, 10, 40, 30, 30);
    config->abs_corner_hold = 1;

    // Straight: 80 ms full, 20 ms released
    run(-1.0f, 1.0f, 0);
    EXPECT_NEAR(0.2f, releasedTime, 0.01f);

    // Full lock: hold 1 ms, about half released
    run(-1.0f, 1.0f, -1.0f);
    EXPECT_NEAR(0.49f, releasedTime, 0.01f);
}

TEST_F(DriveAidsTest, SettingsArePerProfile)
{
    // The current profile (0 here) is used; another profile's settings change nothing
    driveAidsConfigsMutable(1)->abs_mode = DRIVE_ABS_STATIC;
    run(-1.0f, 1.0f);
    EXPECT_FLOAT_EQ(-1.0f, maxOut);

    useAbs(DRIVE_ABS_STATIC, 10, 40, 0, 30);
    run(-1.0f, 1.0f);
    EXPECT_NEAR(-0.4f, maxOut, 1e-5f);
}

// Traction control

TEST_F(DriveAidsTest, TractionControlOffByDefault)
{
    EXPECT_EQ(DRIVE_TC_OFF, config->tc_mode);

    run(1.0f, 0.01f, 0, false);
    EXPECT_FLOAT_EQ(1.0f, minOut);
    EXPECT_FALSE(driveAidsGetFlags() & DRIVE_AIDS_FLAG_TC_ARMED);
}

TEST_F(DriveAidsTest, TractionControlLimitsTheRise)
{
    config->tc_mode = DRIVE_TC_RAMP;
    config->tc_ramp_time = 500;

    // A slam to full: half way after 250 ms, full after 500 ms
    run(1.0f, 0.25f, 0, false);
    EXPECT_NEAR(0.5f, maxOut, 0.005f);
    EXPECT_TRUE(driveAidsGetFlags() & DRIVE_AIDS_FLAG_TC_ARMED);
    EXPECT_TRUE(driveAidsGetFlags() & DRIVE_AIDS_FLAG_TC_ACTIVE);

    run(1.0f, 0.26f, 0, false);
    EXPECT_FLOAT_EQ(1.0f, maxOut);
    EXPECT_FALSE(driveAidsGetFlags() & DRIVE_AIDS_FLAG_TC_ACTIVE);

    // A small command below the limit passes at once
    run(0.3f, 0.01f, 0, false);
    EXPECT_FLOAT_EQ(0.3f, minOut);
}

TEST_F(DriveAidsTest, TractionControlLetsOffAtOnce)
{
    config->tc_mode = DRIVE_TC_RAMP;
    config->tc_ramp_time = 500;

    run(1.0f, 1.0f, 0, false);

    // Off the throttle: instant. Back on: rises again from where it was let off to.
    run(0.2f, 0.001f, 0, false);
    EXPECT_FLOAT_EQ(0.2f, maxOut);

    run(1.0f, 0.1f, 0, false);
    EXPECT_NEAR(0.4f, maxOut, 0.005f);
}

TEST_F(DriveAidsTest, TractionControlStartsFromZeroAfterABrake)
{
    config->tc_mode = DRIVE_TC_RAMP;
    config->tc_ramp_time = 500;

    run(1.0f, 1.0f, 0, false);
    run(-0.5f, 0.1f, 0, true);

    run(1.0f, 0.1f, 0, false);
    EXPECT_NEAR(0.2f, maxOut, 0.005f);
}

TEST_F(DriveAidsTest, TractionControlLeavesABrakeFromReverseAlone)
{
    // Above neutral while the ESC brakes a car rolling backwards
    config->tc_mode = DRIVE_TC_RAMP;
    config->tc_ramp_time = 500;

    run(1.0f, 0.1f, 0, true);
    EXPECT_FLOAT_EQ(1.0f, minOut);
    EXPECT_FALSE(driveAidsGetFlags() & DRIVE_AIDS_FLAG_TC_ARMED);
}

TEST_F(DriveAidsTest, TurningAidsOnWithTheTriggerHeldDoesNotCut)
{
    config->tc_mode = DRIVE_TC_RAMP;
    config->tc_ramp_time = 500;

    // DRIVE AIDS OFF at full throttle, then switched back on
    run(1.0f, 0.5f, 0, false, false);
    run(1.0f, 0.01f, 0, false, true);
    EXPECT_FLOAT_EQ(1.0f, minOut);
}

// Launch control: run(throttle, seconds, steer, braking, enabled, launch)
static const bool LAUNCH = true;

TEST_F(DriveAidsTest, LaunchControlOffByDefault)
{
    EXPECT_EQ(DRIVE_LC_OFF, config->lc_mode);

    run(0, 0.1f, 0, false, true, LAUNCH);
    EXPECT_FALSE(driveAidsGetFlags() & DRIVE_AIDS_FLAG_LC_STAGED);
    run(1.0f, 0.01f, 0, false, true, LAUNCH);
    EXPECT_FLOAT_EQ(1.0f, minOut);
}

TEST_F(DriveAidsTest, LaunchFollowsTheRamp)
{
    config->lc_mode = DRIVE_LC_SWITCH;
    config->lc_throttle = 50;
    config->lc_ramp_time = 1000;

    // On the line on the brake with LAUNCH on: staged, the brake passes through
    run(-0.5f, 0.1f, 0, false, true, LAUNCH);
    EXPECT_FLOAT_EQ(-0.5f, minOut);
    EXPECT_TRUE(driveAidsGetFlags() & DRIVE_AIDS_FLAG_LC_STAGED);

    // Full throttle: 50 % at once, 75 % half way, then full
    run(1.0f, 0.001f, 0, false, true, LAUNCH);
    EXPECT_NEAR(0.5f, maxOut, 0.002f);
    EXPECT_TRUE(driveAidsGetFlags() & DRIVE_AIDS_FLAG_LC_ACTIVE);
    EXPECT_FALSE(driveAidsGetFlags() & DRIVE_AIDS_FLAG_LC_STAGED);

    run(1.0f, 0.499f, 0, false, true, LAUNCH);
    EXPECT_NEAR(0.75f, maxOut, 0.002f);

    run(1.0f, 0.51f, 0, false, true, LAUNCH);
    EXPECT_FLOAT_EQ(1.0f, maxOut);
    EXPECT_FALSE(driveAidsGetFlags() & DRIVE_AIDS_FLAG_LC_ACTIVE);

    // A small command below the limit passes at once
    driveAidsReset();
    run(0, 0.01f, 0, false, true, LAUNCH);
    run(0.3f, 0.01f, 0, false, true, LAUNCH);
    EXPECT_FLOAT_EQ(0.3f, minOut);
}

TEST_F(DriveAidsTest, LaunchRunsOncePerActivation)
{
    config->lc_mode = DRIVE_LC_SWITCH;

    run(0, 0.1f, 0, false, true, LAUNCH);
    run(1.0f, 0.2f, 0, false, true, LAUNCH);
    EXPECT_TRUE(driveAidsGetFlags() & DRIVE_AIDS_FLAG_LC_ACTIVE);

    // Letting off ends it; back on the throttle is not capped again
    run(0, 0.1f, 0, false, true, LAUNCH);
    run(1.0f, 0.01f, 0, false, true, LAUNCH);
    EXPECT_FLOAT_EQ(1.0f, minOut);
    EXPECT_FALSE(driveAidsGetFlags() & (DRIVE_AIDS_FLAG_LC_ACTIVE | DRIVE_AIDS_FLAG_LC_STAGED));

    // LAUNCH off and on at neutral stages the next one
    run(0, 0.01f, 0, false, true, false);
    run(0, 0.01f, 0, false, true, LAUNCH);
    EXPECT_TRUE(driveAidsGetFlags() & DRIVE_AIDS_FLAG_LC_STAGED);
}

TEST_F(DriveAidsTest, LaunchSwitchedOnWhileDrivingStagesNothing)
{
    config->lc_mode = DRIVE_LC_SWITCH;

    run(1.0f, 0.1f, 0, false, true, false);
    run(1.0f, 0.1f, 0, false, true, LAUNCH);
    EXPECT_FLOAT_EQ(1.0f, minOut);

    // Still nothing after a lift with LAUNCH left on
    run(0, 0.1f, 0, false, true, LAUNCH);
    run(1.0f, 0.01f, 0, false, true, LAUNCH);
    EXPECT_FLOAT_EQ(1.0f, minOut);
}

TEST_F(DriveAidsTest, LaunchLeavesABrakeFromReverseAndABSAlone)
{
    config->lc_mode = DRIVE_LC_SWITCH;
    useAbs(DRIVE_ABS_STATIC, 10, 40, 0, 30);

    // Staged while still rolling on the brake: ABS keeps pulsing
    run(-1.0f, 0.5f, 0, true, true, LAUNCH);
    EXPECT_TRUE(driveAidsGetFlags() & DRIVE_AIDS_FLAG_LC_STAGED);
    EXPECT_GT(releasedTime, 0.1f);

    // Above neutral while the ESC brakes a car rolling backwards: no launch
    run(1.0f, 0.1f, 0, true, true, LAUNCH);
    EXPECT_FLOAT_EQ(1.0f, minOut);
    EXPECT_TRUE(driveAidsGetFlags() & DRIVE_AIDS_FLAG_LC_STAGED);
}

TEST_F(DriveAidsTest, TractionControlTakesOverAfterALaunch)
{
    config->lc_mode = DRIVE_LC_SWITCH;
    config->lc_throttle = 50;
    config->lc_ramp_time = 1000;
    config->tc_mode = DRIVE_TC_RAMP;
    config->tc_ramp_time = 2000;

    // Traction control stands aside during the launch: its own ramp would be at 5 % here
    run(0, 0.01f, 0, false, true, LAUNCH);
    run(1.0f, 0.1f, 0, false, true, LAUNCH);
    EXPECT_NEAR(0.55f, maxOut, 0.002f);

    // At the end of the launch ramp traction control carries on at full, without a cut
    run(1.0f, 0.95f, 0, false, true, LAUNCH);
    EXPECT_GE(minOut, 0.55f);
    EXPECT_FLOAT_EQ(1.0f, maxOut);
    EXPECT_FALSE(driveAidsGetFlags() & DRIVE_AIDS_FLAG_LC_ACTIVE);
    run(1.0f, 0.1f, 0, false, true, LAUNCH);
    EXPECT_FLOAT_EQ(1.0f, minOut);
}

TEST_F(DriveAidsTest, LaunchStagesWhenArmedWithLaunchOn)
{
    config->lc_mode = DRIVE_LC_SWITCH;

    run(0, 0.1f, 0, false, false, LAUNCH);
    EXPECT_FALSE(driveAidsGetFlags() & DRIVE_AIDS_FLAG_LC_STAGED);
    run(0, 0.01f, 0, false, true, LAUNCH);
    EXPECT_TRUE(driveAidsGetFlags() & DRIVE_AIDS_FLAG_LC_STAGED);
}

TEST_F(DriveAidsTest, ConfigIsValidated)
{
    config->abs_mode = 7;
    config->abs_freq = 0;
    config->abs_trigger = 0;
    config->abs_floor = 200;
    config->abs_corner_freq = 50;
    config->tc_mode = 9;
    config->tc_ramp_time = 5;
    config->lc_mode = 4;
    config->lc_throttle = 0;
    config->lc_ramp_time = 9000;
    driveAidsConfigsMutable(PID_PROFILE_COUNT - 1)->abs_freq = 99;
    validateAndFixDriveAidsConfig();

    EXPECT_EQ(DRIVE_ABS_FREQ_MAX, driveAidsConfigs(PID_PROFILE_COUNT - 1)->abs_freq);
    EXPECT_EQ(DRIVE_TC_OFF, config->tc_mode);
    EXPECT_EQ(DRIVE_TC_RAMP_TIME_MIN, config->tc_ramp_time);
    EXPECT_EQ(DRIVE_LC_OFF, config->lc_mode);
    EXPECT_EQ(DRIVE_LC_THROTTLE_MIN, config->lc_throttle);
    EXPECT_EQ(DRIVE_LC_RAMP_TIME_MAX, config->lc_ramp_time);

    EXPECT_EQ(DRIVE_ABS_OFF, config->abs_mode);
    EXPECT_EQ(DRIVE_ABS_FREQ_MIN, config->abs_freq);
    EXPECT_EQ(DRIVE_ABS_TRIGGER_MIN, config->abs_trigger);
    EXPECT_EQ(DRIVE_ABS_FLOOR_MAX, config->abs_floor);
    EXPECT_EQ(DRIVE_ABS_CORNER_FREQ_MAX, config->abs_corner_freq);
}
