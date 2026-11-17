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

    void run(float throttle, float seconds, float steer = 0, bool braking = true, bool enabled = true)
    {
        minOut = 2;
        maxOut = -2;
        releasedTime = 0;
        const int n = lrintf(seconds / DT);
        for (int i = 0; i < n; i++) {
            const float out = driveAidsApplyFrom(enabled, throttle, steer, braking, DT);
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
        const float out = driveAidsApplyFrom(true, -1.0f, 0, true, DT);
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

TEST_F(DriveAidsTest, ConfigIsValidated)
{
    config->abs_mode = 7;
    config->abs_freq = 0;
    config->abs_trigger = 0;
    config->abs_floor = 200;
    config->abs_corner_freq = 50;
    driveAidsConfigsMutable(PID_PROFILE_COUNT - 1)->abs_freq = 99;
    validateAndFixDriveAidsConfig();

    EXPECT_EQ(DRIVE_ABS_FREQ_MAX, driveAidsConfigs(PID_PROFILE_COUNT - 1)->abs_freq);

    EXPECT_EQ(DRIVE_ABS_OFF, config->abs_mode);
    EXPECT_EQ(DRIVE_ABS_FREQ_MIN, config->abs_freq);
    EXPECT_EQ(DRIVE_ABS_TRIGGER_MIN, config->abs_trigger);
    EXPECT_EQ(DRIVE_ABS_FLOOR_MAX, config->abs_floor);
    EXPECT_EQ(DRIVE_ABS_CORNER_FREQ_MAX, config->abs_corner_freq);
}
