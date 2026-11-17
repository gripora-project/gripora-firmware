/*
 * This file is part of Rotorflight.
 *
 * Rotorflight is free software. You can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Rotorflight is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this software. If not, see <https://www.gnu.org/licenses/>.
 */

#include <limits.h>
#include <math.h>
#include <stdbool.h>
#include <stdint.h>

#include "gtest/gtest.h"
#include "gmock/gmock.h"

float throttle;

// Dummies
extern "C" {
#include "pg/pid.h"
#include "flight/pid.h"
#include "sensors/gyro.h"
#include "io/gps.h"
#include "fc/runtime_config.h"

gpsSolutionData_t gpsSol;

bool gyroOverflowDetected(void) { return false; }
void beeperConfirmationBeeps(uint8_t) {}
float getThrottle(void) { return throttle; }
void setpointInitProfile(void) {}
float mixerGetInput(uint8_t) { return 0.0; }
bool mixerSaturated(uint8_t) { return false; }
bool stabilizationBypassed = false;
bool mixerStabilizationBypassed(void) { return stabilizationBypassed; }
uint8_t getCurrentPidProfileIndex(void) { return 0; }
void changePidProfile(uint8_t) {}
} // extern "C"

// Mocks
class MockInterface {
  public:
    MOCK_METHOD(float, getDeflection, (int axis), ());
} *g_mock = nullptr;

extern "C" {
// Fixed 1000Hz
gyro_t gyro = {.targetLooptime = 1000000 / 1000};
pidProfile_t *mockPidProfile = pidProfilesMutable(0);
pidProfile_t *currentPidProfile = mockPidProfile;

// Yaw setpoint has the opposite sign to the stick, as in setpointUpdate()
float getSetpoint(int axis) { return g_mock->getDeflection(axis) * 360 * (axis == 2 ? -1 : 1); }
float getDeflection(int axis) { return g_mock->getDeflection(axis); }
float getRcDeflection(int axis) { return g_mock ? g_mock->getDeflection(axis) : 0; }
}

TEST(Empty, BuildTest) {}

TEST(PIDScaleTest, YawUsesFixedWingGainScale)
{
    EXPECT_FLOAT_EQ(YAW_P_TERM_SCALE, ROLL_P_TERM_SCALE);
    EXPECT_FLOAT_EQ(YAW_P_TERM_SCALE, PITCH_P_TERM_SCALE);
    EXPECT_FLOAT_EQ(YAW_I_TERM_SCALE, ROLL_I_TERM_SCALE);
    EXPECT_FLOAT_EQ(YAW_I_TERM_SCALE, PITCH_I_TERM_SCALE);
    EXPECT_FLOAT_EQ(YAW_F_TERM_SCALE, ROLL_F_TERM_SCALE);
    EXPECT_FLOAT_EQ(YAW_F_TERM_SCALE, PITCH_F_TERM_SCALE);
    EXPECT_FLOAT_EQ(YAW_B_TERM_SCALE, ROLL_B_TERM_SCALE);
    EXPECT_FLOAT_EQ(YAW_B_TERM_SCALE, PITCH_B_TERM_SCALE);
    EXPECT_FLOAT_EQ(YAW_D_TERM_SCALE, PITCH_D_TERM_SCALE);
    EXPECT_FLOAT_EQ(YAW_D_TERM_SCALE, ROLL_D_TERM_SCALE);
}

// 4xN matrix for PID input/output
using PIDIO = std::array<std::vector<float>, 4>;

using ::testing::StrictMock;

class PIDTestBase : public ::testing::Test {
  public:
    void SetUp() override {
        g_mock = &mock;
        throttle = 0;
        pgResetAll();
        pidInit(mockPidProfile);
    }
    void TearDown() override
    {
        for (int axis = 0; axis < 3; axis++) {
            gyro.gyroADCf[axis] = 0;
        }
        throttle = 0;
        g_mock = nullptr;
    }
    PIDIO getResponse(PIDIO &input)
    {
        PIDIO output;

        uint32_t time = 0;
        for (size_t i = 0; i < input[0].size(); i++) {
            time += gyro.targetLooptime;
            for (int axis = 0; axis < 4; axis++) {
                EXPECT_CALL(mock, getDeflection(axis))
                    .WillRepeatedly(testing::Return(input[axis][i]));
            }
            pidController(mockPidProfile, time);
            for (int axis = 0; axis < 4; axis++) {
                output[axis].push_back(pidGetOutput(axis));
            }
        }

        return output;
    }
    // Only one gain set on every axis, the rest zero
    void onlyGains(uint16_t P, uint16_t I, uint16_t F, uint16_t B)
    {
        mockPidProfile->pid_mode = 1;
        for (int axis = 0; axis < 3; axis++) {
            mockPidProfile->pid[axis].P = P;
            mockPidProfile->pid[axis].I = I;
            mockPidProfile->pid[axis].D = 0;
            mockPidProfile->pid[axis].F = F;
            mockPidProfile->pid[axis].B = B;
        }
    }
    StrictMock<MockInterface> mock;
};

TEST_F(PIDTestBase, Mode0)
// Mode 0 sanity check: feedforward only, yaw (steering) only
{
    mockPidProfile->pid_mode = 0;
    mockPidProfile->pid[PID_YAW].F = 1;
    pidLoadProfile(mockPidProfile);

    PIDIO input;
    for (int i = 0; i < 100; i++) {
        for (int axis = 0; axis < 4; axis++) {
            input[axis].push_back(0);
        }
    }
    for (int i = 0; i < 100; i++) {
        for (int axis = 0; axis < 4; axis++) {
            input[axis].push_back(1);
        }
    }

    PIDIO output = getResponse(input);
    for (int i = 0; i < 100; i++) {
        EXPECT_EQ(output[PID_YAW][i], 0);
    }
    for (int i = 100; i < 200; i++) {
        EXPECT_NE(output[PID_YAW][i], 0);
        EXPECT_EQ(output[PID_ROLL][i], 0);
        EXPECT_EQ(output[PID_PITCH][i], 0);
    }
}

TEST_F(PIDTestBase, RollAndPitchNeverStabilised)
// Only yaw is stabilised: roll and pitch hold zero setpoint and output whatever the
// sticks and the gyro do, so they never feed the mixer
{
    onlyGains(100, 100, 100, 100);
    pidInit(mockPidProfile);

    gyro.gyroADCf[PID_ROLL] = 50;
    gyro.gyroADCf[PID_PITCH] = -50;

    PIDIO input;
    for (int i = 0; i < 300; i++) {
        for (int axis = 0; axis < 4; axis++) {
            input[axis].push_back(0.5f);
        }
    }

    PIDIO output = getResponse(input);
    for (int i = 0; i < 300; i++) {
        EXPECT_EQ(output[PID_ROLL][i], 0);
        EXPECT_EQ(output[PID_PITCH][i], 0);
    }
    EXPECT_NE(output[PID_YAW][299], 0);

    for (int axis = PID_ROLL; axis <= PID_PITCH; axis++) {
        EXPECT_EQ(pidGetSetpoint(axis), 0);
        EXPECT_EQ(pidGetAxisData()[axis].axisError, 0);
    }
}

class PIDFBTest : public PIDTestBase {
  public:
};

TEST_F(PIDFBTest, B)
// Basic B test
{
    mockPidProfile->pid_mode = 3;
    mockPidProfile->pid[PID_YAW].P = 0;
    mockPidProfile->pid[PID_YAW].I = 0;
    mockPidProfile->pid[PID_YAW].D = 0;
    mockPidProfile->pid[PID_YAW].F = 0;
    mockPidProfile->pid[PID_YAW].B = 100;
    mockPidProfile->bterm_cutoff[PID_YAW] = 30;
    pidLoadProfile(mockPidProfile);

    PIDIO input;
    for (int i = 0; i < 500; i++) {
        for (int axis = 0; axis < 4; axis++) {
            input[axis].push_back(0);
        }
    }
    for (int i = 0; i < 500; i++) {
        for (int axis = 0; axis < 4; axis++) {
            input[axis].push_back(1);
        }
    }

    PIDIO output = getResponse(input);
    // This test is a NOP so far.
}

class PIDBypassTest : public PIDTestBase {
  public:
    void TearDown() override
    {
        stabilizationBypassed = false;
        PIDTestBase::TearDown();
    }
    // Yaw I-term accumulator after holding a stick step with only I active
    float yawAxisErrorAfterStep(bool bypassed)
    {
        stabilizationBypassed = bypassed;
        onlyGains(0, 100, 0, 0);
        pidInit(mockPidProfile);

        PIDIO input;
        for (int i = 0; i < 200; i++) {
            for (int axis = 0; axis < 4; axis++) {
                input[axis].push_back(0.1f);
            }
        }
        getResponse(input);
        return pidGetAxisData()[PID_YAW].axisError;
    }
};

TEST_F(PIDBypassTest, ItermWindsUpWhileStabilised)
{
    // The yaw setpoint has the opposite sign to the stick
    EXPECT_LT(yawAxisErrorAfterStep(false), 0);
}

TEST_F(PIDBypassTest, ItermHeldWhileGyroOff)
{
    EXPECT_FLOAT_EQ(yawAxisErrorAfterStep(true), 0);
}

class PIDAttenuationTest : public PIDTestBase {
  public:
    // Steady yaw output magnitude with only one term active, after a small stick step
    float yawOutputAt(uint8_t tpaGain, int sample)
    {
        mockPidProfile->fw_tpa_gain = tpaGain;
        pidInit(mockPidProfile);

        PIDIO input;
        for (int i = 0; i < 50; i++) {
            for (int axis = 0; axis < 4; axis++) {
                input[axis].push_back(0);
            }
        }
        for (int i = 0; i < 200; i++) {
            for (int axis = 0; axis < 4; axis++) {
                input[axis].push_back(0.01f);
            }
        }
        return fabsf(getResponse(input)[PID_YAW][sample]);
    }
};

TEST_F(PIDAttenuationTest, ThrottleAttenuationScalesF)
{
    onlyGains(0, 0, 100, 0);
    const float full = yawOutputAt(100, 240);
    const float half = yawOutputAt(50, 240);
    EXPECT_GT(full, 0);
    EXPECT_NEAR(half, full * 0.5f, fabsf(full) * 1e-4f);
}

TEST_F(PIDAttenuationTest, ThrottleAttenuationScalesB)
{
    onlyGains(0, 0, 0, 100);
    const float full = yawOutputAt(100, 52);
    const float half = yawOutputAt(50, 52);
    EXPECT_GT(full, 0);
    EXPECT_NEAR(half, full * 0.5f, fabsf(full) * 1e-4f);
}

TEST_F(PIDAttenuationTest, AttenuationNeverDropsBelowFloor)
{
    // 10 % is below anything the CLI allows, standing in for a curve point near zero
    onlyGains(0, 0, 100, 0);
    const float full = yawOutputAt(100, 240);
    const float floored = yawOutputAt(10, 240);
    EXPECT_GT(full, 0);
    EXPECT_NEAR(floored, full * PID_ATTENUATION_MIN, fabsf(full) * 1e-4f);
}

TEST_F(PIDAttenuationTest, ThrottleCurveUsesThrottleMagnitude)
{
    // Car throttle is signed: brake/reverse below neutral attenuates like the same amount of
    // forward throttle. Curve 1 falls from 100 % at neutral to 50 % at full throttle.
    gainCurvesMutable(0)->count = 2;
    gainCurvesMutable(0)->points[0].x = 0;
    gainCurvesMutable(0)->points[0].y = 100;
    gainCurvesMutable(0)->points[1].x = 1000;
    gainCurvesMutable(0)->points[1].y = 50;
    mockPidProfile->fw_tpa_curve = 1;
    onlyGains(0, 0, 100, 0);

    throttle = 0;
    const float neutral = yawOutputAt(100, 240);
    throttle = 0.8f;
    const float forward = yawOutputAt(100, 240);
    throttle = -0.8f;
    const float reverse = yawOutputAt(100, 240);

    EXPECT_GT(neutral, 0);
    EXPECT_NEAR(forward, neutral * 0.6f, fabsf(neutral) * 1e-4f);
    EXPECT_NEAR(reverse, forward, fabsf(neutral) * 1e-4f);

    // Full reverse is the curve's right edge, not below its left edge
    throttle = -1.0f;
    EXPECT_NEAR(yawOutputAt(100, 240), neutral * 0.5f, fabsf(neutral) * 1e-4f);
}

TEST_F(PIDTestBase, ItermDecaysWithoutAirborneGating)
// A car is always on the ground: I accumulates from arming on and decays at the
// configured rate, with no take-off detection holding it back
{
    onlyGains(0, 100, 0, 0);
    mockPidProfile->iterm_decay_time[PID_YAW] = 100;   // 1 s
    pidInit(mockPidProfile);

    // Stick held: I builds
    PIDIO input;
    for (int i = 0; i < 300; i++) {
        for (int axis = 0; axis < 4; axis++) {
            input[axis].push_back(0.1f);
        }
    }
    getResponse(input);
    const float built = fabsf(pidGetAxisData()[PID_YAW].axisError);
    EXPECT_GT(built, 1);

    // Stick released with the car tracking it: I decays away
    PIDIO release;
    for (int i = 0; i < 3000; i++) {
        for (int axis = 0; axis < 4; axis++) {
            release[axis].push_back(0);
        }
    }
    getResponse(release);
    EXPECT_LT(fabsf(pidGetAxisData()[PID_YAW].axisError), built * 0.1f);
}
