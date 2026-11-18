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
float driveThrottle;        // Stabilized throttle the mixer sends to the ESC
bool rpmActive;
float wheelRpm;

// Dummies
extern "C" {
#include "pg/pid.h"
#include "flight/pid.h"
#include "sensors/gyro.h"
#include "io/gps.h"
#include "fc/runtime_config.h"
#include "flight/drive.h"
#include "common/maths.h"
#include "flight/mixer.h"
#include "pg/drive.h"
#include "pg/drift.h"

gpsSolutionData_t gpsSol;

bool gyroOverflowDetected(void) { return false; }
void beeperConfirmationBeeps(uint8_t) {}
float getThrottle(void) { return throttle; }
void setpointInitProfile(void) {}
float mixerGetInput(uint8_t index) { return (index == MIXER_IN_STABILIZED_THROTTLE) ? driveThrottle : 0.0f; }
bool isMotorRpmSourceActive(uint8_t motor) { return rpmActive && motor == 0; }
float getMotor1Speedf(void) { return wheelRpm; }
float getMotor2Speedf(void) { return wheelRpm; }
float getMotorRPMf(uint8_t) { return wheelRpm; }
bool isMotorProtocolDshot(void) { return false; }
bool mixerSaturated(uint8_t) { return false; }
bool stabilizationBypassed = false;
bool mixerStabilizationBypassed(void) { return stabilizationBypassed; }
bool driftActive = false;
bool mixerDriftActive(void) { return driftActive; }
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
        driveThrottle = 0;
        rpmActive = false;
        wheelRpm = 0;
        DISABLE_ARMING_FLAG(ARMED);
        pgResetAll();
        pidInit(mockPidProfile);
    }
    void TearDown() override
    {
        for (int axis = 0; axis < 3; axis++) {
            gyro.gyroADCf[axis] = 0;
        }
        throttle = 0;
        driveThrottle = 0;
        rpmActive = false;
        wheelRpm = 0;
        DISABLE_ARMING_FLAG(ARMED);
        driftActive = false;
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

TEST_F(PIDTestBase, YawGyroReverseFlipsTheCorrection)
// yaw_gyro_reverse turns the steering loop's correction around: the same yaw rotation
// with the sticks centred gives the opposite output
{
    onlyGains(100, 0, 0, 0);
    pidInit(mockPidProfile);

    PIDIO input;
    for (int i = 0; i < 50; i++) {
        for (int axis = 0; axis < 4; axis++) {
            input[axis].push_back(0);
        }
    }

    gyro.gyroADCf[PID_YAW] = 40;
    PIDIO normal = getResponse(input);
    EXPECT_NE(normal[PID_YAW][49], 0);

    pidConfigMutable()->yaw_gyro_reverse = 1;
    pidInit(mockPidProfile);
    PIDIO reversed = getResponse(input);
    EXPECT_FLOAT_EQ(reversed[PID_YAW][49], -normal[PID_YAW][49]);

    // Roll and pitch are untouched by the setting
    EXPECT_EQ(reversed[PID_ROLL][49], 0);
    EXPECT_EQ(reversed[PID_PITCH][49], 0);
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
        ENABLE_ARMING_FLAG(ARMED);

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

TEST_F(PIDTestBase, BenchCorrectsWithoutWindingI)
// Disarmed is the bench: with RPM showing a standstill the gyro still corrects (so turning the
// car by hand shows it), but the yaw I-term stays at zero so nothing carries into arming
{
    onlyGains(100, 100, 0, 0);
    pidInit(mockPidProfile);
    rpmActive = true;
    wheelRpm = 0;

    PIDIO input;
    for (int i = 0; i < 200; i++) {
        for (int axis = 0; axis < 4; axis++) {
            input[axis].push_back(0);
        }
    }

    gyro.gyroADCf[PID_YAW] = 40;
    PIDIO output = getResponse(input);
    EXPECT_NE(output[PID_YAW][199], 0);
    EXPECT_EQ(pidGetAxisData()[PID_YAW].axisError, 0);

    // Armed and standing still, the speed fade takes the correction away
    ENABLE_ARMING_FLAG(ARMED);
    output = getResponse(input);
    EXPECT_EQ(output[PID_YAW][199], 0);
}

TEST_F(PIDTestBase, ItermDecaysWithoutAirborneGating)
// A car is always on the ground: I accumulates from arming on and decays at the
// configured rate, with no take-off detection holding it back
{
    onlyGains(0, 100, 0, 0);
    mockPidProfile->iterm_decay_time[PID_YAW] = 100;   // 1 s
    pidInit(mockPidProfile);
    ENABLE_ARMING_FLAG(ARMED);

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

//// Drive state: reverse and low-speed gyro (flight/drive.c)

class PIDDriveTest : public PIDTestBase {
  public:
    // Constant stick and yaw rotation for some time, armed, at the given throttle
    PIDIO hold(float stick, float seconds)
    {
        ENABLE_ARMING_FLAG(ARMED);
        PIDIO input;
        const int n = lrintf(seconds * 1000);
        for (int i = 0; i < n; i++) {
            for (int axis = 0; axis < 4; axis++) {
                input[axis].push_back(axis == PID_YAW ? stick : 0);
            }
        }
        return getResponse(input);
    }
    // Wheel RPM for a speed with the default 100 mm tyre
    static float rpmFor(float kmh) { return kmh / (M_PIf * 100 * 1e-6f * 60.0f); }
    const pidAxisData_t &yaw() { return pidGetAxisData()[PID_YAW]; }
};

TEST_F(PIDDriveTest, ReverseGivesTheOppositeCorrection)
// The same yaw rotation with the stick centred gives the opposite P and D correction in
// reverse: the yaw response to a steering angle changes sign travelling backwards
{
    driveConfigMutable()->gyro_reverse_gain = 100;
    mockPidProfile->pid_mode = 1;
    mockPidProfile->pid[PID_YAW].P = 100;
    mockPidProfile->pid[PID_YAW].I = 0;
    mockPidProfile->pid[PID_YAW].D = 100;
    mockPidProfile->pid[PID_YAW].F = 0;
    mockPidProfile->pid[PID_YAW].B = 0;

    // A yaw rate that keeps changing, so D has something to see
    auto drive = [&](float throttleCmd) {
        pidInit(mockPidProfile);
        driveThrottle = throttleCmd;
        ENABLE_ARMING_FLAG(ARMED);
        PIDIO input;
        for (int axis = 0; axis < 4; axis++)
            input[axis].push_back(0);
        float P = 0, D = 0;
        for (int i = 0; i < 1500; i++) {
            gyro.gyroADCf[PID_YAW] = 40 + 0.05f * i;
            getResponse(input);
            P = yaw().P;
            D = yaw().D;
        }
        return std::make_pair(P, D);
    };

    const auto forward = drive(0.5f);
    EXPECT_EQ(driveGetDirection(), 1);
    const auto reverse = drive(-0.5f);
    EXPECT_EQ(driveGetDirection(), -1);

    EXPECT_NE(forward.first, 0);
    EXPECT_NE(forward.second, 0);
    EXPECT_NEAR(reverse.first, -forward.first, fabsf(forward.first) * 1e-4f);
    EXPECT_NEAR(reverse.second, -forward.second, fabsf(forward.second) * 1e-3f);
}

TEST_F(PIDDriveTest, StandstillIsStickOnly)
// Speed known and zero: no yaw response to correct with, so P, I and D are zero and I does
// not wind up, while F still follows the stick
{
    onlyGains(100, 100, 100, 0);
    mockPidProfile->pid[PID_YAW].D = 100;
    pidInit(mockPidProfile);

    rpmActive = true;
    wheelRpm = 0;
    driveThrottle = 0.3f;
    gyro.gyroADCf[PID_YAW] = 0;

    hold(0.5f, 1.0f);
    EXPECT_TRUE(driveSpeedKnown());
    EXPECT_EQ(driveGetCorrectionScale(), 0);
    EXPECT_EQ(yaw().P, 0);
    EXPECT_EQ(yaw().I, 0);
    EXPECT_EQ(yaw().D, 0);
    EXPECT_EQ(yaw().axisError, 0);
    EXPECT_NE(yaw().F, 0);
    EXPECT_FLOAT_EQ(pidGetOutput(PID_YAW), yaw().F);

    // Moving: the correction comes in and I builds
    wheelRpm = rpmFor(10);
    hold(0.5f, 0.5f);
    EXPECT_FLOAT_EQ(driveGetCorrectionScale(), 1.0f);
    EXPECT_NE(yaw().P, 0);
    EXPECT_NE(yaw().axisError, 0);
}

TEST_F(PIDDriveTest, ReverseOffIsStickOnly)
// drive_gyro_reverse OFF: no correction in reverse, but the stick still steers
{
    onlyGains(100, 100, 100, 0);
    pidInit(mockPidProfile);
    driveConfigMutable()->esc_mode = DRIVE_ESC_FWD_REV;
    driveConfigMutable()->gyro_reverse = 0;

    driveThrottle = 0.5f;
    gyro.gyroADCf[PID_YAW] = 40;
    hold(0.3f, 1.0f);
    const float forwardF = yaw().F;
    EXPECT_NE(yaw().P, 0);

    driveThrottle = -0.5f;
    hold(0.3f, 2.0f);
    EXPECT_EQ(driveGetDirection(), -1);
    EXPECT_EQ(yaw().P, 0);
    EXPECT_EQ(yaw().I, 0);
    EXPECT_EQ(yaw().D, 0);
    EXPECT_NE(yaw().F, 0);
    EXPECT_FLOAT_EQ(yaw().F, forwardF);
}

TEST_F(PIDDriveTest, DirectionChangeResetsYawIterm)
// The I-term accumulated going forward belongs to that direction: dropped on a change
{
    onlyGains(0, 100, 0, 0);
    pidInit(mockPidProfile);
    driveConfigMutable()->esc_mode = DRIVE_ESC_FWD_REV;

    driveThrottle = 0.5f;
    hold(0.1f, 0.5f);
    EXPECT_GT(fabsf(yaw().axisError), 1);

    // Reverse without RPM: changes at once, the I-term goes, and the faded-in correction
    // holds its accumulation back at first
    driveThrottle = -0.5f;
    hold(0.1f, 0.001f);
    EXPECT_EQ(driveGetDirection(), -1);
    EXPECT_EQ(yaw().axisError, 0);
    EXPECT_EQ(yaw().I, 0);
}

TEST_F(PIDDriveTest, DirectionChangeWithoutRpmFadesCorrectionIn)
{
    onlyGains(100, 0, 0, 0);
    pidInit(mockPidProfile);
    driveConfigMutable()->esc_mode = DRIVE_ESC_FWD_REV;
    driveConfigMutable()->gyro_reverse_gain = 100;
    gyro.gyroADCf[PID_YAW] = 40;

    driveThrottle = 0.5f;
    hold(0, 1.0f);
    const float full = yaw().P;
    EXPECT_NE(full, 0);

    driveThrottle = -0.5f;
    hold(0, 0.001f);
    EXPECT_EQ(yaw().P, 0);

    hold(0, DRIVE_DIR_CHANGE_RAMP_S + 0.5f);
    EXPECT_NEAR(yaw().P, -full, fabsf(full) * 1e-3f);
}


//// DRIFT: steering wheel direct, the gyro only counter-steers (flight/pid.c)

class PIDDriftTest : public PIDDriveTest {
  public:
    void SetUp() override
    {
        PIDDriveTest::SetUp();
        mockPidProfile->pid_mode = 1;
        mockPidProfile->pid[PID_YAW].P = 100;
        mockPidProfile->pid[PID_YAW].I = 100;
        mockPidProfile->pid[PID_YAW].D = 0;
        mockPidProfile->pid[PID_YAW].F = 100;
        mockPidProfile->pid[PID_YAW].B = 0;
        pidInit(mockPidProfile);
        // Moving forward with no RPM source: full correction once faded in
        driveThrottle = 0.5f;
        driftActive = true;
    }
    driftConfig_t *drift() { return driftConfigsMutable(0); }
};

TEST_F(PIDDriftTest, WheelGoesStraightThrough)
// No rotation: the output is the steering wheel itself, with the GYRO OFF sign
{
    gyro.gyroADCf[PID_YAW] = 0;
    hold(0.4f, 1.5f);
    EXPECT_FLOAT_EQ(-0.4f, yaw().F);
    EXPECT_FLOAT_EQ(0, yaw().P);
    EXPECT_FLOAT_EQ(0, yaw().I);
    EXPECT_FLOAT_EQ(-0.4f, yaw().pidSum);
}

TEST_F(PIDDriftTest, RotationWithinTheAllowanceIsLeftAlone)
{
    drift()->allow = 60;
    gyro.gyroADCf[PID_YAW] = 50;
    hold(0, 1.5f);
    EXPECT_FLOAT_EQ(0, yaw().P);
    gyro.gyroADCf[PID_YAW] = -50;
    hold(0, 0.1f);
    EXPECT_FLOAT_EQ(0, yaw().P);
}

TEST_F(PIDDriftTest, CounterSteersPastTheAllowance)
// Same sign as the normal loop's P on a rotation, sized by drift_gain and the allowance
{
    drift()->allow = 30;
    drift()->gain = 100;

    gyro.gyroADCf[PID_YAW] = 30 + DRIFT_FULL_RATE / 4;
    hold(0, 1.5f);
    EXPECT_NEAR(-0.25f, yaw().P, 1e-4f);

    gyro.gyroADCf[PID_YAW] = -(30 + DRIFT_FULL_RATE / 4);
    hold(0, 0.1f);
    EXPECT_NEAR(0.25f, yaw().P, 1e-4f);

    // The normal loop corrects the same rotation the same way round
    driftActive = false;
    pidResetAxisError(PID_YAW);
    hold(0, 0.1f);
    EXPECT_GT(yaw().P, 0);
}

TEST_F(PIDDriftTest, CounterSteerIsLimited)
{
    drift()->allow = 0;
    drift()->gain = 200;
    drift()->limit = 40;
    gyro.gyroADCf[PID_YAW] = 500;
    hold(0, 1.5f);
    EXPECT_NEAR(-0.4f, yaw().P, 1e-5f);
}

TEST_F(PIDDriftTest, NoItermAndACleanExit)
// A sustained error in the normal loop builds I; DRIFT drops it and leaving starts from zero
{
    gyro.gyroADCf[PID_YAW] = 40;
    driftActive = false;
    hold(0, 1.5f);
    EXPECT_NE(0, yaw().axisError);

    driftActive = true;
    hold(0, 0.01f);
    EXPECT_FLOAT_EQ(0, yaw().axisError);
    EXPECT_FLOAT_EQ(0, yaw().I);

    driftActive = false;
    gyro.gyroADCf[PID_YAW] = 0;
    hold(0, 0.001f);
    EXPECT_NEAR(0, yaw().I, 1e-3f);
}

TEST_F(PIDDriftTest, StandstillIsWheelOnly)
// The drive state's correction scale applies: no counter-steer standing still
{
    rpmActive = true;
    wheelRpm = 0;
    drift()->allow = 0;
    gyro.gyroADCf[PID_YAW] = 100;
    hold(-0.3f, 0.5f);
    EXPECT_FLOAT_EQ(0, yaw().P);
    EXPECT_FLOAT_EQ(0.3f, yaw().pidSum);
}

TEST_F(PIDDriftTest, ConfigIsValidated)
{
    drift()->gain = 255;
    drift()->allow = 5000;
    drift()->limit = 0;
    validateAndFixDriftConfig();
    EXPECT_EQ(DRIFT_GAIN_MAX, drift()->gain);
    EXPECT_EQ(DRIFT_ALLOW_MAX, drift()->allow);
    EXPECT_EQ(DRIFT_LIMIT_MIN, drift()->limit);
}
