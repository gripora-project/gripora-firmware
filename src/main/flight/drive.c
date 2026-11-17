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

// Drive state for the steering loop.
//
// The yaw response to a steering angle is proportional to the signed vehicle speed
// (yaw rate ~ v * tan(steer) / wheelbase). Reversing flips its sign, so a correction tuned
// going forward becomes positive feedback; near standstill there is no response at all, so
// with the stick held over the I-term winds up and slams the steering. This module estimates
// the travel direction and speed so the PID can (pid.c):
//
//  - negate the yaw gyro rate while travelling backwards. The stick keeps meaning "wheel
//    angle" in both directions and F/B are left alone, so correcting properly in reverse is
//    exactly P = Kp * (setpoint - direction * gyro), and likewise for I and D;
//  - scale P, I and D (not F and B) by driveGetCorrectionScale(): faded out at low speed,
//    reduced or off in reverse, and faded back in after an unconfirmed direction change.
//
// Inputs: the stabilized throttle the mixer sends to the ESC (failsafe included) and, when
// the motor has an RPM source, the wheel speed from it. RPM is a magnitude: no RPM source
// reports a direction, so the direction comes from what the ESC was commanded to do.
//
// ESC model (drive_esc_mode): FWD_BRK_REV is the usual car ESC: below neutral brakes after
// forward, back to neutral and below again reverses. FWD_REV reverses straight away below
// neutral; while still rolling forward the ESC brakes or coasts until slow (AM32
// bidirectional with "RC car reverse" off). FWD_BRK never reverses. AUTO picks FWD_REV on
// DShot (the AM32 default), FWD_BRK_REV otherwise; an AM32 with "RC car reverse" on needs
// FWD_BRK_REV set by hand.
//
// With RPM the travel direction only changes at zero speed: an RPM-confirmed standstill, or
// the moment the wheels pass through zero, where the correction is already faded out. Braking
// while rolling forward is never taken for reverse. Wheels speeding up from zero under a
// below-neutral command are taken as reverse in every mode that can reverse (a brake cannot
// set them turning), and under an above-neutral command as forward, so a reversal the model
// did not expect (a sensored ESC, an AM32 reversing at its own threshold) is still caught.
//
// Without RPM the direction follows the command at once (FORWARD +1, REVERSE -1; BRAKE and
// NEUTRAL keep it). The ESC may coast or brake for a while before it really changes
// direction, so on each such change the correction scale drops to zero and fades back in
// over DRIVE_DIR_CHANGE_RAMP_S.
//
// Disarmed, the state is held at NEUTRAL and forward, so every arming starts from there.

#include <stdbool.h>
#include <stdint.h>
#include <math.h>

#include "platform.h"

#include "build/debug.h"

#include "common/filter.h"
#include "common/maths.h"

#include "drivers/motor.h"

#include "fc/runtime_config.h"

#include "flight/mixer.h"
#include "flight/motors.h"

#include "pg/drive.h"

#include "drive.h"

typedef struct {
    driveEscMode_e escMode;         // Resolved ESC model in use
    driveEscState_e state;          // Commanded state
    driveEscState_e lastActive;     // Last state other than NEUTRAL since reset (NEUTRAL: none)
    float neutralTime;              // Seconds at neutral

    int8_t direction;               // Travel direction, +1 / -1

    bool rpmKnown;
    bool haveSpeed;                 // speedFilter holds a real speed
    pt1Filter_t speedFilter;
    float speed;                    // Filtered |speed|, km/h
    float stopTime;                 // Seconds below DRIVE_STOP_KMH (filtered)
    bool stopped;                   // Below DRIVE_STOP_KMH for DRIVE_STOP_TIME_S
    bool slow;                      // Unfiltered speed below DRIVE_STOP_KMH right now

    float rampTime;                 // Seconds since an unconfirmed direction change
    float scale;                    // Steering P, I and D correction scale
} driveState_t;

static FAST_DATA_ZERO_INIT driveState_t drive;

void driveReset(void)
{
    drive.state = DRIVE_STATE_NEUTRAL;
    drive.lastActive = DRIVE_STATE_NEUTRAL;
    drive.neutralTime = DRIVE_NEUTRAL_DWELL_S;
    drive.direction = 1;
    drive.rampTime = DRIVE_DIR_CHANGE_RAMP_S;
}

driveEscMode_e driveResolveEscMode(uint8_t configMode, bool dshot)
{
    switch (configMode) {
        case DRIVE_ESC_FWD_BRK_REV:
        case DRIVE_ESC_FWD_REV:
        case DRIVE_ESC_FWD_BRK:
            return configMode;
        default:
            return dshot ? DRIVE_ESC_FWD_REV : DRIVE_ESC_FWD_BRK_REV;
    }
}

float driveWheelRpmToKmh(float wheelRpm, uint16_t diameterMm)
{
    // Circumference in km per revolution, times revolutions per hour
    return fabsf(wheelRpm) * M_PIf * diameterMm * 1e-6f * 60.0f;
}

static void driveUpdateSpeed(bool rpmKnown, float wheelRpm, float dT)
{
    drive.rpmKnown = rpmKnown;

    if (rpmKnown) {
        const float speed = driveWheelRpmToKmh(wheelRpm, driveConfig()->wheel_diameter);

        pt1FilterUpdateGain(&drive.speedFilter, pt1FilterGain(DRIVE_SPEED_CUTOFF_HZ, 1.0f / dT));
        // Start the filter at the first reading so it doesn't climb from zero
        if (!drive.haveSpeed) {
            drive.speedFilter.y1 = speed;
            drive.haveSpeed = true;
        }
        drive.speed = pt1FilterApply(&drive.speedFilter, speed);
        drive.slow = (speed < DRIVE_STOP_KMH);

        if (drive.speed < DRIVE_STOP_KMH)
            drive.stopTime = fminf(drive.stopTime + dT, DRIVE_STOP_TIME_S);
        else
            drive.stopTime = 0;

        drive.stopped = (drive.stopTime >= DRIVE_STOP_TIME_S);
    }
    else {
        drive.haveSpeed = false;
        drive.speed = 0;
        drive.stopTime = 0;
        drive.stopped = false;
        drive.slow = false;
    }
}

// What the ESC makes of the throttle command. rolling: moving, with RPM to show it.
static driveEscState_e driveNextState(float throttle, bool rolling)
{
    if (throttle > DRIVE_THROTTLE_DEADBAND) {
        // FWD_REV: an opposite command at speed brakes or coasts until slow
        if (drive.escMode == DRIVE_ESC_FWD_REV && rolling && drive.direction < 0)
            return DRIVE_STATE_BRAKE;
        return DRIVE_STATE_FORWARD;
    }

    if (throttle >= -DRIVE_THROTTLE_DEADBAND)
        return DRIVE_STATE_NEUTRAL;

    switch (drive.escMode) {
        case DRIVE_ESC_FWD_BRK:
            return DRIVE_STATE_BRAKE;

        case DRIVE_ESC_FWD_REV:
            return (rolling && drive.direction > 0) ? DRIVE_STATE_BRAKE : DRIVE_STATE_REVERSE;

        default:
            // FWD_BRK_REV: a brake or reverse carries on while the stick stays below neutral
            if (drive.state == DRIVE_STATE_BRAKE || drive.state == DRIVE_STATE_REVERSE)
                return drive.state;
            if (drive.state == DRIVE_STATE_FORWARD)
                return DRIVE_STATE_BRAKE;
            // From neutral: brake after forward; after a brake, reverse once neutral was held
            // long enough, else the brake resumes; reverse again after reverse, and reverse
            // as the first command since arming
            switch (drive.lastActive) {
                case DRIVE_STATE_FORWARD:
                    return DRIVE_STATE_BRAKE;
                case DRIVE_STATE_BRAKE:
                    return (drive.neutralTime >= DRIVE_NEUTRAL_DWELL_S) ? DRIVE_STATE_REVERSE : DRIVE_STATE_BRAKE;
                default:
                    return DRIVE_STATE_REVERSE;
            }
    }
}

static float driveSpeedFactor(void)
{
    if (!drive.rpmKnown)
        return 1.0f;

    const float minSpeed = driveConfig()->gyro_min_speed * 0.1f;
    const float fullSpeed = driveConfig()->gyro_full_speed * 0.1f;

    if (drive.speed < minSpeed)
        return 0;
    if (fullSpeed <= minSpeed || drive.speed >= fullSpeed)
        return 1.0f;

    return (drive.speed - minSpeed) / (fullSpeed - minSpeed);
}

static float driveReverseFactor(void)
{
    if (drive.direction > 0)
        return 1.0f;
    if (!driveConfig()->gyro_reverse)
        return 0;

    return MIN(driveConfig()->gyro_reverse_gain, DRIVE_GYRO_REVERSE_GAIN_MAX) * 0.01f;
}

void driveUpdateFrom(bool armed, float throttle, bool rpmKnown, float wheelRpm,
                     driveEscMode_e escMode, float dT)
{
    const bool wasSlow = drive.slow;

    drive.escMode = escMode;

    driveUpdateSpeed(rpmKnown, wheelRpm, dT);

    if (!armed) {
        driveReset();
    }
    else {
        const int8_t oldDirection = drive.direction;
        // Moving off from (near) zero speed. To change direction the wheels have to pass
        // through zero, which the unfiltered speed shows even when the ESC reverses too
        // quickly for the stopped time to run out.
        const bool movingOff = rpmKnown && wasSlow && !drive.slow;

        drive.state = driveNextState(throttle, rpmKnown && !drive.stopped);

        // Wheels speeding up from zero are driven, whatever the model expected (a brake or a
        // coasting ESC cannot do that): below neutral that is reverse in every mode that can
        // reverse, above neutral it is forward
        if (movingOff) {
            if (throttle < -DRIVE_THROTTLE_DEADBAND && escMode != DRIVE_ESC_FWD_BRK)
                drive.state = DRIVE_STATE_REVERSE;
            else if (throttle > DRIVE_THROTTLE_DEADBAND)
                drive.state = DRIVE_STATE_FORWARD;
        }

        if (drive.state == DRIVE_STATE_NEUTRAL) {
            drive.neutralTime = fminf(drive.neutralTime + dT, DRIVE_NEUTRAL_DWELL_S);
        }
        else {
            drive.neutralTime = 0;
            drive.lastActive = drive.state;
        }

        // Travel direction: with RPM only at (or moving off from) zero speed, without it at once
        if (!rpmKnown || drive.stopped || drive.slow || movingOff) {
            if (drive.state == DRIVE_STATE_FORWARD)
                drive.direction = 1;
            else if (drive.state == DRIVE_STATE_REVERSE)
                drive.direction = -1;
        }

        // Without RPM nothing confirms when the car really changes direction
        if (drive.direction != oldDirection && !rpmKnown)
            drive.rampTime = 0;
        else
            drive.rampTime = fminf(drive.rampTime + dT, DRIVE_DIR_CHANGE_RAMP_S);
    }

    const float rampFactor = drive.rampTime / DRIVE_DIR_CHANGE_RAMP_S;

    // Disarmed is the bench: the motor can't run, and turning the car by hand must show the
    // gyro correcting (the setup wizard's direction check), so the speed fade doesn't apply.
    // Direction is forward and the ramp complete while disarmed; pid.c holds the I-term at
    // zero, so nothing built up on the bench carries into arming.
    const float speedFactor = armed ? driveSpeedFactor() : 1.0f;

    drive.scale = speedFactor * driveReverseFactor() * rampFactor;

    DEBUG(DRIVE, 0, lrintf(throttle * 1000));
    DEBUG(DRIVE, 1, drive.state);
    DEBUG(DRIVE, 2, drive.direction);
    DEBUG(DRIVE, 3, lrintf(drive.speed * 10));
    DEBUG(DRIVE, 4, drive.rpmKnown);
    DEBUG(DRIVE, 5, lrintf(drive.scale * 1000));
    DEBUG(DRIVE, 7, drive.stopped);
}

void driveUpdate(float dT)
{
    // The command the mixer sends to the ESC: failsafe throttle included
    const float throttle = mixerGetInput(MIXER_IN_STABILIZED_THROTTLE);

    // Motor 1, averaged with motor 2 when that has an RPM source too. The motor speeds
    // include each motor's gear ratio (motorN_gear_ratio, output shaft : motor).
    const bool rpmKnown = isMotorRpmSourceActive(0);
    float wheelRpm = 0;

    if (rpmKnown) {
        wheelRpm = fabsf(getMotor1Speedf());
        if (isMotorRpmSourceActive(1))
            wheelRpm = (wheelRpm + fabsf(getMotor2Speedf())) * 0.5f;
    }

    const driveEscMode_e escMode = driveResolveEscMode(driveConfig()->esc_mode, isMotorProtocolDshot());

    driveUpdateFrom(ARMING_FLAG(ARMED), throttle, rpmKnown, wheelRpm, escMode, dT);

    DEBUG(DRIVE, 6, lrintf(getMotorRPMf(0)));
}

driveEscMode_e driveGetEscMode(void)
{
    return drive.escMode;
}

driveEscState_e driveGetEscState(void)
{
    return drive.state;
}

int8_t driveGetDirection(void)
{
    return (drive.direction < 0) ? -1 : 1;
}

bool driveIsReversing(void)
{
    return drive.direction < 0;
}

bool driveSpeedKnown(void)
{
    return drive.rpmKnown;
}

float driveGetSpeedKmh(void)
{
    return drive.rpmKnown ? drive.speed : 0;
}

bool driveIsStopped(void)
{
    return drive.stopped;
}

float driveGetCorrectionScale(void)
{
    return drive.scale;
}
