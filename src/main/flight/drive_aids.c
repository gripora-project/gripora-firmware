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

// Drive aids: shape the pilot's throttle/brake command before it reaches the ESC.
//
// The mixer runs the command through driveAidsApply() and sends the result as
// STABILIZED_THROTTLE (mixer.c). An aid only ever moves the command towards neutral, never
// past it, and never into the opposite direction. Failsafe replaces the result after the aids,
// so a failsafe throttle is never shaped.
//
// The aids are off, and the command passes through unchanged, while disarmed, in failsafe and
// while the DRIVE AIDS OFF mode is active. Each aid is also OFF by default. A mixer rule fed
// from the RC throttle input (not STABILIZED_THROTTLE) bypasses them.
//
// ABS (timed): past abs_trigger of brake, the brake pulses. Each cycle (1 / abs_freq) holds the
// brake command for half the cycle plus abs_hold (at most DRIVE_ABS_FULL_MAX of the cycle), then
// eases it to abs_floor % of the command for the rest. DYNAMIC moves the rate towards
// abs_dyn_freq, the floor towards 60 % of abs_floor and the hold towards abs_dyn_hold as the
// brake deepens from the trigger to full. abs_corner_freq adds rate and abs_corner_hold moves the
// hold with steering lock. Where both a DYNAMIC and a steering hold apply, the shorter one wins.
//
// ABS runs only while the drive state (flight/drive.c) has the ESC braking a car that is not
// RPM-confirmed stopped. A brake-then-reverse ESC that sees neutral in the middle of a brake
// reverses on the next pull, so the released brake never drops below DRIVE_ABS_MIN_BRAKE: the
// command stays on the brake side of neutral, and the drive state stays BRAKE with it.
//
// The settings are per PID profile (the current one is used).
//
// The pulse timing uses the pilot's inputs and time only. Sensor-based ABS (wheel slip from the
// motor RPM) will be a separate mode.

#include <stdbool.h>
#include <stdint.h>
#include <math.h>

#include "platform.h"

#include "build/debug.h"

#include "common/maths.h"

#include "config/config.h"

#include "fc/rc.h"
#include "fc/rc_modes.h"
#include "fc/runtime_config.h"

#include "flight/drive.h"
#include "flight/failsafe.h"

#include "pg/drive_aids.h"

#include "drive_aids.h"

typedef struct {
    float throttleIn;
    float throttleOut;
    uint8_t flags;

    float absPhase;             // Seconds into the current ABS cycle
} driveAidsState_t;

static FAST_DATA_ZERO_INIT driveAidsState_t aids;

void driveAidsReset(void)
{
    aids.absPhase = 0;
    aids.flags = 0;
}

// The brake the release part of an ABS pulse keeps: floor share of the brake, but never so
// little that the ESC sees neutral, and never more than the brake itself
static float driveAbsReleasedBrake(float brake, float floor)
{
    return fminf(brake, fmaxf(brake * floor, DRIVE_ABS_MIN_BRAKE));
}

// ABS on a brake command (0..1). Returns the brake to send.
static float driveAbsApply(float brake, float steer, bool braking, float dT)
{
    const driveAidsConfig_t *config = driveAidsConfigs(getCurrentPidProfileIndex());
    const float trigger = config->abs_trigger * 0.01f;

    if (config->abs_mode == DRIVE_ABS_OFF || !braking || brake < trigger) {
        aids.absPhase = 0;
        return brake;
    }

    float freq = config->abs_freq;
    float floor = config->abs_floor * 0.01f;
    float hold = config->abs_hold * 0.001f;

    if (config->abs_mode == DRIVE_ABS_DYNAMIC) {
        const float depth = (trigger < 1.0f) ? constrainf((brake - trigger) / (1.0f - trigger), 0, 1) : 1.0f;

        freq += (config->abs_dyn_freq - freq) * depth;
        floor *= 1.0f - 0.4f * depth;

        if (config->abs_dyn_hold) {
            const float dynHold = hold + (config->abs_dyn_hold * 0.001f - hold) * depth;
            hold = fminf(hold, dynHold);
        }
    }

    const float lock = fminf(fabsf(steer), 1.0f);

    if (config->abs_corner_freq) {
        freq += config->abs_corner_freq * lock;
    }
    if (config->abs_corner_hold) {
        const float cornerHold = hold + (config->abs_corner_hold * 0.001f - hold) * lock;
        hold = fminf(hold, cornerHold);
    }

    freq = constrainf(freq, DRIVE_ABS_FREQ_MIN, DRIVE_ABS_FREQ_LIMIT);

    const float period = 1.0f / freq;
    const float fullTime = fminf(period * 0.5f + hold, period * DRIVE_ABS_FULL_MAX);

    aids.absPhase += dT;
    if (aids.absPhase >= period)
        aids.absPhase = fmodf(aids.absPhase, period);

    aids.flags |= DRIVE_AIDS_FLAG_ABS_ARMED;

    DEBUG(DRIVE_AIDS, 3, lrintf(freq * 10));
    DEBUG(DRIVE_AIDS, 4, lrintf(floor * 1000));
    DEBUG(DRIVE_AIDS, 5, lrintf(hold * 1000));
    DEBUG(DRIVE_AIDS, 6, lrintf(aids.absPhase * 1000));

    if (aids.absPhase < fullTime)
        return brake;

    const float released = driveAbsReleasedBrake(brake, floor);
    if (released < brake)
        aids.flags |= DRIVE_AIDS_FLAG_ABS_ACTIVE;

    return released;
}

float driveAidsApplyFrom(bool enabled, float throttle, float steer, bool braking, float dT)
{
    aids.throttleIn = throttle;
    aids.flags = 0;

    float out = throttle;

    if (!enabled) {
        driveAidsReset();
    }
    else {
        aids.flags |= DRIVE_AIDS_FLAG_ENABLED;

        // Brake is below neutral. Above it ABS has nothing to do, and its cycle restarts.
        if (throttle < 0) {
            out = -driveAbsApply(-throttle, steer, braking, dT);
        }
        else {
            aids.absPhase = 0;
        }
    }

    aids.throttleOut = out;

    DEBUG(DRIVE_AIDS, 0, lrintf(throttle * 1000));
    DEBUG(DRIVE_AIDS, 1, lrintf(out * 1000));
    DEBUG(DRIVE_AIDS, 2, aids.flags);
    DEBUG(DRIVE_AIDS, 7, lrintf(steer * 1000));

    return out;
}

float driveAidsApply(float throttle, float dT)
{
    const bool enabled = ARMING_FLAG(ARMED) && !failsafeIsActive() && !IS_RC_MODE_ACTIVE(BOXDRIVEAIDSOFF);

    // Braking a car that is still rolling: the ESC brakes, and RPM (when there is one) does not
    // show it stopped
    const bool braking = (driveGetEscState() == DRIVE_STATE_BRAKE) && !driveIsStopped();

    return driveAidsApplyFrom(enabled, throttle, getRcDeflection(FD_YAW), braking, dT);
}

uint8_t driveAidsGetFlags(void)
{
    return aids.flags;
}

float driveAidsGetThrottleIn(void)
{
    return aids.throttleIn;
}

float driveAidsGetThrottleOut(void)
{
    return aids.throttleOut;
}
