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

#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "pg/drive_aids.h"

// Drive aids: shape the throttle/brake command before it reaches the ESC. See flight/drive_aids.c.

// The brake an ABS pulse releases to never drops below this (throttle units, 1 = full brake).
// Below it a brake-then-reverse ESC could see neutral, and the next full brake would reverse.
#define DRIVE_ABS_MIN_BRAKE         0.10f

// Highest pulse rate after the DYNAMIC and steering adjustments
#define DRIVE_ABS_FREQ_LIMIT        40.0f

// Longest full-brake part of a pulse, as a share of the cycle
#define DRIVE_ABS_FULL_MAX          0.85f

// driveAidsGetFlags() bits
#define DRIVE_AIDS_FLAG_ENABLED     (1 << 0)    // Aids on: armed, no failsafe, DRIVE AIDS OFF not active
#define DRIVE_AIDS_FLAG_ABS_ARMED   (1 << 1)    // ABS set up and braking past the trigger
#define DRIVE_AIDS_FLAG_ABS_ACTIVE  (1 << 2)    // ABS releasing the brake right now
#define DRIVE_AIDS_FLAG_TC_ARMED    (1 << 3)    // Traction control set up and driving forward
#define DRIVE_AIDS_FLAG_TC_ACTIVE   (1 << 4)    // Traction control holding the throttle back right now
#define DRIVE_AIDS_FLAG_LC_STAGED   (1 << 5)    // Launch control staged: the next forward throttle launches
#define DRIVE_AIDS_FLAG_LC_ACTIVE   (1 << 6)    // Launch control launching: the throttle follows the launch ramp

void driveAidsReset(void);

// Once per loop, on the pilot's throttle command (-1..1, below zero brake/reverse), returning
// the command to send on. driveAidsApply() reads the arming state, the DRIVE AIDS OFF mode, the
// steering and the drive state itself; driveAidsApplyFrom() takes them so the logic can be unit
// tested. steer is the steering deflection, -1..1; braking is true while the ESC brakes a car
// that is still rolling (flight/drive.c); above neutral that is a brake from reverse, which
// traction control and launch control leave alone. launch is true while the LAUNCH mode is
// active.
float driveAidsApply(float throttle, float dT);
float driveAidsApplyFrom(bool enabled, float throttle, float steer, bool braking, bool launch, float dT);

uint8_t driveAidsGetFlags(void);
float driveAidsGetThrottleIn(void);
float driveAidsGetThrottleOut(void);
