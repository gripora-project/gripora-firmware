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

#include "pg/drive.h"

// Drive state: which way the car is travelling and how fast, for the steering loop.
// See flight/drive.c.

// Throttle within +-this of neutral counts as neutral
#define DRIVE_THROTTLE_DEADBAND     0.05f

// FWD_BRK_REV: neutral held this long after a brake makes the next push below neutral reverse
#define DRIVE_NEUTRAL_DWELL_S       0.05f

// With RPM: "stopped" below this speed for DRIVE_STOP_TIME_S
#define DRIVE_STOP_KMH              1.0f
#define DRIVE_STOP_TIME_S           0.1f

// Low-pass cutoff on the RPM wheel speed
#define DRIVE_SPEED_CUTOFF_HZ       10.0f

// Without RPM: after a change of travel direction the correction fades in from zero over this
// time, since the ESC may coast or brake for a while before it really changes direction
#define DRIVE_DIR_CHANGE_RAMP_S     1.0f

// What the ESC is doing with the throttle command
typedef enum {
    DRIVE_STATE_NEUTRAL = 0,
    DRIVE_STATE_FORWARD,
    DRIVE_STATE_BRAKE,
    DRIVE_STATE_REVERSE,
} driveEscState_e;

void driveReset(void);

// Once per PID loop, before the PID. driveUpdate() reads the mixer, the motors and the arming
// state itself; driveUpdateFrom() takes them directly so the logic can be unit tested.
// escMode is resolved (not AUTO), wheelRpm is the driven wheel's RPM (ignored without RPM).
void driveUpdate(float dT);
void driveUpdateFrom(bool armed, float throttle, bool rpmKnown, float wheelRpm,
                     driveEscMode_e escMode, float dT);

driveEscMode_e driveResolveEscMode(uint8_t configMode, bool dshot);
float driveWheelRpmToKmh(float wheelRpm, uint16_t diameterMm);

driveEscMode_e driveGetEscMode(void);
driveEscState_e driveGetEscState(void);
int8_t driveGetDirection(void);             // +1 forward, -1 reverse
bool driveIsReversing(void);
bool driveSpeedKnown(void);
float driveGetSpeedKmh(void);               // Filtered |speed|, 0 when unknown
bool driveIsStopped(void);                  // RPM-confirmed standstill
float driveGetCorrectionScale(void);        // 0..1, applied to the steering P, I and D
