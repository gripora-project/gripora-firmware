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

#include <stdint.h>
#include <stdbool.h>

#include "pg/pg.h"
#include "pg/pid.h"

// Drive aids (flight/drive_aids.c): ABS and traction control; speed-based steering will
// follow in this group. One set per PID profile, so a profile switch changes the aids
// with the tune (a track or surface preset).

typedef enum {
    DRIVE_ABS_OFF = 0,
    DRIVE_ABS_STATIC,       // Fixed pulse rate, floor and hold
    DRIVE_ABS_DYNAMIC,      // Rate, floor and hold move with brake depth
    DRIVE_ABS_MODE_COUNT
} driveAbsMode_e;

#define DRIVE_ABS_FREQ_MIN              2       // Hz
#define DRIVE_ABS_FREQ_MAX              33      // Hz
#define DRIVE_ABS_FLOOR_MAX             100     // %
#define DRIVE_ABS_HOLD_MAX              100     // ms
#define DRIVE_ABS_TRIGGER_MIN           5       // %
#define DRIVE_ABS_TRIGGER_MAX           100     // %
#define DRIVE_ABS_CORNER_FREQ_MAX       10      // Hz added at full lock

typedef enum {
    DRIVE_TC_OFF = 0,
    DRIVE_TC_RAMP,          // Forward throttle rises no faster than zero to full in tc_ramp_time
    DRIVE_TC_MODE_COUNT
} driveTcMode_e;

#define DRIVE_TC_RAMP_TIME_MIN          50      // ms, zero to full throttle
#define DRIVE_TC_RAMP_TIME_MAX          2000    // ms

typedef struct driveAidsConfig_s {
    uint8_t  abs_mode;              // driveAbsMode_e
    uint8_t  abs_freq;              // Pulse rate, Hz
    uint8_t  abs_floor;             // Brake during the release part of a pulse, % of the brake command
    uint8_t  abs_hold;              // Full brake per pulse beyond half the cycle, ms
    uint8_t  abs_trigger;           // ABS runs past this brake command, %
    uint8_t  abs_dyn_freq;          // DYNAMIC: pulse rate at full brake, Hz
    uint8_t  abs_dyn_hold;          // DYNAMIC: hold at full brake, ms (0 = off, the hold stays abs_hold)
    uint8_t  abs_corner_freq;       // Pulse rate added at full steering lock, Hz (0 = off)
    uint8_t  abs_corner_hold;       // Hold at full steering lock, ms (0 = off)
    uint8_t  tc_mode;               // driveTcMode_e
    uint16_t tc_ramp_time;          // RAMP: zero to full forward throttle, ms
} driveAidsConfig_t;

PG_DECLARE_ARRAY(driveAidsConfig_t, PID_PROFILE_COUNT, driveAidsConfigs);

void resetDriveAidsConfig(driveAidsConfig_t *config);
void validateAndFixDriveAidsConfig(void);
