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

// How the ESC treats a command below neutral (flight/drive.c models it)
typedef enum {
    DRIVE_ESC_AUTO = 0,     // DShot protocols: FWD_REV, otherwise FWD_BRK_REV
    DRIVE_ESC_FWD_BRK_REV,  // Below neutral brakes after forward; neutral, then below again reverses
    DRIVE_ESC_FWD_REV,      // Below neutral reverses (coasting or braking first while still rolling)
    DRIVE_ESC_FWD_BRK,      // Below neutral only ever brakes
    DRIVE_ESC_MODE_COUNT
} driveEscMode_e;

#define DRIVE_WHEEL_DIAMETER_MIN        20      // mm
#define DRIVE_WHEEL_DIAMETER_MAX        300     // mm

// drive_gyro_min_speed / drive_gyro_full_speed, 0.1 km/h. full is always above min.
#define DRIVE_GYRO_SPEED_MAX            2000    // 200 km/h
// min is never below the stop speed (flight/drive.h DRIVE_STOP_KMH): with RPM the travel
// direction flips there, and the correction must already be faded to zero when it does.
#define DRIVE_GYRO_MIN_SPEED_MIN        10      // 1.0 km/h

#define DRIVE_GYRO_REVERSE_GAIN_MAX     100     // %

typedef struct driveConfig_s {
    uint8_t  esc_mode;              // driveEscMode_e
    uint16_t wheel_diameter;        // Driven tyre diameter, mm
    uint16_t gyro_min_speed;        // Below this the gyro does not correct, 0.1 km/h
    uint16_t gyro_full_speed;       // Full gyro correction from this speed up, 0.1 km/h
    uint8_t  gyro_reverse;          // Gyro correction while reversing, OFF/ON
    uint8_t  gyro_reverse_gain;     // Correction while reversing, % of forward
} driveConfig_t;

PG_DECLARE(driveConfig_t, driveConfig);

void validateAndFixDriveConfig(void);
