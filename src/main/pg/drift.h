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

// Drift (flight/pid.c, the DRIFT mode): while DRIFT is active the steering wheel goes straight to
// the steering servo and the gyro only counter-steers rotation past drift_allow. One set per PID
// profile, so a profile switch changes them with the tune.

// Turn rate past the allowance that gives full counter-lock at drift_gain 100 %
#define DRIFT_FULL_RATE                 360.0f  // deg/s

#define DRIFT_GAIN_MAX                  200     // %
#define DRIFT_ALLOW_MAX                 720     // deg/s
#define DRIFT_LIMIT_MIN                 10      // % of full lock
#define DRIFT_LIMIT_MAX                 100     // % of full lock

typedef struct driftConfig_s {
    uint8_t  gain;          // Counter-steer gain, %: 100 = full lock at DRIFT_FULL_RATE deg/s past the allowance
    uint16_t allow;         // Turn rate the gyro leaves alone, deg/s
    uint8_t  limit;         // Most counter-steer the gyro adds, % of full lock
} driftConfig_t;

PG_DECLARE_ARRAY(driftConfig_t, PID_PROFILE_COUNT, driftConfigs);

void resetDriftConfig(driftConfig_t *config);
void validateAndFixDriftConfig(void);
