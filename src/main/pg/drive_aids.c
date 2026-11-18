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

#include <stdint.h>
#include <stdbool.h>

#include "platform.h"

#include "common/maths.h"

#include "config/config_reset.h"

#include "pg/pg.h"
#include "pg/pg_ids.h"
#include "pg/drive_aids.h"

PG_REGISTER_ARRAY_WITH_RESET_FN(driveAidsConfig_t, PID_PROFILE_COUNT, driveAidsConfigs, PG_DRIVE_AIDS_CONFIG, 2);

void resetDriveAidsConfig(driveAidsConfig_t *config)
{
    RESET_CONFIG(driveAidsConfig_t, config,
        .abs_mode = DRIVE_ABS_OFF,
        .abs_freq = 10,
        .abs_floor = 40,
        .abs_hold = 20,
        .abs_trigger = 30,
        .abs_dyn_freq = 15,
        .abs_dyn_hold = 0,
        .abs_corner_freq = 0,
        .abs_corner_hold = 0,
        .tc_mode = DRIVE_TC_OFF,
        .tc_ramp_time = 500,
        .lc_mode = DRIVE_LC_OFF,
        .lc_throttle = 50,
        .lc_ramp_time = 1000,
    );
}

void pgResetFn_driveAidsConfigs(driveAidsConfig_t *configs)
{
    for (int i = 0; i < PID_PROFILE_COUNT; i++) {
        resetDriveAidsConfig(&configs[i]);
    }
}

static void validateAndFixDriveAidsProfile(driveAidsConfig_t *config)
{
    if (config->abs_mode >= DRIVE_ABS_MODE_COUNT)
        config->abs_mode = DRIVE_ABS_OFF;

    config->abs_freq = constrain(config->abs_freq, DRIVE_ABS_FREQ_MIN, DRIVE_ABS_FREQ_MAX);
    config->abs_floor = MIN(config->abs_floor, DRIVE_ABS_FLOOR_MAX);
    config->abs_hold = MIN(config->abs_hold, DRIVE_ABS_HOLD_MAX);
    config->abs_trigger = constrain(config->abs_trigger, DRIVE_ABS_TRIGGER_MIN, DRIVE_ABS_TRIGGER_MAX);
    config->abs_dyn_freq = constrain(config->abs_dyn_freq, DRIVE_ABS_FREQ_MIN, DRIVE_ABS_FREQ_MAX);
    config->abs_dyn_hold = MIN(config->abs_dyn_hold, DRIVE_ABS_HOLD_MAX);
    config->abs_corner_freq = MIN(config->abs_corner_freq, DRIVE_ABS_CORNER_FREQ_MAX);
    config->abs_corner_hold = MIN(config->abs_corner_hold, DRIVE_ABS_HOLD_MAX);

    if (config->tc_mode >= DRIVE_TC_MODE_COUNT)
        config->tc_mode = DRIVE_TC_OFF;

    config->tc_ramp_time = constrain(config->tc_ramp_time, DRIVE_TC_RAMP_TIME_MIN, DRIVE_TC_RAMP_TIME_MAX);

    if (config->lc_mode >= DRIVE_LC_MODE_COUNT)
        config->lc_mode = DRIVE_LC_OFF;

    config->lc_throttle = constrain(config->lc_throttle, DRIVE_LC_THROTTLE_MIN, DRIVE_LC_THROTTLE_MAX);
    config->lc_ramp_time = constrain(config->lc_ramp_time, DRIVE_LC_RAMP_TIME_MIN, DRIVE_LC_RAMP_TIME_MAX);
}

void validateAndFixDriveAidsConfig(void)
{
    for (int i = 0; i < PID_PROFILE_COUNT; i++) {
        validateAndFixDriveAidsProfile(driveAidsConfigsMutable(i));
    }
}
