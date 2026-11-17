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

#include "pg/pg.h"
#include "pg/pg_ids.h"
#include "pg/drive.h"

PG_REGISTER_WITH_RESET_TEMPLATE(driveConfig_t, driveConfig, PG_DRIVE_CONFIG, 0);

PG_RESET_TEMPLATE(driveConfig_t, driveConfig,
    .esc_mode = DRIVE_ESC_AUTO,
    .wheel_diameter = 100,
    .gyro_min_speed = 20,       // 2.0 km/h
    .gyro_full_speed = 60,      // 6.0 km/h
    .gyro_reverse = 1,
    .gyro_reverse_gain = 50,
);

void validateAndFixDriveConfig(void)
{
    driveConfig_t *config = driveConfigMutable();

    if (config->esc_mode >= DRIVE_ESC_MODE_COUNT)
        config->esc_mode = DRIVE_ESC_AUTO;

    config->wheel_diameter = constrain(config->wheel_diameter, DRIVE_WHEEL_DIAMETER_MIN, DRIVE_WHEEL_DIAMETER_MAX);
    config->gyro_min_speed = constrain(config->gyro_min_speed, DRIVE_GYRO_MIN_SPEED_MIN, DRIVE_GYRO_SPEED_MAX - 1);
    config->gyro_full_speed = constrain(config->gyro_full_speed, config->gyro_min_speed + 1, DRIVE_GYRO_SPEED_MAX);
    config->gyro_reverse = config->gyro_reverse ? 1 : 0;
    config->gyro_reverse_gain = MIN(config->gyro_reverse_gain, DRIVE_GYRO_REVERSE_GAIN_MAX);
}
