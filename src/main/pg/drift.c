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
#include "pg/drift.h"

PG_REGISTER_ARRAY_WITH_RESET_FN(driftConfig_t, PID_PROFILE_COUNT, driftConfigs, PG_DRIFT_CONFIG, 0);

void resetDriftConfig(driftConfig_t *config)
{
    RESET_CONFIG(driftConfig_t, config,
        .gain = 100,
        .allow = 30,
        .limit = 100,
    );
}

void pgResetFn_driftConfigs(driftConfig_t *configs)
{
    for (int i = 0; i < PID_PROFILE_COUNT; i++) {
        resetDriftConfig(&configs[i]);
    }
}

void validateAndFixDriftConfig(void)
{
    for (int i = 0; i < PID_PROFILE_COUNT; i++) {
        driftConfig_t *config = driftConfigsMutable(i);
        config->gain = MIN(config->gain, DRIFT_GAIN_MAX);
        config->allow = MIN(config->allow, DRIFT_ALLOW_MAX);
        config->limit = constrain(config->limit, DRIFT_LIMIT_MIN, DRIFT_LIMIT_MAX);
    }
}
