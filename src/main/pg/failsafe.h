/*
 * This file is part of Rotorflight.
 *
 * Rotorflight is free software. You can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Rotorflight is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this software. If not, see <https://www.gnu.org/licenses/>.
 */

#pragma once

#include "types.h"
#include "platform.h"

#include "pg/pg.h"

typedef struct {
    uint16_t failsafe_throttle;             // Throttle during failsafe, 1000..2000us: 1500 is neutral, below brakes/reverses, above drives.
    uint16_t failsafe_throttle_low_delay;   // Time throttle stick must have been at neutral to "JustDisarm" instead of "full failsafe procedure".
    uint8_t failsafe_delay;                 // Guard time for failsafe activation after signal lost. 1 step = 0.1sec - 1sec in example (10)
    uint8_t failsafe_off_delay;             // Time for Landing before motors stop in 0.1sec. 1 step = 0.1sec - 20sec in example (200)
    uint8_t failsafe_switch_mode;           // failsafe switch action is 0: Stage 1, 1: Disarms instantly, 2: Stage 2
    uint8_t failsafe_procedure;             // selected full failsafe procedure is 0: hold failsafe_throttle for failsafe_off_delay, then disarm, 1: disarm at once
    uint16_t failsafe_recovery_delay;       // Time (in 0.1sec) of valid rx data (min 200ms PERIOD_RXDATA_RECOVERY) to allow recovering from failsafe procedure
    uint8_t failsafe_stick_threshold;       // unused (GPS rescue removed), kept for the layout
} failsafeConfig_t;

PG_DECLARE(failsafeConfig_t, failsafeConfig);
