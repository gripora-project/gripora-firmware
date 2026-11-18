/*
 * This file is part of Cleanflight and Betaflight.
 *
 * Cleanflight and Betaflight are free software. You can redistribute
 * this software and/or modify this software under the terms of the
 * GNU General Public License as published by the Free Software
 * Foundation, either version 3 of the License, or (at your option)
 * any later version.
 *
 * Cleanflight and Betaflight are distributed in the hope that they
 * will be useful, but WITHOUT ANY WARRANTY; without even the implied
 * warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this software.
 *
 * If not, see <http://www.gnu.org/licenses/>.
 */

#pragma once

#include <stdbool.h>

#include "common/maths.h"
#include "fc/rc_controls.h"
#include "rx/rx.h"
#include "pg/modes.h"

#define BOXID_NONE 255

typedef enum {
    // ARM flag
    BOXARM = 0,

    // Flight modes
    BOXANGLE,           // reserved (removed) -- kept to avoid renumbering later ids
    BOXHORIZON,         // reserved (HORIZON removed) -- kept to avoid renumbering later ids
    BOXTRAINER,         // reserved (removed) -- kept to avoid renumbering later ids
    BOXATTHOLD,         // reserved (removed) -- kept to avoid renumbering later ids
    BOXALTHOLD,         // reserved (never implemented, removed) -- kept to avoid renumbering later ids
    BOXRESCUE,          // reserved (heli rescue removed) -- kept to avoid renumbering later ids
    BOXGPSRESCUE,       // reserved (redundant alias for BOXRTH removed) -- kept to avoid renumbering later ids
    BOXLOITER,          // reserved (removed) -- kept to avoid renumbering later ids
    BOXRTH,             // reserved (removed) -- kept to avoid renumbering later ids
    BOXFAILSAFE,
    BOXPASSTHROUGH,     // GYRO OFF: raw sticks straight to the outputs, no gyro (was named PASSTHROUGH, SETUP)

    BOXID_FLIGHTMODE_LAST = BOXPASSTHROUGH,

    // RC modes
    BOXPREARM,
    BOXPARALYZE,        // reserved (PARALYZE removed) -- kept to avoid renumbering later ids
    BOXBEEPERON,
    BOXBEEPERMUTE,
    BOXLEDLOW,
    BOXCALIB,           // reserved (never implemented, removed) -- kept to avoid renumbering later ids
    BOXOSD,             // reserved (OSD removed) -- kept to avoid renumbering later ids
    BOXTELEMETRY,
    BOXBEEPGPSCOUNT,
    BOXBLACKBOX,
    BOXBLACKBOXERASE,
    BOXCAMERA1,
    BOXCAMERA2,
    BOXCAMERA3,
    BOXVTXPITMODE,      // reserved (VTX removed) -- kept to avoid renumbering later ids
    BOXVTXCONTROLDISABLE, // reserved (VTX removed) -- kept to avoid renumbering later ids
    BOXSTICKCOMMANDDISABLE, // reserved (STICK COMMANDS DISABLE removed) -- kept to avoid renumbering later ids
    BOXGOVERNOR,        // reserved (removed) -- kept to avoid renumbering later ids
    BOXGOVSUSPEND,      // reserved (heli governor removed) -- kept to avoid renumbering later ids
    BOXGOVBYPASS,       // reserved (heli governor removed) -- kept to avoid renumbering later ids
    BOXUSER1,
    BOXUSER2,
    BOXUSER3,
    BOXUSER4,
    BOXAUTOHOVER,       // reserved (AUTO HOVER removed) -- kept to avoid renumbering later ids
    BOXMANUAL,          // reserved (removed) -- kept to avoid renumbering later ids
    BOXAUTOTRIM,        // reserved (removed) -- kept to avoid renumbering later ids
    BOXTHRUSTVECTOR,    // reserved (removed) -- kept to avoid renumbering later ids
    BOXTVHOLD,          // reserved (removed) -- kept to avoid renumbering later ids
    BOXTRADITIONAL,     // reserved (removed) -- kept to avoid renumbering later ids
    BOXDRIVEAIDSOFF,    // DRIVE AIDS OFF: the throttle/brake command passes through unshaped (flight/drive_aids.c)
    BOXLAUNCH,          // LAUNCH: stages a launch for launch control (flight/drive_aids.c)
    BOXDRIFT,           // DRIFT: steering wheel direct, the gyro only counter-steers (flight/pid.c)

    CHECKBOX_ITEM_COUNT,

    BOXID_RCMODE_LAST = CHECKBOX_ITEM_COUNT - 1

} boxId_e;

typedef enum {
    MODELOGIC_OR = 0,
    MODELOGIC_AND
} modeLogic_e;

// type to hold enough bits for CHECKBOX_ITEM_COUNT. Struct used for value-like behavior
typedef struct boxBitmask_s { uint32_t bits[(CHECKBOX_ITEM_COUNT + 31) / 32]; } boxBitmask_t;

// steps are 5 apart
// a value of 0 corresponds to a channel value of 1500
// a value of 100 corresponds to a channel value of 2000
// a value of -100 corresponds to a channel value of 1000

#define MIN_MODE_RANGE_STEP -125
#define MAX_MODE_RANGE_STEP  125

#define STEP_TO_CHANNEL_VALUE(step)   (1500 + 5 * (step))
#define CHANNEL_VALUE_TO_STEP(value)  (((value) - 1500) / 5)

typedef struct modeActivationProfile_s {
    modeActivationCondition_t modeActivationConditions[MAX_MODE_ACTIVATION_CONDITION_COUNT];
} modeActivationProfile_t;

bool IS_RC_MODE_ACTIVE(boxId_e boxId);
void rcModeUpdate(boxBitmask_t *newState);

void updateActivatedModes(void);
bool isModeActivationConditionPresent(boxId_e modeId);
bool isModeActivationConditionLinked(boxId_e modeId);
void removeModeActivationCondition(boxId_e modeId);
bool isModeActivationConditionConfigured(const modeActivationCondition_t *mac, const modeActivationCondition_t *emptyMac);
void analyzeModeActivationConditions(void);

// Temporary, RAM-only modes forced over MSP. See rc_modes.c.
#define MODE_OVERRIDE_MAX_COUNT        4

bool isModeOverrideAllowed(boxId_e boxId);
bool setModeOverride(const boxBitmask_t *mask, uint16_t timeoutMs);
void clearModeOverride(void);
bool isModeOverrideActive(void);
uint16_t getModeOverride(boxBitmask_t *mask);


static inline bool isRangeUsable(const channelRange_t *range)
{
    return
        range->startStep >= MIN_MODE_RANGE_STEP &&
        range->endStep <= MAX_MODE_RANGE_STEP &&
        range->startStep < range->endStep;
}

static inline bool isRangeActive(uint8_t auxChannelIndex, const channelRange_t *range)
{
    if (isRangeUsable(range)) {
        const uint16_t channelValue = rcInput[auxChannelIndex + CONTROL_CHANNEL_COUNT];
        return (channelValue >= STEP_TO_CHANNEL_VALUE(range->startStep) && channelValue < STEP_TO_CHANNEL_VALUE(range->endStep));
    }

    return false;
}
