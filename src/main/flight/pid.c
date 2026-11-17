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

#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include <math.h>

#include "platform.h"

#include "build/build_config.h"
#include "build/debug.h"

#include "common/axis.h"
#include "common/filter.h"

#include "config/config_reset.h"

#include "pg/pg.h"
#include "pg/pid.h"
#include "pg/adjustments.h"
#include "pg/pg_ids.h"

#include "drivers/pwm_output.h"
#include "drivers/dshot_command.h"
#include "drivers/sound_beeper.h"
#include "drivers/time.h"

#include "sensors/acceleration.h"
#include "sensors/battery.h"
#include "sensors/gyro.h"

#include "fc/core.h"
#include "fc/rc.h"
#include "fc/rc_controls.h"
#include "fc/rc_rates.h"
#include "fc/runtime_config.h"

#include "flight/drive.h"
#include "flight/mixer.h"
#include "flight/rpm_filter.h"
#include "flight/speed_atten.h"

#include "io/gps.h"

#include "pid.h"

static FAST_DATA_ZERO_INIT pidData_t pid;

// Travel direction the yaw I-term was accumulated in (flight/drive.c)
static FAST_DATA_ZERO_INIT int8_t pidYawDirection;


//// Access functions

float pidGetDT(void)
{
    return pid.dT;
}

float pidGetPidFrequency(void)
{
    return pid.freq;
}

float pidGetSetpoint(int axis)
{
    return pid.data[axis].setPoint;
}

float pidGetOutput(int axis)
{
    return pid.data[axis].pidSum;
}

const pidAxisData_t * pidGetAxisData(void)
{
    return pid.data;
}

void INIT_CODE pidReset(void)
{
    memset(pid.data, 0, sizeof(pid.data));
}

void INIT_CODE pidResetAxisError(int axis)
{
    pid.data[axis].I = 0;
    pid.data[axis].axisError = 0;
}

void INIT_CODE pidResetAxisErrors(void)
{
    for (int axis = 0; axis < 3; axis++) {
        pid.data[axis].I = 0;
        pid.data[axis].axisError = 0;
    }
}


// Exponential I-term decay rate (1/s) for a decay time constant in 0.01 s. Clamped because MSP
// writes and CLI array values are not range-checked.
float pidItermDecayRate(uint8_t decayTime)
{
    return 100.0f / constrain(decayTime, ITERM_DECAY_TIME_MIN, ITERM_DECAY_TIME_MAX);
}

// I-term relax setpoint filter cutoff (Hz) for an I-term relax score. A lower cutoff
// treats more of each stick move as "fast" and suppresses I build-up for longer, so less
// bounce-back. Score 5 (default) is the long-standing 10 Hz; 5-9 cover 10-5 Hz, one Hz per
// step. The score is clamped because MSP writes and CLI array values are not range-checked.
uint8_t pidItermRelaxCutoff(uint8_t score)
{
    static const uint8_t cutoffHz[ITERM_RELAX_MAX] = { 50, 30, 20, 15, 10, 8, 7, 6, 5, 3 };

    return cutoffHz[constrain(score, ITERM_RELAX_MIN, ITERM_RELAX_MAX) - 1];
}


//// Adjustment functions

int get_ADJUSTMENT_PID_PROFILE(void)
{
    return getCurrentPidProfileIndex() + 1;
}

void set_ADJUSTMENT_PID_PROFILE(int value)
{
    changePidProfile(value - 1);
}

int get_ADJUSTMENT_MASTER_GAIN_PITCH(void)
{
    return currentPidProfile->master_gain[PID_PITCH];
}

void set_ADJUSTMENT_MASTER_GAIN_PITCH(int value)
{
    currentPidProfile->master_gain[PID_PITCH] = value;
    pid.masterGain[PID_PITCH] = value * 0.01f;
}

int get_ADJUSTMENT_MASTER_GAIN_ROLL(void)
{
    return currentPidProfile->master_gain[PID_ROLL];
}

void set_ADJUSTMENT_MASTER_GAIN_ROLL(int value)
{
    currentPidProfile->master_gain[PID_ROLL] = value;
    pid.masterGain[PID_ROLL] = value * 0.01f;
}

int get_ADJUSTMENT_MASTER_GAIN_YAW(void)
{
    return currentPidProfile->master_gain[PID_YAW];
}

void set_ADJUSTMENT_MASTER_GAIN_YAW(int value)
{
    currentPidProfile->master_gain[PID_YAW] = value;
    pid.masterGain[PID_YAW] = value * 0.01f;
}

int get_ADJUSTMENT_ITERM_DECAY_TIME_ROLL(void)
{
    return currentPidProfile->iterm_decay_time[PID_ROLL];
}

void set_ADJUSTMENT_ITERM_DECAY_TIME_ROLL(int value)
{
    currentPidProfile->iterm_decay_time[PID_ROLL] = value;
    pid.itermDecayRate[PID_ROLL] = pidItermDecayRate(value);
}

int get_ADJUSTMENT_ITERM_DECAY_TIME_PITCH(void)
{
    return currentPidProfile->iterm_decay_time[PID_PITCH];
}

void set_ADJUSTMENT_ITERM_DECAY_TIME_PITCH(int value)
{
    currentPidProfile->iterm_decay_time[PID_PITCH] = value;
    pid.itermDecayRate[PID_PITCH] = pidItermDecayRate(value);
}

int get_ADJUSTMENT_ITERM_DECAY_TIME_YAW(void)
{
    return currentPidProfile->iterm_decay_time[PID_YAW];
}

void set_ADJUSTMENT_ITERM_DECAY_TIME_YAW(int value)
{
    currentPidProfile->iterm_decay_time[PID_YAW] = value;
    pid.itermDecayRate[PID_YAW] = pidItermDecayRate(value);
}

int get_ADJUSTMENT_ITERM_RELAX_ROLL(void)
{
    return currentPidProfile->iterm_relax[PID_ROLL];
}

void set_ADJUSTMENT_ITERM_RELAX_ROLL(int value)
{
    currentPidProfile->iterm_relax[PID_ROLL] = value;
    pt1FilterUpdate(&pid.relaxFilter[PID_ROLL], pidItermRelaxCutoff(value), pid.freq);
}

int get_ADJUSTMENT_ITERM_RELAX_PITCH(void)
{
    return currentPidProfile->iterm_relax[PID_PITCH];
}

void set_ADJUSTMENT_ITERM_RELAX_PITCH(int value)
{
    currentPidProfile->iterm_relax[PID_PITCH] = value;
    pt1FilterUpdate(&pid.relaxFilter[PID_PITCH], pidItermRelaxCutoff(value), pid.freq);
}

int get_ADJUSTMENT_ITERM_RELAX_YAW(void)
{
    return currentPidProfile->iterm_relax[PID_YAW];
}

void set_ADJUSTMENT_ITERM_RELAX_YAW(int value)
{
    currentPidProfile->iterm_relax[PID_YAW] = value;
    pt1FilterUpdate(&pid.relaxFilter[PID_YAW], pidItermRelaxCutoff(value), pid.freq);
}

int get_ADJUSTMENT_PITCH_P_GAIN(void)
{
    return currentPidProfile->pid[PID_PITCH].P;
}

void set_ADJUSTMENT_PITCH_P_GAIN(int value)
{
    currentPidProfile->pid[PID_PITCH].P = value;
    pid.coef[PID_PITCH].Kp = PITCH_P_TERM_SCALE * value;
}

int get_ADJUSTMENT_ROLL_P_GAIN(void)
{
    return currentPidProfile->pid[PID_ROLL].P;
}

void set_ADJUSTMENT_ROLL_P_GAIN(int value)
{
    currentPidProfile->pid[PID_ROLL].P = value;
    pid.coef[PID_ROLL].Kp = ROLL_P_TERM_SCALE * value;
}

int get_ADJUSTMENT_YAW_P_GAIN(void)
{
    return currentPidProfile->pid[PID_YAW].P;
}

void set_ADJUSTMENT_YAW_P_GAIN(int value)
{
    currentPidProfile->pid[PID_YAW].P = value;
    pid.coef[PID_YAW].Kp = YAW_P_TERM_SCALE * value;
}

int get_ADJUSTMENT_PITCH_I_GAIN(void)
{
    return currentPidProfile->pid[PID_PITCH].I;
}

void set_ADJUSTMENT_PITCH_I_GAIN(int value)
{
    currentPidProfile->pid[PID_PITCH].I = value;
    pid.coef[PID_PITCH].Ki = PITCH_I_TERM_SCALE * value;
}

int get_ADJUSTMENT_ROLL_I_GAIN(void)
{
    return currentPidProfile->pid[PID_ROLL].I;
}

void set_ADJUSTMENT_ROLL_I_GAIN(int value)
{
    currentPidProfile->pid[PID_ROLL].I = value;
    pid.coef[PID_ROLL].Ki = ROLL_I_TERM_SCALE * value;
}

int get_ADJUSTMENT_YAW_I_GAIN(void)
{
    return currentPidProfile->pid[PID_YAW].I;
}

void set_ADJUSTMENT_YAW_I_GAIN(int value)
{
    currentPidProfile->pid[PID_YAW].I = value;
    pid.coef[PID_YAW].Ki = YAW_I_TERM_SCALE * value;
}

int get_ADJUSTMENT_PITCH_D_GAIN(void)
{
    return currentPidProfile->pid[PID_PITCH].D;
}

void set_ADJUSTMENT_PITCH_D_GAIN(int value)
{
    currentPidProfile->pid[PID_PITCH].D = value;
    pid.coef[PID_PITCH].Kd = PITCH_D_TERM_SCALE * value;
}

int get_ADJUSTMENT_ROLL_D_GAIN(void)
{
    return currentPidProfile->pid[PID_ROLL].D;
}

void set_ADJUSTMENT_ROLL_D_GAIN(int value)
{
    currentPidProfile->pid[PID_ROLL].D = value;
    pid.coef[PID_ROLL].Kd = ROLL_D_TERM_SCALE * value;
}

int get_ADJUSTMENT_YAW_D_GAIN(void)
{
    return currentPidProfile->pid[PID_YAW].D;
}

void set_ADJUSTMENT_YAW_D_GAIN(int value)
{
    currentPidProfile->pid[PID_YAW].D = value;
    pid.coef[PID_YAW].Kd = YAW_D_TERM_SCALE * value;
}

int get_ADJUSTMENT_PITCH_F_GAIN(void)
{
    return currentPidProfile->pid[PID_PITCH].F;
}

void set_ADJUSTMENT_PITCH_F_GAIN(int value)
{
    currentPidProfile->pid[PID_PITCH].F = value;
    pid.coef[PID_PITCH].Kf = PITCH_F_TERM_SCALE * value;
}

int get_ADJUSTMENT_ROLL_F_GAIN(void)
{
    return currentPidProfile->pid[PID_ROLL].F;
}

void set_ADJUSTMENT_ROLL_F_GAIN(int value)
{
    currentPidProfile->pid[PID_ROLL].F = value;
    pid.coef[PID_ROLL].Kf = ROLL_F_TERM_SCALE * value;
}

int get_ADJUSTMENT_YAW_F_GAIN(void)
{
    return currentPidProfile->pid[PID_YAW].F;
}

void set_ADJUSTMENT_YAW_F_GAIN(int value)
{
    currentPidProfile->pid[PID_YAW].F = value;
    pid.coef[PID_YAW].Kf = YAW_F_TERM_SCALE * value;
}

int get_ADJUSTMENT_PITCH_B_GAIN(void)
{
    return currentPidProfile->pid[PID_PITCH].B;
}

void set_ADJUSTMENT_PITCH_B_GAIN(int value)
{
    currentPidProfile->pid[PID_PITCH].B = value;
    pid.coef[PID_PITCH].Kb = PITCH_B_TERM_SCALE * value;
}

int get_ADJUSTMENT_ROLL_B_GAIN(void)
{
    return currentPidProfile->pid[PID_ROLL].B;
}

void set_ADJUSTMENT_ROLL_B_GAIN(int value)
{
    currentPidProfile->pid[PID_ROLL].B = value;
    pid.coef[PID_ROLL].Kb = ROLL_B_TERM_SCALE * value;
}

int get_ADJUSTMENT_YAW_B_GAIN(void)
{
    return currentPidProfile->pid[PID_YAW].B;
}

void set_ADJUSTMENT_YAW_B_GAIN(int value)
{
    currentPidProfile->pid[PID_YAW].B = value;
    pid.coef[PID_YAW].Kb = YAW_B_TERM_SCALE * value;
}


int get_ADJUSTMENT_PITCH_GYRO_CUTOFF(void)
{
    return currentPidProfile->gyro_cutoff[PID_PITCH];
}

void set_ADJUSTMENT_PITCH_GYRO_CUTOFF(int value)
{
    currentPidProfile->gyro_cutoff[PID_PITCH] = value;
    filterUpdate(&pid.gyrorFilter[PID_PITCH], value, pid.freq);
}

int get_ADJUSTMENT_ROLL_GYRO_CUTOFF(void)
{
    return currentPidProfile->gyro_cutoff[PID_ROLL];
}

void set_ADJUSTMENT_ROLL_GYRO_CUTOFF(int value)
{
    currentPidProfile->gyro_cutoff[PID_ROLL] = value;
    filterUpdate(&pid.gyrorFilter[PID_ROLL], value, pid.freq);
}

int get_ADJUSTMENT_YAW_GYRO_CUTOFF(void)
{
    return currentPidProfile->gyro_cutoff[PID_YAW];
}

void set_ADJUSTMENT_YAW_GYRO_CUTOFF(int value)
{
    currentPidProfile->gyro_cutoff[PID_YAW] = value;
    filterUpdate(&pid.gyrorFilter[PID_YAW], value, pid.freq);
}

int get_ADJUSTMENT_PITCH_DTERM_CUTOFF(void)
{
    return currentPidProfile->dterm_cutoff[PID_PITCH];
}

void set_ADJUSTMENT_PITCH_DTERM_CUTOFF(int value)
{
    currentPidProfile->dterm_cutoff[PID_PITCH] = value;
    difFilterUpdate(&pid.dtermFilter[PID_PITCH], value, pid.freq);
}

int get_ADJUSTMENT_ROLL_DTERM_CUTOFF(void)
{
    return currentPidProfile->dterm_cutoff[PID_ROLL];
}

void set_ADJUSTMENT_ROLL_DTERM_CUTOFF(int value)
{
    currentPidProfile->dterm_cutoff[PID_ROLL] = value;
    difFilterUpdate(&pid.dtermFilter[PID_ROLL], value, pid.freq);
}

int get_ADJUSTMENT_YAW_DTERM_CUTOFF(void)
{
    return currentPidProfile->dterm_cutoff[PID_YAW];
}

void set_ADJUSTMENT_YAW_DTERM_CUTOFF(int value)
{
    currentPidProfile->dterm_cutoff[PID_YAW] = value;
    difFilterUpdate(&pid.dtermFilter[PID_YAW], value, pid.freq);
}


//// Internal functions

static void INIT_CODE pidSetLooptime(uint32_t pidLooptime)
{
    pid.dT = pidLooptime * 1e-6f;
    pid.freq = 1.0f / pid.dT;

#ifdef USE_DSHOT
    dshotSetPidLoopTime(pidLooptime);
#endif
}

static void INIT_CODE pidInitFilters(const pidProfile_t *pidProfile)
{
    // PID Filters
    for (int i = 0; i < XYZ_AXIS_COUNT; i++) {
        lowpassFilterInit(&pid.gyrorFilter[i], LPF_1ST_ORDER, pidProfile->gyro_cutoff[i], pid.freq, LPF_UPDATE);
        difFilterInit(&pid.dtermFilter[i], pidProfile->dterm_cutoff[i], pid.freq);
        difFilterInit(&pid.btermFilter[i], pidProfile->bterm_cutoff[i], pid.freq);
        pt1FilterInit(&pid.relaxFilter[i], 1, pid.freq);
    }
}

void INIT_CODE pidLoadProfile(const pidProfile_t *pidProfile)
{
    // PID not initialised yet
    if (pid.dT == 0)
      return;

    // PID algorithm
    pid.pidMode = pidProfile->pid_mode;

    // Live per-axis P/I/D scale - applied at the point of use
    // (pidApplyMode0/1), not baked into pid.coef[], so it stays correct
    // regardless of which gain adjustment (including this one) last touched
    // the coefficients.
    for (int i = 0; i < PID_AXIS_COUNT; i++)
        pid.masterGain[i] = constrain(pidProfile->master_gain[i], MASTER_GAIN_MIN, MASTER_GAIN_MAX) * 0.01f;

    // Optional per-axis curve that further scales master gain by |stick deflection|
    for (int i = 0; i < PID_AXIS_COUNT; i++)
        pid.gainCurveIndex[i] = pidProfile->gain_curve[i];

    // Throttle-based gain attenuation: baseline gain plus an optional
    // shaping curve, mirroring master_gain + gain_curve
    pid.fwTpaGain = pidProfile->fw_tpa_gain * 0.01f;
    pid.fwTpaCurveIndex = pidProfile->fw_tpa_curve;

    // Speed attenuation, RPM or GPS speed (separate per-profile storage, see fwSpaConfig_t)
    speedAttenInit(fwSpaConfigs(getCurrentPidProfileIndex()), pid.freq);

    // Roll axis
    pid.coef[PID_ROLL].Kp = ROLL_P_TERM_SCALE * pidProfile->pid[PID_ROLL].P;
    pid.coef[PID_ROLL].Ki = ROLL_I_TERM_SCALE * pidProfile->pid[PID_ROLL].I;
    pid.coef[PID_ROLL].Kd = ROLL_D_TERM_SCALE * pidProfile->pid[PID_ROLL].D;
    pid.coef[PID_ROLL].Kf = ROLL_F_TERM_SCALE * pidProfile->pid[PID_ROLL].F;
    pid.coef[PID_ROLL].Kb = ROLL_B_TERM_SCALE * pidProfile->pid[PID_ROLL].B;

    // Pitch axis
    pid.coef[PID_PITCH].Kp = PITCH_P_TERM_SCALE * pidProfile->pid[PID_PITCH].P;
    pid.coef[PID_PITCH].Ki = PITCH_I_TERM_SCALE * pidProfile->pid[PID_PITCH].I;
    pid.coef[PID_PITCH].Kd = PITCH_D_TERM_SCALE * pidProfile->pid[PID_PITCH].D;
    pid.coef[PID_PITCH].Kf = PITCH_F_TERM_SCALE * pidProfile->pid[PID_PITCH].F;
    pid.coef[PID_PITCH].Kb = PITCH_B_TERM_SCALE * pidProfile->pid[PID_PITCH].B;

    // Yaw axis
    pid.coef[PID_YAW].Kp = YAW_P_TERM_SCALE * pidProfile->pid[PID_YAW].P;
    pid.coef[PID_YAW].Ki = YAW_I_TERM_SCALE * pidProfile->pid[PID_YAW].I;
    pid.coef[PID_YAW].Kd = YAW_D_TERM_SCALE * pidProfile->pid[PID_YAW].D;
    pid.coef[PID_YAW].Kf = YAW_F_TERM_SCALE * pidProfile->pid[PID_YAW].F;
    pid.coef[PID_YAW].Kb = YAW_B_TERM_SCALE * pidProfile->pid[PID_YAW].B;

    // Accumulated error limit
    for (int i = 0; i < XYZ_AXIS_COUNT; i++)
        pid.errorLimit[i] = pidProfile->error_limit[i];

    // Per-axis exponential I-term decay rate
    for (int i = 0; i < PID_AXIS_COUNT; i++)
        pid.itermDecayRate[i] = pidItermDecayRate(pidProfile->iterm_decay_time[i]);

    // Max I-term decay speed in degs/s (linear decay)
    pid.itermDecayLimit = (pidProfile->iterm_decay_limit) ? pidProfile->iterm_decay_limit : 3600;

    // Filters
    for (int i = 0; i < XYZ_AXIS_COUNT; i++) {
        filterUpdate(&pid.gyrorFilter[i], pidProfile->gyro_cutoff[i], pid.freq);
        difFilterUpdate(&pid.dtermFilter[i], pidProfile->dterm_cutoff[i], pid.freq);
        difFilterUpdate(&pid.btermFilter[i], pidProfile->bterm_cutoff[i], pid.freq);
    }

    // Error relax -- always on, there is no type or off switch
    for (int i = 0; i < XYZ_AXIS_COUNT; i++) {
        pt1FilterUpdate(&pid.relaxFilter[i], pidItermRelaxCutoff(pidProfile->iterm_relax[i]), pid.freq);
        pid.itermRelaxLevel[i] = constrain(pidProfile->iterm_relax_level[i], 10, 250);
    }
}

void INIT_CODE pidChangeProfile(const pidProfile_t *pidProfile)
{
    pidLoadProfile(pidProfile);
    //pidResetAxisErrors();
}

void INIT_CODE pidInit(const pidProfile_t *pidProfile)
{
    pidReset();
    driveReset();
    pidYawDirection = 1;
    speedAttenReset();
    pidSetLooptime(gyro.targetLooptime);
    pidInitFilters(pidProfile);
    pidChangeProfile(pidProfile);
}

void INIT_CODE pidCopyProfile(uint8_t dstPidProfileIndex, uint8_t srcPidProfileIndex)
{
    if (dstPidProfileIndex < PID_PROFILE_COUNT && srcPidProfileIndex < PID_PROFILE_COUNT &&
        dstPidProfileIndex != srcPidProfileIndex) {
        memcpy(pidProfilesMutable(dstPidProfileIndex), pidProfilesMutable(srcPidProfileIndex), sizeof(pidProfile_t));
        memcpy(fwSpaConfigsMutable(dstPidProfileIndex), fwSpaConfigs(srcPidProfileIndex), sizeof(fwSpaConfig_t));
    }
}


static float applyItermRelax(int axis, float itermError, float gyroRate, float setpoint)
{
    const float setpointLpf = pt1FilterApply(&pid.relaxFilter[axis], setpoint);
    const float setpointHpf = setpoint - setpointLpf;

    const float itermRelaxFactor = MAX(0, 1.0f - fabsf(setpointHpf) / pid.itermRelaxLevel[axis]);

    itermError *= itermRelaxFactor;

    DEBUG_AXIS(ITERM_RELAX, axis, 0, setpoint * 1000);
    DEBUG_AXIS(ITERM_RELAX, axis, 1, gyroRate * 1000);
    DEBUG_AXIS(ITERM_RELAX, axis, 2, setpointLpf * 1000);
    DEBUG_AXIS(ITERM_RELAX, axis, 3, setpointHpf * 1000);
    DEBUG_AXIS(ITERM_RELAX, axis, 4, itermRelaxFactor * 1000);
    DEBUG_AXIS(ITERM_RELAX, axis, 5, itermError * 1000);

    return itermError;
}

static float pidApplySetpoint(uint8_t axis)
{
    // Rate setpoint
    const float setpoint = getSetpoint(axis);

    // Save setpoint
    pid.data[axis].setPoint = setpoint;

    return setpoint;
}

static float pidApplyGyroRate(uint8_t axis)
{
    // Get gyro rate
    float gyroRate = gyro.gyroADCf[axis];

    if (axis == PID_YAW) {
        // yaw_gyro_reverse turns the steering loop's sense of yaw around, for when the gyro
        // corrects the wrong way (typically a transmitter with a reversed steering channel,
        // fixed with a servo reverse). Only this loop sees it: the board orientation and the
        // attitude estimate stay as they are.
        if (pidConfig()->yaw_gyro_reverse) {
            gyroRate = -gyroRate;
        }

        // Travelling backwards the yaw response to a steering angle changes sign. The stick
        // keeps meaning "wheel angle" (F and B are untouched), so correcting the right way
        // in reverse is exactly negating the gyro rate the loop sees: P, I and D all follow.
        // See flight/drive.c.
        gyroRate *= driveGetDirection();
    }

    // Bandwidth limiter
    gyroRate = filterApply(&pid.gyrorFilter[axis], gyroRate);

    // Save current rate
    pid.data[axis].gyroRate = gyroRate;

    return gyroRate;
}

/** ** ** ** ** ** ** ** ** ** ** ** ** ** ** ** ** ** ** ** ** ** ** ** ** ** ** ** ** ** **
 **
 ** MODE 0 - PASSTHROUGH
 **
 ** ** ** ** ** ** ** ** ** ** ** ** ** ** ** ** ** ** ** ** ** ** ** ** ** ** ** ** ** ** **/

static void pidApplyMode0(uint8_t axis)
{
    // Rate setpoint
    float setpoint = pidApplySetpoint(axis);

  //// Unused term
    pid.data[axis].P = 0;
    pid.data[axis].I = 0;
    pid.data[axis].D = 0;

  //// F-term

    // Calculate feedforward component
    pid.data[axis].F = pid.coef[axis].Kf * setpoint;

  //// PID Sum

    // Calculate PID sum
    pid.data[axis].pidSum = pid.data[axis].F;
}


/** ** ** ** ** ** ** ** ** ** ** ** ** ** ** ** ** ** ** ** ** ** ** ** ** ** ** ** ** ** **
 **
 ** MODE 1 - RATE PID
 **
 ** ** ** ** ** ** ** ** ** ** ** ** ** ** ** ** ** ** ** ** ** ** ** ** ** ** ** ** ** ** **/

// Linear interpolation through a curve's (ascending-x) points, evaluated on
// a 0..1 magnitude -- |stick deflection| for per-axis gain curves, |throttle|
// for fw_tpa_curve. Mirrors mixerEvaluateCurve()'s algorithm but the domain
// is unipolar since the caller always passes a magnitude.
static float pidEvaluateGainCurve(const gainCurve_t *curve, float mag)
{
    const int n = curve->count;

    if (n < 2)
        return 1.0f;

    const float xs = mag * 1000.0f;

    if (xs <= curve->points[0].x)
        return curve->points[0].y * 0.01f;

    if (xs >= curve->points[n - 1].x)
        return curve->points[n - 1].y * 0.01f;

    for (int i = 0; i < n - 1; i++) {
        const gainCurvePoint_t *p0 = &curve->points[i];
        const gainCurvePoint_t *p1 = &curve->points[i + 1];

        if (xs >= p0->x && xs <= p1->x) {
            const float t = (p1->x != p0->x) ? (xs - p0->x) / (float)(p1->x - p0->x) : 0;
            return (p0->y + t * (p1->y - p0->y)) * 0.01f;
        }
    }

    return 1.0f;
}

// Gain curve position (0..1) for an axis: its |stick deflection|.
float pidGetGainCurvePosition(uint8_t axis)
{
    return fminf(1.0f, fabsf(getRcDeflection(axis)));
}

// Scale from a gain curve (1..GAIN_CURVE_COUNT, 0 = none) at this axis's
// |stick deflection|. Out-of-range indices (MSP and CLI array writes are
// unchecked) count as no curve.
float pidGetGainCurveScale(uint8_t curveIndex, uint8_t axis)
{
    return pidGetGainCurveScaleAt(curveIndex, pidGetGainCurvePosition(axis));
}

// Same, at a caller-supplied 0..1 position (speed attenuation's speed / speed_max).
float pidGetGainCurveScaleAt(uint8_t curveIndex, float position)
{
    return (curveIndex > 0 && curveIndex <= GAIN_CURVE_COUNT)
        ? pidEvaluateGainCurve(gainCurves(curveIndex - 1), position)
        : 1.0f;
}

static float pidAxisGainCurve(uint8_t axis)
{
    return pidGetGainCurveScale(pid.gainCurveIndex[axis], axis);
}

static float pidThrottleAttenuation(void)
{
    // Mirrors masterGain + gain_curve: fwTpaGain is the baseline scale, an
    // optional curve from the same shared pool further shapes it by |throttle|
    // (0..1) instead of |stick deflection|. Throttle is signed (brake/reverse
    // below neutral), so the curve sees its magnitude. Applied (with speed attenuation)
    // to P, D, F and B through pidSurfaceAttenuation(); not to I.
    return pid.fwTpaGain * pidGetGainCurveScaleAt(pid.fwTpaCurveIndex, fminf(fabsf(getThrottle()), 1.0f));
}

// Throttle and speed attenuation together, as applied to P, D, F and B.
// Floored so a curve point near zero, or both baselines at their minimum, can
// never leave the steering without throw: F carries most of the stick authority,
// so a zero here would leave the stick almost nothing to move.
static float pidSurfaceAttenuation(void)
{
    return fmaxf(pidThrottleAttenuation() * speedAttenGetScale(), PID_ATTENUATION_MIN);
}

static uint32_t pidScaleToCentiPercent(float scale)
{
    return lrintf(fmaxf(0.0f, scale) * 10000.0f);
}

static uint32_t pidGainToCenti(float gain)
{
    return lrintf(fmaxf(0.0f, gain) * 100.0f);
}

void pidGetRuntimeGains(pidRuntimeGains_t *runtimeGains)
{
    memset(runtimeGains, 0, sizeof(*runtimeGains));

    const float fwTpa = pidThrottleAttenuation();
    const float fwSpa = speedAttenGetScale();
    const float atten = pidSurfaceAttenuation();
    runtimeGains->fwTpa = pidScaleToCentiPercent(fwTpa);
    runtimeGains->fwSpa = pidScaleToCentiPercent(fwSpa);
    runtimeGains->fwSpaSpeed = lrintf(speedAttenGetSpeed() * 10.0f);
    runtimeGains->fwSpaEnabled = speedAttenIsEnabled();

    for (int axis = 0; axis < PID_AXIS_COUNT; axis++) {
        const pidf_t *raw = &currentPidProfile->pid[axis];
        const float gainCurve = pidAxisGainCurve(axis);
        const float masterGain = pid.masterGain[axis] * gainCurve;

        runtimeGains->raw[axis] = *raw;
        runtimeGains->masterGain[axis] = currentPidProfile->master_gain[axis];
        runtimeGains->gainCurve[axis] = pidScaleToCentiPercent(gainCurve);
        runtimeGains->gainCurvePosition[axis] = pidScaleToCentiPercent(pidGetGainCurvePosition(axis));

        runtimeGains->effective[axis].P = pidGainToCenti(raw->P * masterGain * atten);
        runtimeGains->effective[axis].I = pidGainToCenti(raw->I * masterGain);
        runtimeGains->effective[axis].D = pidGainToCenti(raw->D * masterGain * atten);
        runtimeGains->effective[axis].F = pidGainToCenti(raw->F * atten);
        runtimeGains->effective[axis].B = pidGainToCenti(raw->B * atten);
    }
}

static void pidApplyMode1(uint8_t axis)
{
    // Rate setpoint
    const float setpoint = pidApplySetpoint(axis);

    // Get gyro rate
    const float gyroRate = pidApplyGyroRate(axis);

    // Calculate error rate
    const float errorRate = setpoint - gyroRate;

    // Throttle- and speed-based gain attenuation
    const float atten = pidSurfaceAttenuation();

    // Gyro correction scale from the drive state: zero at a standstill (no yaw response to
    // correct with, so P, I and D would only wind up), reduced or off in reverse, faded in
    // after a direction change. P, I and D only: F and B carry the stick, which must always
    // steer the wheels.
    const float corr = (axis == PID_YAW) ? driveGetCorrectionScale() : 1.0f;

    // Optional per-axis curve scaling master gain by |stick deflection|
    const float curveMult = pidAxisGainCurve(axis);
    const float masterGain = pid.masterGain[axis] * curveMult;


  //// P-term

    // Calculate P-component
    pid.data[axis].P = pid.coef[axis].Kp * masterGain * atten * corr * errorRate;


  //// D-term (gyro only)

    // Calculate D-term with bandwidth limit
    const float dTerm = difFilterApply(&pid.dtermFilter[axis], -gyroRate);

    // Calculate D-component
    pid.data[axis].D = pid.coef[axis].Kd * masterGain * atten * corr * dTerm;


  //// I-term

    // Apply error relax
    const float itermErrorRate = applyItermRelax(axis, errorRate, gyroRate, setpoint);

    // Saturation
    const bool saturation = (pidAxisSaturated(axis) && pid.data[axis].axisError * itermErrorRate > 0);

    // While GYRO OFF (PASSTHROUGH) drives the outputs the gyro is not tracking this setpoint, so
    // integrating the error only winds I up, and it was released as a bump on switching back.
    // Hold the accumulation; decay below still applies as normal.
    const bool bypassed = mixerStabilizationBypassed();

    // I-term change, held in proportion to the correction scale. Disarmed (on the bench) the
    // yaw I-term is held at zero: the car can't respond, and whatever built up would arrive
    // as a steering bump on arming.
    const bool benchHold = (axis == PID_YAW) && !ARMING_FLAG(ARMED);
    if (benchHold) {
        pid.data[axis].axisError = 0;
    }
    const float itermDelta = (saturation || bypassed || benchHold) ? 0 : itermErrorRate * pid.dT * corr;

    // Calculate I-component
    pid.data[axis].axisError = limitf(pid.data[axis].axisError + itermDelta, pid.errorLimit[axis]);
    pid.data[axis].I = pid.coef[axis].Ki * masterGain * corr * pid.data[axis].axisError;

    // Apply error decay
    const float errorDecay = limitf(pid.data[axis].axisError * pid.itermDecayRate[axis], pid.itermDecayLimit);
    pid.data[axis].axisError -= errorDecay * pid.dT;


  //// Feedforward

    // Calculate F component. Attenuated like P and D: F sets the steering deflection for a
    // commanded rate far more than the small P-term does.
    pid.data[axis].F = pid.coef[axis].Kf * atten * setpoint;


  //// Feedforward Boost (FF Derivative)

    // Calculate B-term with bandwidth limit
    const float bTerm = difFilterApply(&pid.btermFilter[axis], setpoint);

    // Calculate B-component
    pid.data[axis].B = pid.coef[axis].Kb * atten * bTerm;


  //// PID Sum

    // Calculate sum of all terms
    pid.data[axis].pidSum = pid.data[axis].P + pid.data[axis].I + pid.data[axis].D +
                            pid.data[axis].F + pid.data[axis].B;
}


// Only yaw (steering) is stabilised. Roll and pitch keep their slots in the per-axis arrays
// (MSP, Blackbox and the PID profile keep their layout) but always hold zero: no setpoint, no
// output, nothing for the mixer.
static void pidResetRollPitch(void)
{
    memset(&pid.data[PID_ROLL], 0, sizeof(pid.data[PID_ROLL]));
    memset(&pid.data[PID_PITCH], 0, sizeof(pid.data[PID_PITCH]));
}

void pidController(const pidProfile_t *pidProfile, timeUs_t currentTimeUs)
{
    UNUSED(pidProfile);
    UNUSED(currentTimeUs);

    // Drive state first: the speed attenuation and the steering loop both use it
    driveUpdate(pid.dT);
    speedAttenUpdate(pid.dT);

    // The yaw I-term was accumulated for the other travel direction: drop it on a change
    const int8_t yawDirection = driveGetDirection();
    if (yawDirection != pidYawDirection) {
        pidYawDirection = yawDirection;
        pidResetAxisError(PID_YAW);
    }

    DEBUG(GAIN_ATTEN, 0, lrintf(pidThrottleAttenuation() * 1000));
    DEBUG(GAIN_ATTEN, 1, lrintf(speedAttenGetScale() * 1000));
    DEBUG(GAIN_ATTEN, 2, lrintf(speedAttenGetSpeed() * 10));
#ifdef USE_GPS
    DEBUG(GAIN_ATTEN, 3, MAX(gpsSol.speed3d, gpsSol.groundSpeed));
    DEBUG(GAIN_ATTEN, 4, STATE(GPS_FIX) ? 1 : 0);
#endif

    pidResetRollPitch();

    // Only mode 0 (feedforward only, no gyro) and mode 1 exist; pid_mode accepts 0..9, so any
    // other value runs the stabilized mode 1 rather than silently dropping all gyro correction.
    switch (pid.pidMode) {
        case 0:
            pidApplyMode0(PID_YAW);
            break;
        default:
            pidApplyMode1(PID_YAW);
            break;
    }

    // Reset PID control if gyro overflow detected
    if (gyroOverflowDetected())
        pidReset();
}
