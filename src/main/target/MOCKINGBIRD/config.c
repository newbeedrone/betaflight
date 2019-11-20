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

#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "platform.h"

#ifdef USE_TARGET_CONFIG

#include "common/axis.h"
#include "common/maths.h"

#include "config/feature.h"

#include "drivers/barometer/barometer.h"
#include "drivers/light_led.h"
#include "drivers/pwm_esc_detect.h"
#include "drivers/pwm_output.h"

#include "fc/config.h"
#include "fc/controlrate_profile.h"
#include "fc/core.h"
#include "fc/rc_adjustments.h"
#include "fc/rc_modes.h"
#include "fc/rc_controls.h"

#include "flight/mixer.h"
#include "flight/pid.h"
#include "flight/imu.h"

#include "pg/vcd.h"
#include "pg/rx.h"
#include "pg/motor.h"
#include "pg/vtx_table.h"

#include "rx/rx.h"
#include "rx/cc2500_frsky_common.h"

#include "pg/rx.h"
#include "pg/rx_spi.h"
#include "pg/rx_spi_cc2500.h"
#include "pg/vcd.h"

#include "osd/osd.h"

#include "io/serial.h"
#include "io/vtx.h"
#include "io/ledstrip.h"

#include "sensors/battery.h"
#include "sensors/gyro.h"
#include "sensors/barometer.h"

#include "telemetry/telemetry.h"

#ifdef BRUSHED_MOTORS_PWM_RATE
#undef BRUSHED_MOTORS_PWM_RATE
#endif

#define BRUSHED_MOTORS_PWM_RATE 25000           // 25kHz

#if (defined(MOCKINGBIRD_DSM_US) || defined(MOCKINGBIRD_DSM_INTL))
#define BB_LITE_RSSI_CH_IDX     9
#endif

void targetConfiguration(void)
{
    if (getDetectedMotorType() == MOTOR_BRUSHED) {
        motorConfigMutable()->dev.motorPwmRate = BRUSHED_MOTORS_PWM_RATE;
        motorConfigMutable()->minthrottle = 1030;
        pidConfigMutable()->pid_process_denom = 1;
    }

    for (uint8_t pidProfileIndex = 0; pidProfileIndex < PID_PROFILE_COUNT; pidProfileIndex++) {
        pidProfile_t *pidProfile = pidProfilesMutable(pidProfileIndex);

        pidProfile->pid[PID_ROLL].P  = 84;
        pidProfile->pid[PID_ROLL].I  = 50;
        pidProfile->pid[PID_ROLL].D  = 58;
        pidProfile->pid[PID_PITCH].P = 87;
        pidProfile->pid[PID_PITCH].I = 55;
        pidProfile->pid[PID_PITCH].D = 58;
        pidProfile->pid[PID_YAW].P   = 110;
        pidProfile->pid[PID_YAW].I   = 75;
        pidProfile->pid[PID_YAW].D   = 25;
        pidProfile->dterm_notch_cutoff = 0;
    }

    for (uint8_t rateProfileIndex = 0; rateProfileIndex < CONTROL_RATE_PROFILE_COUNT; rateProfileIndex++) {
        controlRateConfig_t *controlRateConfig = controlRateProfilesMutable(rateProfileIndex);

        controlRateConfig->rcRates[FD_YAW] = 100;
        controlRateConfig->rcExpo[FD_ROLL] = 15;
        controlRateConfig->rcExpo[FD_PITCH] = 15;
        controlRateConfig->rcExpo[FD_YAW]  = 15;
        controlRateConfig->rates[FD_ROLL]  = 73;
        controlRateConfig->rates[FD_PITCH] = 73;
        controlRateConfig->rates[FD_YAW] = 73;
        controlRateConfig->dynThrPID = 55;
    }

    osdConfigMutable()->item_pos[OSD_CRAFT_NAME]        = OSD_POS(9, 10) | OSD_PROFILE_1_FLAG;
    osdConfigMutable()->item_pos[OSD_MAIN_BATT_VOLTAGE] = OSD_POS(23, 9) | OSD_PROFILE_1_FLAG;
    osdConfigMutable()->item_pos[OSD_ITEM_TIMER_2]      = OSD_POS(2,  9) | OSD_PROFILE_1_FLAG;
    osdConfigMutable()->item_pos[OSD_FLYMODE]           = OSD_POS(18, 9) | OSD_PROFILE_1_FLAG;
    osdConfigMutable()->item_pos[OSD_VTX_CHANNEL]       = OSD_POS(10, 9) | OSD_PROFILE_1_FLAG;
    osdConfigMutable()->item_pos[OSD_RSSI_VALUE]        = OSD_POS(2, 10) | OSD_PROFILE_1_FLAG;
    osdConfigMutable()->item_pos[OSD_WARNINGS]          = OSD_POS(9, 10);

    vtxSettingsConfigMutable()->band = 5;
    vtxSettingsConfigMutable()->channel = 1;
    vtxSettingsConfigMutable()->power = 2;
#if (defined(MOCKINGBIRD_DSM_US) || defined(MOCKINGBIRD_FRSKY_US) || defined(MOCKINGBIRD_SFHSS_US))
    uint16_t vtxTableFrequency[6][8] = {
        { 5865, 5845, 5825, 5805, 5785, 5765, 5745, 5725 }, // Boscam A
        { 5733, 5752, 5771, 5790, 5809, 5828, 5847, 5866 }, // Boscam B
        { 5705, 5685, 5665,    0, 5885, 5905,    0,    0 }, // Boscam E
        { 5740, 5760, 5780, 5800, 5820, 5840, 5860, 5880 }, // FatShark
        { 5658, 5695, 5732, 5769, 5806, 5843, 5880, 5917 }, // RaceBand
        { 5732, 5765, 5828, 5840, 5866, 5740,    0,    0 }, // IMD6
    };
#else 
    uint16_t vtxTableFrequency[6][8] = {
        { 5865, 5845, 5825, 5805, 5785, 5765, 5745, 5725 }, // Boscam A
        { 5733, 5752, 5771, 5790, 5809, 5828, 5847, 5866 }, // Boscam B
        { 5705, 5685, 5665, 5645, 5885, 5905, 5925, 5945 }, // Boscam E
        { 5740, 5760, 5780, 5800, 5820, 5840, 5860, 5880 }, // FatShark
        { 5658, 5695, 5732, 5769, 5806, 5843, 5880, 5917 }, // RaceBand
        { 5732, 5765, 5828, 5840, 5866, 5740,    0,    0 }, // IMD6
    };
#endif
    const char * vtxTableBandNames[6] = {
            "BOSCAM A",
            "BOSCAM B",
            "BOSCAM E",
            "FATSHARK",
            "RACEBAND",
            "IMD6"
    };
    char vtxTableBandLetters[7] = "ABEFRI";
    vtxTableConfigMutable()->bands = 6;
    vtxTableConfigMutable()->channels = 8;
    for (uint8_t i = 0; i < 6; i++) {
        for (uint8_t j = 0; j < 8; j++) {
            vtxTableConfigMutable()->frequency[i][j] = vtxTableFrequency[i][j];
        }
    }
    for (uint8_t i = 0; i < 6; i++) {
        strcpy(vtxTableConfigMutable()->bandNames[i], vtxTableBandNames[i]);
        vtxTableConfigMutable()->bandLetters[i] = vtxTableBandLetters[i];
    }
    strcpy(vtxTableConfigMutable()->channelNames[0], "1");
    strcpy(vtxTableConfigMutable()->channelNames[1], "2");
    strcpy(vtxTableConfigMutable()->channelNames[2], "3");
    strcpy(vtxTableConfigMutable()->channelNames[3], "4");
    strcpy(vtxTableConfigMutable()->channelNames[4], "5");
    strcpy(vtxTableConfigMutable()->channelNames[5], "6");
    strcpy(vtxTableConfigMutable()->channelNames[6], "7");
    strcpy(vtxTableConfigMutable()->channelNames[7], "8");
    vtxTableConfigMutable()->powerLevels = 3;
    vtxTableConfigMutable()->powerValues[0] = 0;
    vtxTableConfigMutable()->powerValues[1] = 1;
    vtxTableConfigMutable()->powerValues[2] = 2;
    strcpy(vtxTableConfigMutable()->powerLabels[0], "OFF");
    strcpy(vtxTableConfigMutable()->powerLabels[1], "MIN");
    strcpy(vtxTableConfigMutable()->powerLabels[2], "MAX");
    

    batteryConfigMutable()->batteryCapacity = 250;
    batteryConfigMutable()->vbatmincellvoltage = 28;
    batteryConfigMutable()->vbatwarningcellvoltage = 32;

    imuConfigMutable()->small_angle = 180;

    modeActivationConditionsMutable(0)->modeId           = BOXARM;
    modeActivationConditionsMutable(0)->auxChannelIndex  = AUX1 - NON_AUX_CHANNEL_COUNT;
    modeActivationConditionsMutable(0)->range.startStep  = CHANNEL_VALUE_TO_STEP(1700);
    modeActivationConditionsMutable(0)->range.endStep    = CHANNEL_VALUE_TO_STEP(2100);

    modeActivationConditionsMutable(1)->modeId           = BOXANGLE;
    modeActivationConditionsMutable(1)->auxChannelIndex  = AUX2 - NON_AUX_CHANNEL_COUNT;
    modeActivationConditionsMutable(1)->range.startStep  = CHANNEL_VALUE_TO_STEP(900);
    modeActivationConditionsMutable(1)->range.endStep    = CHANNEL_VALUE_TO_STEP(1300);

    modeActivationConditionsMutable(2)->modeId           = BOXHORIZON;
    modeActivationConditionsMutable(2)->auxChannelIndex  = AUX2 - NON_AUX_CHANNEL_COUNT;
    modeActivationConditionsMutable(2)->range.startStep  = CHANNEL_VALUE_TO_STEP(1300);
    modeActivationConditionsMutable(2)->range.endStep    = CHANNEL_VALUE_TO_STEP(1700);

    // modeActivationConditionsMutable(3)->modeId           = BOXAIRMODE;
    // modeActivationConditionsMutable(3)->auxChannelIndex  = AUX2 - NON_AUX_CHANNEL_COUNT;
    // modeActivationConditionsMutable(3)->range.startStep  = CHANNEL_VALUE_TO_STEP(1700);
    // modeActivationConditionsMutable(3)->range.endStep    = CHANNEL_VALUE_TO_STEP(2100);

    ledStripStatusModeConfigMutable()->ledConfigs[0] = DEFINE_LED(7, 7,  8, 0, LF(COLOR), LO(LARSON_SCANNER) | LO(THROTTLE), 0);
    ledStripStatusModeConfigMutable()->ledConfigs[1] = DEFINE_LED(8, 7, 13, 0, LF(COLOR), LO(LARSON_SCANNER) | LO(THROTTLE), 0);
    ledStripStatusModeConfigMutable()->ledConfigs[2] = DEFINE_LED(9, 7, 11, 0, LF(COLOR), LO(LARSON_SCANNER) | LO(THROTTLE), 0);

    strcpy(pilotConfigMutable()->name, "Mocking Bird");
    systemConfigMutable()->activeRateProfile = 2;
    systemConfigMutable()->pidProfileIndex = 2;

    pidConfigMutable()->runaway_takeoff_prevention = false;

    pidProfilesMutable(2)->dyn_lpf_dterm_min_hz = 105;
    pidProfilesMutable(2)->dyn_lpf_dterm_max_hz = 255;
    pidProfilesMutable(2)->dterm_lowpass2_hz = 225;
    pidProfilesMutable(2)->feedForwardTransition = 25;
    pidProfilesMutable(2)->vbatPidCompensation = true;
    pidProfilesMutable(2)->iterm_relax = ITERM_RELAX_RP;
    pidProfilesMutable(2)->iterm_rotation = true;
    pidProfilesMutable(2)->pidSumLimit = 500;
    pidProfilesMutable(2)->pidSumLimitYaw = 400;
    pidProfilesMutable(2)->pid[PID_PITCH].P = 85;
    pidProfilesMutable(2)->pid[PID_PITCH].I = 80;
    pidProfilesMutable(2)->pid[PID_PITCH].D = 90;
    pidProfilesMutable(2)->pid[PID_PITCH].F = 100;
    pidProfilesMutable(2)->pid[PID_ROLL].P  = 85;
    pidProfilesMutable(2)->pid[PID_ROLL].I  = 80;
    pidProfilesMutable(2)->pid[PID_ROLL].D  = 90;
    pidProfilesMutable(2)->pid[PID_ROLL].F  = 100;
    pidProfilesMutable(2)->pid[PID_YAW].P   = 100;
    pidProfilesMutable(2)->pid[PID_YAW].I   = 80;
    pidProfilesMutable(2)->pid[PID_YAW].D   = 0;
    pidProfilesMutable(2)->pid[PID_YAW].F   = 75;
    pidProfilesMutable(2)->pid[PID_LEVEL].P = 110;
    pidProfilesMutable(2)->pid[PID_LEVEL].I = 30;
    pidProfilesMutable(2)->pid[PID_LEVEL].D = 80;
    pidProfilesMutable(2)->levelAngleLimit  = 60;
    pidProfilesMutable(2)->horizon_tilt_effect = 80;
    pidProfilesMutable(2)->horizon_tilt_expert_mode = true;
    pidProfilesMutable(2)->abs_control_gain = 20;
    pidProfilesMutable(2)->use_integrated_yaw = false;
    pidProfilesMutable(2)->d_min[FD_ROLL] = 28;
    pidProfilesMutable(2)->d_min[FD_PITCH] = 28;
    pidProfilesMutable(2)->d_min_gain = 30;
    pidProfilesMutable(2)->d_min_advance = 0;

    pidProfilesMutable(1)->vbatPidCompensation = 1;
    pidProfilesMutable(1)->iterm_relax = ITERM_RELAX_RP;
    pidProfilesMutable(1)->vbatPidCompensation = false;
    pidProfilesMutable(1)->pid[PID_PITCH].P = 71;
    pidProfilesMutable(1)->pid[PID_PITCH].I = 45;
    pidProfilesMutable(1)->pid[PID_PITCH].D = 41;
    pidProfilesMutable(1)->pid[PID_PITCH].F = 20;
    pidProfilesMutable(1)->pid[PID_ROLL].P  = 62;
    pidProfilesMutable(1)->pid[PID_ROLL].I  = 40;
    pidProfilesMutable(1)->pid[PID_ROLL].D  = 44;
    pidProfilesMutable(1)->pid[PID_ROLL].F  = 20;
    pidProfilesMutable(1)->pid[PID_YAW].P   = 80;
    pidProfilesMutable(1)->pid[PID_YAW].I   = 65;
    pidProfilesMutable(1)->pid[PID_YAW].D   = 17;
    pidProfilesMutable(1)->pid[PID_YAW].F   = 0;
    pidProfilesMutable(1)->pid[PID_LEVEL].P = 100;
    pidProfilesMutable(1)->pid[PID_LEVEL].I = 30;
    pidProfilesMutable(1)->pid[PID_LEVEL].D = 80;
    pidProfilesMutable(1)->levelAngleLimit  = 60;
    pidProfilesMutable(1)->horizon_tilt_effect = 80;
    pidProfilesMutable(1)->horizon_tilt_expert_mode = true;
    pidProfilesMutable(1)->d_min[FD_ROLL] = 16;
    pidProfilesMutable(1)->d_min[FD_PITCH] = 18;
    pidProfilesMutable(1)->d_min_gain = 30;
    pidProfilesMutable(1)->d_min_advance = 0;

    pidProfilesMutable(0)->dterm_lowpass2_hz = 0;
    pidProfilesMutable(0)->dterm_filter_type = FILTER_PT1;
    pidProfilesMutable(0)->vbatPidCompensation = true;
    pidProfilesMutable(0)->iterm_relax = ITERM_RELAX_RP;
    pidProfilesMutable(0)->pidSumLimit = 1000;
    pidProfilesMutable(0)->pidSumLimitYaw = 1000;
    pidProfilesMutable(0)->pid[PID_PITCH].P = 50;
    pidProfilesMutable(0)->pid[PID_PITCH].I = 40;
    pidProfilesMutable(0)->pid[PID_PITCH].D = 35;
    pidProfilesMutable(0)->pid[PID_PITCH].F = 140;
    pidProfilesMutable(0)->pid[PID_ROLL].P  = 50;
    pidProfilesMutable(0)->pid[PID_ROLL].I  = 40;
    pidProfilesMutable(0)->pid[PID_ROLL].D  = 35;
    pidProfilesMutable(0)->pid[PID_ROLL].F  = 140;
    pidProfilesMutable(0)->pid[PID_YAW].P   = 50;
    pidProfilesMutable(0)->pid[PID_YAW].I   = 40;
    pidProfilesMutable(0)->pid[PID_YAW].D   = 35;
    pidProfilesMutable(0)->pid[PID_YAW].F   = 0;
    pidProfilesMutable(0)->pid[PID_LEVEL].P = 50;
    pidProfilesMutable(0)->pid[PID_LEVEL].I = 50;
    pidProfilesMutable(0)->pid[PID_LEVEL].D = 75;
    pidProfilesMutable(0)->levelAngleLimit  = 60;
    pidProfilesMutable(0)->horizon_tilt_effect = 75;
    pidProfilesMutable(0)->horizon_tilt_expert_mode = false;
    pidProfilesMutable(0)->d_min[FD_ROLL] = 16;
    pidProfilesMutable(0)->d_min[FD_PITCH] = 18;
    pidProfilesMutable(0)->d_min_gain = 30;
    pidProfilesMutable(0)->d_min_advance = 0;

    controlRateProfilesMutable(0)->rcRates[FD_YAW] = 210;
    controlRateProfilesMutable(0)->rcExpo[FD_ROLL] = 0;
    controlRateProfilesMutable(0)->rcExpo[FD_PITCH] = 0;
    controlRateProfilesMutable(0)->rcExpo[FD_YAW]  = 0;
    controlRateProfilesMutable(0)->rates[FD_PITCH] = 80;
    controlRateProfilesMutable(0)->rates[FD_ROLL] = 80;
    controlRateProfilesMutable(0)->rates[FD_YAW] = 20;
    controlRateProfilesMutable(0)->dynThrPID = 0;
    controlRateProfilesMutable(0)->tpa_breakpoint = 2000;
    controlRateProfilesMutable(0)->throttle_limit_type = THROTTLE_LIMIT_TYPE_CLIP;

    controlRateProfilesMutable(1)->rcRates[FD_YAW] = 210;
    controlRateProfilesMutable(1)->rcExpo[FD_ROLL] = 0;
    controlRateProfilesMutable(1)->rcExpo[FD_PITCH] = 0;
    controlRateProfilesMutable(1)->rcExpo[FD_YAW]  = 0;
    controlRateProfilesMutable(1)->rates[FD_PITCH] = 80;
    controlRateProfilesMutable(1)->rates[FD_ROLL] = 80;
    controlRateProfilesMutable(1)->rates[FD_YAW] = 25;
    controlRateProfilesMutable(1)->dynThrPID = 0;
    controlRateProfilesMutable(1)->tpa_breakpoint = 2000;
    controlRateProfilesMutable(1)->throttle_limit_type = THROTTLE_LIMIT_TYPE_CLIP;

    controlRateProfilesMutable(2)->rcRates[FD_ROLL] = 1;
    controlRateProfilesMutable(2)->rcRates[FD_PITCH] = 1;
    controlRateProfilesMutable(2)->rcRates[FD_YAW] = 210;
    controlRateProfilesMutable(2)->rcExpo[FD_ROLL] = 0;
    controlRateProfilesMutable(2)->rcExpo[FD_PITCH] = 0;
    controlRateProfilesMutable(2)->rcExpo[FD_YAW]  = 0;
    controlRateProfilesMutable(2)->rates[FD_ROLL] = 0;
    controlRateProfilesMutable(2)->rates[FD_PITCH] = 0;
    controlRateProfilesMutable(2)->rates[FD_YAW] = 0;
    controlRateProfilesMutable(2)->dynThrPID = 0;
    controlRateProfilesMutable(2)->tpa_breakpoint = 2000;
    controlRateProfilesMutable(2)->throttle_limit_type = THROTTLE_LIMIT_TYPE_CLIP;

    osdConfigMutable()->item_pos[OSD_CRAFT_NAME]        = OSD_POS(9, 10) | OSD_PROFILE_1_FLAG;
    osdConfigMutable()->item_pos[OSD_MAIN_BATT_VOLTAGE] = OSD_POS(23, 9) | OSD_PROFILE_1_FLAG;
    osdConfigMutable()->item_pos[OSD_ITEM_TIMER_2]      = OSD_POS(2,  9) | OSD_PROFILE_1_FLAG;
    osdConfigMutable()->item_pos[OSD_FLYMODE]           = OSD_POS(17, 9) | OSD_PROFILE_1_FLAG;
    osdConfigMutable()->item_pos[OSD_VTX_CHANNEL]       = OSD_POS(10, 9) | OSD_PROFILE_1_FLAG;
    osdConfigMutable()->item_pos[OSD_RSSI_VALUE]        = OSD_POS(2, 10) | OSD_PROFILE_1_FLAG;
    osdConfigMutable()->item_pos[OSD_WARNINGS]          = OSD_POS(9, 10);

    vtxSettingsConfigMutable()->band = 5;
    vtxSettingsConfigMutable()->channel = 1;
    vtxSettingsConfigMutable()->power = 2;

    batteryConfigMutable()->batteryCapacity = 300;
    batteryConfigMutable()->vbatmincellvoltage = 290;
    batteryConfigMutable()->vbatwarningcellvoltage = 320;
    batteryConfigMutable()->vbatmaxcellvoltage = 450;
    batteryConfigMutable()->vbatwarningcellvoltage = 300;

    imuConfigMutable()->small_angle = 180;

    // modeActivationConditionsMutable(0)->modeId           = BOXARM;
    // modeActivationConditionsMutable(0)->auxChannelIndex  = AUX1 - NON_AUX_CHANNEL_COUNT;
    // modeActivationConditionsMutable(0)->range.startStep  = CHANNEL_VALUE_TO_STEP(1700);
    // modeActivationConditionsMutable(0)->range.endStep    = CHANNEL_VALUE_TO_STEP(2100);

    modeActivationConditionsMutable(1)->modeId           = BOXANGLE;
    modeActivationConditionsMutable(1)->auxChannelIndex  = AUX2 - NON_AUX_CHANNEL_COUNT;
    modeActivationConditionsMutable(1)->range.startStep  = CHANNEL_VALUE_TO_STEP(1825);
    modeActivationConditionsMutable(1)->range.endStep    = CHANNEL_VALUE_TO_STEP(2100);

    // modeActivationConditionsMutable(2)->modeId           = BOXHORIZON;
    // modeActivationConditionsMutable(2)->auxChannelIndex  = AUX2 - NON_AUX_CHANNEL_COUNT;
    // modeActivationConditionsMutable(2)->range.startStep  = CHANNEL_VALUE_TO_STEP(1300);
    // modeActivationConditionsMutable(2)->range.endStep    = CHANNEL_VALUE_TO_STEP(1700);

    modeActivationConditionsMutable(3)->modeId           = BOXAIRMODE;
    modeActivationConditionsMutable(3)->auxChannelIndex  = AUX2 - NON_AUX_CHANNEL_COUNT;
    modeActivationConditionsMutable(3)->range.startStep  = CHANNEL_VALUE_TO_STEP(900);
    modeActivationConditionsMutable(3)->range.endStep    = CHANNEL_VALUE_TO_STEP(1550);

    modeActivationConditionsMutable(4)->modeId           = BOXFPVANGLEMIX;
    modeActivationConditionsMutable(4)->auxChannelIndex  = AUX2 - NON_AUX_CHANNEL_COUNT;
    modeActivationConditionsMutable(4)->range.startStep  = CHANNEL_VALUE_TO_STEP(1450);
    modeActivationConditionsMutable(4)->range.endStep    = CHANNEL_VALUE_TO_STEP(2100);

    ledStripStatusModeConfigMutable()->ledConfigs[0] = DEFINE_LED(5, 7,  1, LD(WEST), LF(COLOR), 0, 0);
    ledStripStatusModeConfigMutable()->ledConfigs[1] = DEFINE_LED(6, 7,  2, LD(EAST), LF(COLOR), LO(LARSON_SCANNER) | LO(THROTTLE) | LO(INDICATOR), 0);
    ledStripStatusModeConfigMutable()->ledConfigs[2] = DEFINE_LED(7, 7, 10,        0, LF(COLOR), LO(LARSON_SCANNER) | LO(THROTTLE) | LO(INDICATOR), 0);

    strcpy(pilotConfigMutable()->name, "MKNGBRD");

    adjustmentRangesMutable(0) -> auxSwitchChannelIndex = AUX2 - NON_AUX_CHANNEL_COUNT;
    adjustmentRangesMutable(0) -> auxChannelIndex       = AUX2 - NON_AUX_CHANNEL_COUNT;
    adjustmentRangesMutable(0) -> range.startStep       = CHANNEL_VALUE_TO_STEP(900);
    adjustmentRangesMutable(0) -> range.endStep         = CHANNEL_VALUE_TO_STEP(2100);
    adjustmentRangesMutable(0) -> adjustmentConfig    = ADJUSTMENT_RATE_PROFILE;
    adjustmentRangesMutable(0) -> auxSwitchChannelIndex = 1;

    gyroConfigMutable()->gyro_lowpass2_hz = 375;
    gyroConfigMutable()->dyn_notch_range = DYN_NOTCH_RANGE_HIGH;
    gyroConfigMutable()->dyn_notch_width_percent = 0;
    gyroConfigMutable()->dyn_notch_q =  70;
    gyroConfigMutable()->dyn_notch_min_hz = 130;
    gyroConfigMutable()->dyn_lpf_gyro_min_hz = 300;
    gyroConfigMutable()->dyn_lpf_gyro_max_hz = 750;
    barometerConfigMutable()->baro_hardware = BARO_BMP280;
    rxConfigMutable()->mincheck = 1007;
    rxConfigMutable()->maxcheck = 1900;
    rxConfigMutable()->fpvCamAngleDegrees = 12;
    rxConfigMutable()->airModeActivateThreshold = 25;
    motorConfigMutable()->minthrottle = 1050;
    throttleCorrectionConfigMutable()->throttle_correction_value = 3;
    throttleCorrectionConfigMutable()->throttle_correction_angle = 600;
    mixerConfigMutable()->yaw_motors_reversed = true;
    mixerConfigMutable()->crashflip_motor_percent = 50;
    osdConfigMutable()->enabledWarnings = 0;
    osdConfigMutable()->cap_alarm = 250;
    osdConfigMutable()->item_pos[OSD_MAIN_BATT_VOLTAGE] = 2370;
    osdConfigMutable()->item_pos[OSD_RSSI_VALUE] = 2082;
    osdConfigMutable()->item_pos[OSD_ITEM_TIMER_2] = 2102;
    osdConfigMutable()->item_pos[OSD_FLYMODE] = 2392;
    osdConfigMutable()->item_pos[OSD_VTX_CHANNEL] = 2092;
    osdConfigMutable()->item_pos[OSD_CRAFT_NAME] = 2379;
    osdConfigMutable()->item_pos[OSD_WARNINGS] = 298;
    vcdProfileMutable()->video_system = VIDEO_SYSTEM_NTSC;

#if (defined(MOCKINGBIRD_DSM_US) || defined(MOCKINGBIRD_DSM_INTL))
    // DSM version
    rxConfigMutable()->rssi_channel = BB_LITE_RSSI_CH_IDX;
    rxFailsafeChannelConfig_t *channelFailsafeConfig = rxFailsafeChannelConfigsMutable(BB_LITE_RSSI_CH_IDX - 1);
    channelFailsafeConfig->mode = RX_FAILSAFE_MODE_SET;
    channelFailsafeConfig->step = CHANNEL_VALUE_TO_RXFAIL_STEP(1000);

    for (uint8_t rxRangeIndex = 0; rxRangeIndex < NON_AUX_CHANNEL_COUNT; rxRangeIndex++) {
        rxChannelRangeConfig_t *channelRangeConfig = rxChannelRangeConfigsMutable(rxRangeIndex);

        channelRangeConfig->min = 1160;
        channelRangeConfig->max = 1840;
    }
#else
    do {
        // T8SG
        uint8_t defaultTXHopTable[50] = {0,30,60,91,120,150,180,210,5,35,65,95,125,155,185,215,10,40,70,100,130,160,190,221,15,45,75,105,135,165,195,225,20,50,80,110,140,170,200,230,25,55,85,115,145,175,205,0,0,0};
        rxCc2500SpiConfigMutable()->bindOffset  = -42;
        rxCc2500SpiConfigMutable()->bindTxId[0] = 0;
        rxCc2500SpiConfigMutable()->bindTxId[1] = 191;
        for (uint8_t i = 0; i < 50; i++) {
            rxCc2500SpiConfigMutable()->bindHopData[i] = defaultTXHopTable[i];
        }
    } while (0);
#endif
}
#endif
