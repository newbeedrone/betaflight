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

void targetConfiguration(void)
{
    for (uint8_t pidProfileIndex = 0; pidProfileIndex < PID_PROFILE_COUNT; pidProfileIndex++) {
        pidProfile_t *pidProfile = pidProfilesMutable(pidProfileIndex);

        pidProfile->pid[PID_ROLL].P  = 86;
        pidProfile->pid[PID_ROLL].I  = 50;
        pidProfile->pid[PID_ROLL].D  = 60;
        pidProfile->pid[PID_PITCH].P = 90;
        pidProfile->pid[PID_PITCH].I = 55;
        pidProfile->pid[PID_PITCH].D = 60;
        pidProfile->pid[PID_YAW].P   = 120;
        pidProfile->pid[PID_YAW].I   = 75;
        pidProfile->pid[PID_YAW].D   = 20;
        pidProfile->dterm_notch_cutoff = 0;
    }

    for (uint8_t rateProfileIndex = 0; rateProfileIndex < CONTROL_RATE_PROFILE_COUNT; rateProfileIndex++) {
        controlRateConfig_t *controlRateConfig = controlRateProfilesMutable(rateProfileIndex);

        controlRateConfig->rcRates[FD_YAW] = 100;
        controlRateConfig->rcExpo[FD_ROLL] = 15;
        controlRateConfig->rcExpo[FD_PITCH] = 15;
        controlRateConfig->rcExpo[FD_YAW]  = 15;
        controlRateConfig->rates[FD_ROLL]  = 80;
        controlRateConfig->rates[FD_PITCH] = 80;
        controlRateConfig->rates[FD_YAW] = 80;
        controlRateConfig->dynThrPID = 50;
    }

    osdConfigMutable()->item_pos[OSD_CRAFT_NAME]        = OSD_POS(9, 10) | OSD_PROFILE_1_FLAG;
    osdConfigMutable()->item_pos[OSD_MAIN_BATT_VOLTAGE] = OSD_POS(23, 9) | OSD_PROFILE_1_FLAG;
    osdConfigMutable()->item_pos[OSD_ITEM_TIMER_2]      = OSD_POS(2,  9) | OSD_PROFILE_1_FLAG;
    osdConfigMutable()->item_pos[OSD_FLYMODE]           = OSD_POS(17, 9) | OSD_PROFILE_1_FLAG;
    osdConfigMutable()->item_pos[OSD_VTX_CHANNEL]       = OSD_POS(9,  9) | OSD_PROFILE_1_FLAG;
    osdConfigMutable()->item_pos[OSD_RSSI_VALUE]        = OSD_POS(2, 10) | OSD_PROFILE_1_FLAG;
    osdConfigMutable()->item_pos[OSD_WARNINGS]          = OSD_POS(9, 10);
    osdConfigMutable()->item_pos[OSD_CURRENT_DRAW]      = OSD_POS(22,10) | OSD_PROFILE_1_FLAG;

    vtxSettingsConfigMutable()->band = 5;
    vtxSettingsConfigMutable()->channel = 8;
    vtxSettingsConfigMutable()->power = 2;
#if defined(BEEBRAIN_BL_6000_SBUS_US) || defined (BEEBRAIN_BL_6000_DSM_US) || defined (BEEBRAIN_BL_6000_CRSF_US) || \
    defined(BEEBRAIN_BL_6500_SBUS_US) || defined (BEEBRAIN_BL_6500_DSM_US) || defined (BEEBRAIN_BL_6500_CRSF_US) 
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
    strcpy(vtxTableConfigMutable()->powerLabels[0], "5  ");
    strcpy(vtxTableConfigMutable()->powerLabels[1], "25 ");
    strcpy(vtxTableConfigMutable()->powerLabels[2], "100");
    

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

    modeActivationConditionsMutable(3)->modeId           = BOXFPVANGLEMIX;
    modeActivationConditionsMutable(3)->auxChannelIndex  = AUX2 - NON_AUX_CHANNEL_COUNT;
    modeActivationConditionsMutable(3)->range.startStep  = CHANNEL_VALUE_TO_STEP(900);
    modeActivationConditionsMutable(3)->range.endStep    = CHANNEL_VALUE_TO_STEP(1300);

    modeActivationConditionsMutable(4)->modeId           = BOXFLIPOVERAFTERCRASH;
    modeActivationConditionsMutable(4)->auxChannelIndex  = AUX3 - NON_AUX_CHANNEL_COUNT;
    modeActivationConditionsMutable(4)->range.startStep  = CHANNEL_VALUE_TO_STEP(900);
    modeActivationConditionsMutable(4)->range.endStep    = CHANNEL_VALUE_TO_STEP(1300);

    // ledStripConfigMutable()->ledConfigs[0] = DEFINE_LED(0, 0,  1, 0, LF(COLOR), 0, 0);
    // ledStripConfigMutable()->ledConfigs[1] = DEFINE_LED(1, 0, 10, 0, LF(COLOR), LO(LARSON_SCANNER), 0);
    // ledStripConfigMutable()->ledConfigs[2] = DEFINE_LED(2, 0,  2, 0, LF(COLOR), LO(LARSON_SCANNER), 0);

    strcpy(pilotConfigMutable()->name, "BeeBrain BL");

    // rxConfigMutable()->rssi_channel = BB_LITE_RSSI_CH_IDX;
    // rxFailsafeChannelConfig_t *channelFailsafeConfig = rxFailsafeChannelConfigsMutable(BB_LITE_RSSI_CH_IDX - 1);
    // channelFailsafeConfig->mode = RX_FAILSAFE_MODE_SET;
    // channelFailsafeConfig->step = CHANNEL_VALUE_TO_RXFAIL_STEP(1000);

    // for (uint8_t rxRangeIndex = 0; rxRangeIndex < NON_AUX_CHANNEL_COUNT; rxRangeIndex++) {
    //     rxChannelRangeConfig_t *channelRangeConfig = rxChannelRangeConfigsMutable(rxRangeIndex);

    //     channelRangeConfig->min = 1140;
    //     channelRangeConfig->max = 1857;
    // }

    gyroConfigMutable()->gyro_lowpass_type = FILTER_BIQUAD;
    gyroConfigMutable()->gyro_lowpass_hz = 150;
    gyroConfigMutable()->gyro_lowpass2_hz = 0;
    gyroConfigMutable()->yaw_spin_threshold = 1400;
    rxConfigMutable()->mincheck = 1010;
    rxConfigMutable()->maxcheck = 2000;
    rxConfigMutable()->rc_smoothing_type = RC_SMOOTHING_TYPE_FILTER;
    rxConfigMutable()->fpvCamAngleDegrees = 12;
    rxConfigMutable()->rssi_channel = 9;
    //motorConfigMutable()->digitalIdleOffsetValue = 1000;
    motorConfigMutable()->dev.useBurstDshot = true;
    motorConfigMutable()->dev.useDshotTelemetry = false;
    motorConfigMutable()->motorPoleCount = 12;
    motorConfigMutable()->dev.motorPwmProtocol = PWM_TYPE_DSHOT600;
    batteryConfigMutable()->batteryCapacity = 300;
    batteryConfigMutable()->vbatmaxcellvoltage = 440;
    batteryConfigMutable()->vbatfullcellvoltage = 400;
    batteryConfigMutable()->vbatmincellvoltage = 290;
    batteryConfigMutable()->vbatwarningcellvoltage = 320;
    voltageSensorADCConfigMutable(0)->vbatscale = 114;
    mixerConfigMutable()->yaw_motors_reversed = true;
    mixerConfigMutable()->crashflip_motor_percent = 50;
    imuConfigMutable()->small_angle = 180;
    pidConfigMutable()->pid_process_denom = 1;
    pidConfigMutable()->runaway_takeoff_prevention = false;
    osdConfigMutable()->enabledWarnings &= ~(1 << OSD_WARNING_CORE_TEMPERATURE);
    osdConfigMutable()->cap_alarm = 255;

    pidProfilesMutable(0)->dterm_filter_type = FILTER_BIQUAD;
    pidProfilesMutable(0)->dterm_lowpass_hz = 200;
    pidProfilesMutable(0)->dterm_lowpass2_hz = 0;
    pidProfilesMutable(0)->dterm_notch_cutoff = 0;
    pidProfilesMutable(0)->vbatPidCompensation = true;
    pidProfilesMutable(0)->itermThrottleThreshold = 200;
    pidProfilesMutable(0)->yawRateAccelLimit = 0;
    pidProfilesMutable(0)->iterm_relax = ITERM_RELAX_RP;
    pidProfilesMutable(0)->iterm_relax_type = ITERM_RELAX_SETPOINT;
    pidProfilesMutable(0)->pidSumLimit = 1000;
    pidProfilesMutable(0)->pidSumLimitYaw = 1000;
    pidProfilesMutable(0)->pid[PID_PITCH].P = 78;
    pidProfilesMutable(0)->pid[PID_PITCH].I = 75;
    pidProfilesMutable(0)->pid[PID_PITCH].D = 35;
    pidProfilesMutable(0)->pid[PID_PITCH].F = 155;
    pidProfilesMutable(0)->pid[PID_ROLL].P  = 75;
    pidProfilesMutable(0)->pid[PID_ROLL].I  = 70;
    pidProfilesMutable(0)->pid[PID_ROLL].D  = 30;
    pidProfilesMutable(0)->pid[PID_ROLL].F  = 155;
    pidProfilesMutable(0)->pid[PID_YAW].P   = 95;
    pidProfilesMutable(0)->pid[PID_YAW].I   = 70;
    pidProfilesMutable(0)->pid[PID_YAW].F   = 100;
    pidProfilesMutable(0)->pid[PID_LEVEL].P = 100;
    pidProfilesMutable(0)->pid[PID_LEVEL].I = 30;
    pidProfilesMutable(0)->pid[PID_LEVEL].D = 80;
    pidProfilesMutable(0)->levelAngleLimit  = 70;
    pidProfilesMutable(0)->horizon_tilt_effect = 80;
    pidProfilesMutable(0)->horizon_tilt_expert_mode = true;

    controlRateProfilesMutable(0)->rcRates[FD_YAW] = 207;
    controlRateProfilesMutable(0)->rates[FD_ROLL] = 80;
    controlRateProfilesMutable(0)->rates[FD_PITCH] = 80;
    controlRateProfilesMutable(0)->rates[FD_YAW] = 25;
    controlRateProfilesMutable(0)->rcExpo[FD_ROLL] = 0;
    controlRateProfilesMutable(0)->rcExpo[FD_PITCH] = 0;
    controlRateProfilesMutable(0)->rcExpo[FD_YAW]  = 0;
    controlRateProfilesMutable(0)->dynThrPID = 60;
    controlRateProfilesMutable(0)->tpa_breakpoint = 1750;

    ledStripStatusModeConfigMutable()->ledConfigs[0] = DEFINE_LED(7, 7,  8, 0, LF(COLOR), LO(LARSON_SCANNER) | LO(THROTTLE), 0);
    ledStripStatusModeConfigMutable()->ledConfigs[1] = DEFINE_LED(8, 7, 13, 0, LF(COLOR), LO(LARSON_SCANNER) | LO(THROTTLE), 0);
    ledStripStatusModeConfigMutable()->ledConfigs[2] = DEFINE_LED(9, 7, 11, 0, LF(COLOR), LO(LARSON_SCANNER) | LO(THROTTLE), 0);

#if defined(BEEBRAIN_BL_6500_SBUS_INTL) || defined (BEEBRAIN_BL_6500_DSM_INTL) || defined (BEEBRAIN_BL_6500_CRSF_INTL) || \
    defined(BEEBRAIN_BL_6500_SBUS_US)   || defined (BEEBRAIN_BL_6500_DSM_US)   || defined (BEEBRAIN_BL_6500_CRSF_US)
    gyroConfigMutable()->gyro_lowpass2_type = FILTER_BIQUAD;
    rxConfigMutable()->fpvCamAngleDegrees = 0;
    pidProfilesMutable(0)->dterm_filter2_type = FILTER_BIQUAD;
    pidProfilesMutable(0)->pid[PID_PITCH].P = 43;
    pidProfilesMutable(0)->pid[PID_PITCH].I = 36;
    pidProfilesMutable(0)->pid[PID_PITCH].D = 37;
    pidProfilesMutable(0)->pid[PID_PITCH].F = 20;
    pidProfilesMutable(0)->pid[PID_ROLL].P  = 47;
    pidProfilesMutable(0)->pid[PID_ROLL].I  = 39;
    pidProfilesMutable(0)->pid[PID_ROLL].D  = 37;
    pidProfilesMutable(0)->pid[PID_ROLL].F  = 20;
    pidProfilesMutable(0)->pid[PID_YAW].P   = 58;
    pidProfilesMutable(0)->pid[PID_YAW].I   = 63;
    pidProfilesMutable(0)->pid[PID_YAW].D   = 26;
    pidProfilesMutable(0)->pid[PID_YAW].F   = 0;
    pidProfilesMutable(0)->pid[PID_LEVEL].P = 50;
    pidProfilesMutable(0)->pid[PID_LEVEL].I = 50;
    pidProfilesMutable(0)->pid[PID_LEVEL].D = 75;
    pidProfilesMutable(0)->levelAngleLimit  = 90;

    controlRateProfilesMutable(0)->rcRates[FD_YAW] = 100;
    controlRateProfilesMutable(0)->rates[FD_ROLL] = 70;
    controlRateProfilesMutable(0)->rates[FD_PITCH] = 70;
    controlRateProfilesMutable(0)->rates[FD_YAW] = 70;
    // controlRateProfilesMutable(0)->rcExpo[FD_ROLL] = 0;
    // controlRateProfilesMutable(0)->rcExpo[FD_PITCH] = 0;
    // controlRateProfilesMutable(0)->rcExpo[FD_YAW]  = 0;
    controlRateProfilesMutable(0)->dynThrPID = 50;
    controlRateProfilesMutable(0)->tpa_breakpoint = 1500;
#else
    adjustmentRangesMutable(0)->auxChannelIndex = 1;
    adjustmentRangesMutable(0)->range.startStep = CHANNEL_VALUE_TO_STEP(1400);
    adjustmentRangesMutable(0)->range.endStep = CHANNEL_VALUE_TO_STEP(1600);
    adjustmentRangesMutable(0)->adjustmentConfig = 12;
    adjustmentRangesMutable(0)->auxSwitchChannelIndex = 1;
#endif

}
#endif
