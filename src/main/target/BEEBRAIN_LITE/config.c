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

#include "drivers/light_led.h"
#include "drivers/pwm_esc_detect.h"
#include "drivers/pwm_output.h"

#include "config/config.h"
#include "fc/controlrate_profile.h"
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

#include "osd/osd.h"

#include "io/serial.h"
#include "io/vtx.h"
#include "io/ledstrip.h"

#include "sensors/battery.h"
#include "sensors/gyro.h"

#include "telemetry/telemetry.h"

#ifdef BRUSHED_MOTORS_PWM_RATE
#undef BRUSHED_MOTORS_PWM_RATE
#endif

#define BRUSHED_MOTORS_PWM_RATE 25000 // 25kHz

#if (defined(BEEBRAIN_LITE_DSM_INTL) || defined(BEEBRAIN_LITE_DSM_US))
#define BB_LITE_RSSI_CH_IDX 9
#endif

void targetConfiguration(void)
{
    if (getDetectedMotorType() == MOTOR_BRUSHED)
    {
        motorConfigMutable()->dev.motorPwmRate = BRUSHED_MOTORS_PWM_RATE;
        motorConfigMutable()->minthrottle = 1030;
        pidConfigMutable()->pid_process_denom = 1;
    }
    // gold
    pidProfilesMutable(0)->pid[PID_ROLL].P = 66;
    pidProfilesMutable(0)->pid[PID_ROLL].I = 40;
    pidProfilesMutable(0)->pid[PID_ROLL].D = 44;
    pidProfilesMutable(0)->pid[PID_ROLL].F = 20;
    pidProfilesMutable(0)->pid[PID_PITCH].P = 72;
    pidProfilesMutable(0)->pid[PID_PITCH].I = 45;
    pidProfilesMutable(0)->pid[PID_PITCH].D = 43;
    pidProfilesMutable(0)->pid[PID_PITCH].F = 25;
    pidProfilesMutable(0)->pid[PID_YAW].P = 95;
    pidProfilesMutable(0)->pid[PID_YAW].I = 45;
    pidProfilesMutable(0)->pid[PID_YAW].D = 43;
    pidProfilesMutable(0)->pid[PID_YAW].F = 0;
    pidProfilesMutable(0)->dterm_notch_cutoff = 0;
    pidProfilesMutable(0)->d_min[FD_ROLL] = 20;
    pidProfilesMutable(0)->d_min[FD_PITCH] = 22;
    // plaid
    pidProfilesMutable(1)->pid[PID_ROLL].P = 62;
    pidProfilesMutable(1)->pid[PID_ROLL].I = 40;
    pidProfilesMutable(1)->pid[PID_ROLL].D = 44;
    pidProfilesMutable(1)->pid[PID_ROLL].F = 20;
    pidProfilesMutable(1)->pid[PID_PITCH].P = 71;
    pidProfilesMutable(1)->pid[PID_PITCH].I = 45;
    pidProfilesMutable(1)->pid[PID_PITCH].D = 41;
    pidProfilesMutable(1)->pid[PID_PITCH].F = 20;
    pidProfilesMutable(1)->pid[PID_YAW].P = 80;
    pidProfilesMutable(1)->pid[PID_YAW].I = 65;
    pidProfilesMutable(1)->pid[PID_YAW].D = 17;
    pidProfilesMutable(1)->pid[PID_YAW].F = 0;
    pidProfilesMutable(1)->dterm_notch_cutoff = 0;
    pidProfilesMutable(1)->d_min[FD_ROLL] = 17;
    pidProfilesMutable(1)->d_min[FD_PITCH] = 19;
    // black
    pidProfilesMutable(2)->pid[PID_ROLL].P = 71;
    pidProfilesMutable(2)->pid[PID_ROLL].I = 44;
    pidProfilesMutable(2)->pid[PID_ROLL].D = 44;
    pidProfilesMutable(2)->pid[PID_ROLL].F = 70;
    pidProfilesMutable(2)->pid[PID_PITCH].P = 75;
    pidProfilesMutable(2)->pid[PID_PITCH].I = 46;
    pidProfilesMutable(2)->pid[PID_PITCH].D = 43;
    pidProfilesMutable(2)->pid[PID_PITCH].F = 75;
    pidProfilesMutable(2)->pid[PID_YAW].P = 97;
    pidProfilesMutable(2)->pid[PID_YAW].I = 65;
    pidProfilesMutable(2)->pid[PID_YAW].D = 23;
    pidProfilesMutable(2)->pid[PID_YAW].F = 0;
    pidProfilesMutable(2)->dterm_notch_cutoff = 0;
    pidProfilesMutable(2)->d_min[FD_ROLL] = 16;
    pidProfilesMutable(2)->d_min[FD_PITCH] = 15;

    for (uint8_t rateProfileIndex = 0; rateProfileIndex < CONTROL_RATE_PROFILE_COUNT; rateProfileIndex++)
    {
        controlRateConfig_t *controlRateConfig = controlRateProfilesMutable(rateProfileIndex);

        controlRateConfig->rcRates[FD_ROLL] = 100;
        controlRateConfig->rcRates[FD_PITCH] = 100;
        controlRateConfig->rcRates[FD_YAW] = 100;
        controlRateConfig->rcExpo[FD_ROLL] = 15;
        controlRateConfig->rcExpo[FD_PITCH] = 15;
        controlRateConfig->rcExpo[FD_YAW] = 15;
        controlRateConfig->rates[FD_ROLL] = 73;
        controlRateConfig->rates[FD_PITCH] = 73;
        controlRateConfig->rates[FD_YAW] = 73;
        controlRateConfig->dynThrPID = 55;
    }

#if (defined(BEEBRAIN_LITED_US) || defined(BEEBRAIN_LITESF_US) || defined(BEEBRAIN_LITE_US))
    uint16_t vtxTableFrequency[6][8] = {
        {5865, 5845, 5825, 5805, 5785, 5765, 5745, 5725}, // Boscam A
        {5733, 5752, 5771, 5790, 5809, 5828, 5847, 5866}, // Boscam B
        {5705, 5685, 5665, 0, 5885, 5905, 0, 0},          // Boscam E
        {5740, 5760, 5780, 5800, 5820, 5840, 5860, 5880}, // FatShark
        {5658, 5695, 5732, 5769, 5806, 5843, 5880, 5917}, // RaceBand
        {5732, 5765, 5828, 5840, 5866, 5740, 0, 0},       // IMD6
    };
#else
    uint16_t vtxTableFrequency[6][8] = {
        {5865, 5845, 5825, 5805, 5785, 5765, 5745, 5725}, // Boscam A
        {5733, 5752, 5771, 5790, 5809, 5828, 5847, 5866}, // Boscam B
        {5705, 5685, 5665, 5645, 5885, 5905, 5925, 5945}, // Boscam E
        {5740, 5760, 5780, 5800, 5820, 5840, 5860, 5880}, // FatShark
        {5658, 5695, 5732, 5769, 5806, 5843, 5880, 5917}, // RaceBand
        {5732, 5765, 5828, 5840, 5866, 5740, 0, 0},       // IMD6
    };
#endif
    const char *vtxTableBandNames[6] = {
        "BOSCAM A",
        "BOSCAM B",
        "BOSCAM E",
        "FATSHARK",
        "RACEBAND",
        "IMD6"};
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

    vtxSettingsConfigMutable()->band = 5;
    vtxSettingsConfigMutable()->channel = 1;
    vtxSettingsConfigMutable()->power = 2;

    batteryConfigMutable()->batteryCapacity = 250;
    batteryConfigMutable()->vbatmincellvoltage = 280;
    batteryConfigMutable()->vbatwarningcellvoltage = 320;

    imuConfigMutable()->small_angle = 180;

    modeActivationConditionsMutable(0)->modeId          = BOXARM;
    modeActivationConditionsMutable(0)->auxChannelIndex = AUX1 - NON_AUX_CHANNEL_COUNT;
    modeActivationConditionsMutable(0)->range.startStep = CHANNEL_VALUE_TO_STEP(1700);
    modeActivationConditionsMutable(0)->range.endStep   = CHANNEL_VALUE_TO_STEP(2100);

    modeActivationConditionsMutable(1)->modeId          = BOXANGLE;
    modeActivationConditionsMutable(1)->auxChannelIndex = AUX2 - NON_AUX_CHANNEL_COUNT;
    modeActivationConditionsMutable(1)->range.startStep = CHANNEL_VALUE_TO_STEP(900);
    modeActivationConditionsMutable(1)->range.endStep   = CHANNEL_VALUE_TO_STEP(1300);

    modeActivationConditionsMutable(2)->modeId          = BOXHORIZON;
    modeActivationConditionsMutable(2)->auxChannelIndex = AUX2 - NON_AUX_CHANNEL_COUNT;
    modeActivationConditionsMutable(2)->range.startStep = CHANNEL_VALUE_TO_STEP(1300);
    modeActivationConditionsMutable(2)->range.endStep   = CHANNEL_VALUE_TO_STEP(1700);

    osdElementConfigMutable()->item_pos[OSD_CRAFT_NAME]         = OSD_POS( 9, 10) | OSD_PROFILE_1_FLAG;
    osdElementConfigMutable()->item_pos[OSD_MAIN_BATT_VOLTAGE]  = OSD_POS(23,  9) | OSD_PROFILE_1_FLAG;
    osdElementConfigMutable()->item_pos[OSD_ITEM_TIMER_2]       = OSD_POS( 2,  9) | OSD_PROFILE_1_FLAG;
    osdElementConfigMutable()->item_pos[OSD_FLYMODE]            = OSD_POS(18,  9) | OSD_PROFILE_1_FLAG;
    osdElementConfigMutable()->item_pos[OSD_VTX_CHANNEL]        = OSD_POS(10,  9) | OSD_PROFILE_1_FLAG;
    osdElementConfigMutable()->item_pos[OSD_RSSI_VALUE]         = OSD_POS( 2, 10) | OSD_PROFILE_1_FLAG;
    osdElementConfigMutable()->item_pos[OSD_WARNINGS]           = OSD_POS( 9, 10);

    ledStripStatusModeConfigMutable()->ledConfigs[0] = DEFINE_LED(7, 7, 8, 0, LF(COLOR), LO(LARSON_SCANNER) | LO(THROTTLE), 0);
    ledStripStatusModeConfigMutable()->ledConfigs[1] = DEFINE_LED(8, 7, 13, 0, LF(COLOR), LO(LARSON_SCANNER) | LO(THROTTLE), 0);
    ledStripStatusModeConfigMutable()->ledConfigs[2] = DEFINE_LED(9, 7, 11, 0, LF(COLOR), LO(LARSON_SCANNER) | LO(THROTTLE), 0);

    strcpy(pilotConfigMutable()->name, "BeeBrain Lite");

#if (defined(BEEBRAIN_LITE_DSM_INTL) || defined(BEEBRAIN_LITE_DSM_US))
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
#endif
}
#endif
