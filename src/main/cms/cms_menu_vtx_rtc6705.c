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
#include <ctype.h>

#include "platform.h"

#if defined(USE_CMS) && defined(USE_VTX_RTC6705)

#include "common/printf.h"
#include "common/utils.h"

#include "cms/cms.h"
#include "cms/cms_types.h"

#include "drivers/vtx_common.h"

#include "fc/config.h"

#include "io/vtx_string.h"
#include "io/vtx_rtc6705.h"
#include "io/vtx.h"

#if defined(USE_VTX_LOCK_FREQ)
#define VTX_FREQ_ISM_MAX                5920
#define VTX_FREQ_ISM_MIN                5650
#endif // USE_VTX_LOCK_FREQ

static uint8_t cmsx_vtxBand;
static uint8_t cmsx_vtxChannel;
static uint8_t cmsx_vtxPower;
#if defined(USE_VTX_LOCK_FREQ)
static uint8_t lastBand;
static uint8_t lastChannel;
#endif // USE_VTX_LOCK_FREQ

static OSD_TAB_t entryVtxBand;
static OSD_UINT8_t entryVtxChannel;
static OSD_TAB_t entryVtxPower;

static void cmsx_Vtx_ConfigRead(void)
{
    cmsx_vtxBand = vtxSettingsConfig()->band - 1;
    cmsx_vtxChannel = vtxSettingsConfig()->channel;
    cmsx_vtxPower = vtxSettingsConfig()->power - VTX_RTC6705_MIN_POWER;
#if defined(USE_VTX_LOCK_FREQ)
    lastBand = cmsx_vtxBand;
    lastChannel = cmsx_vtxChannel;
#endif // USE_VTX_LOCK_FREQ
}

static void cmsx_Vtx_ConfigWriteback(void)
{
    // update vtx_ settings
    vtxSettingsConfigMutable()->band = cmsx_vtxBand + 1;
    vtxSettingsConfigMutable()->channel = cmsx_vtxChannel;
    vtxSettingsConfigMutable()->power = cmsx_vtxPower + VTX_RTC6705_MIN_POWER;
    vtxSettingsConfigMutable()->freq = vtxCommonLookupFrequency(vtxCommonDevice(), cmsx_vtxBand + 1, cmsx_vtxChannel);

    saveConfigAndNotify();
}

#if defined(USE_VTX_LOCK_FREQ)
static uint8_t cms_Vtx_Check_Lock_Freq(uint16_t freq) {
    if (freq < VTX_FREQ_ISM_MIN || freq > VTX_FREQ_ISM_MAX) {
        return 1;
    }
    return 0;
}

static long cmsx_Vtx_Band_Lock_Check(displayPort_t *pDisp, const void *self) {
    UNUSED(pDisp);
    UNUSED(self);
    if (cms_Vtx_Check_Lock_Freq(vtxCommonLookupFrequency(vtxCommonDevice(), cmsx_vtxBand + 1, cmsx_vtxChannel))) {
        cmsx_vtxChannel = 1;
    }
    while (1) {
        if (cms_Vtx_Check_Lock_Freq(vtxCommonLookupFrequency(vtxCommonDevice(), cmsx_vtxBand + 1, cmsx_vtxChannel))) {
            cmsx_vtxChannel += 1;
            if (cmsx_vtxChannel > VTX_SETTINGS_CHANNEL_COUNT - 1) {
                if (lastBand - cmsx_vtxBand >= 1) {
                    cmsx_vtxBand -= 1;
                    cmsx_vtxChannel = 1;
                } else {
                    cmsx_vtxBand += 1;
                    cmsx_vtxChannel = 1;
                }
            }
        } else {
            lastBand = cmsx_vtxBand;
            break;
        }
    }
    return 0;
}

static long cmsx_Vtx_Channel_Lock_Check(displayPort_t *pDisp, const void *self) {
    UNUSED(pDisp);
    UNUSED(self);
    while (1) {
        if (cms_Vtx_Check_Lock_Freq(vtxCommonLookupFrequency(vtxCommonDevice(), cmsx_vtxBand + 1, cmsx_vtxChannel))) {
            if (lastChannel - cmsx_vtxChannel >= 1) {
                cmsx_vtxChannel -= 1;
            } else {
                cmsx_vtxChannel += 1;
            }
            if (cmsx_vtxChannel == 0 || cmsx_vtxChannel == 9) {
                cmsx_vtxChannel = lastChannel;
                break;
            }
        } else {
            lastChannel = cmsx_vtxChannel;
            break;
        }
    }
    return 0;
}
#endif // USE_VTX_LOCK_FREQ

static long cmsx_Vtx_onEnter(void)
{
    cmsx_Vtx_ConfigRead();

    vtxDevice_t *device = vtxCommonDevice();

    entryVtxBand.val = &cmsx_vtxBand;
    entryVtxBand.max = device->capability.bandCount - 1;
    entryVtxBand.names = &device->bandNames[1];

    entryVtxChannel.val = &cmsx_vtxChannel;
    entryVtxChannel.min = 1;
    entryVtxChannel.max = device->capability.channelCount;
    entryVtxChannel.step = 1;

    entryVtxPower.val = &cmsx_vtxPower;
    entryVtxPower.max = device->capability.powerCount - 1;
    entryVtxPower.names = device->powerNames;

    return 0;
}

static long cmsx_Vtx_onExit(const OSD_Entry *self)
{
    UNUSED(self);

    cmsx_Vtx_ConfigWriteback();

    return 0;
}


static const OSD_Entry cmsx_menuVtxEntries[] =
{
    {"--- VTX ---", OME_Label, NULL, NULL, 0},
#if defined(USE_VTX_LOCK_FREQ)
    {"BAND", OME_TAB, cmsx_Vtx_Band_Lock_Check, &entryVtxBand, DYNAMIC},
    {"CHANNEL", OME_UINT8, cmsx_Vtx_Channel_Lock_Check, &entryVtxChannel, DYNAMIC},
#else
    {"BAND", OME_TAB, NULL, &entryVtxBand, 0},
    {"CHANNEL", OME_UINT8, NULL, &entryVtxChannel, 0},
#endif // USE_VTX_LOCK_FREQ
    {"POWER", OME_TAB, NULL, &entryVtxPower, 0},
    {"BACK", OME_Back, NULL, NULL, 0},
    {NULL, OME_END, NULL, NULL, 0}
};

CMS_Menu cmsx_menuVtxRTC6705 = {
#ifdef CMS_MENU_DEBUG
    .GUARD_text = "MENUVTX",
    .GUARD_type = OME_MENU,
#endif
    .onEnter = cmsx_Vtx_onEnter,
    .onExit= cmsx_Vtx_onExit,
    .entries = cmsx_menuVtxEntries
};

#endif // CMS
