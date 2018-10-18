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

#include <stdint.h>

typedef struct beeSignDevice_s {
    uint8_t version;
    uint8_t channel;
    uint8_t power;
    uint8_t lastPower;
    uint8_t mode;
    uint16_t freq;
    uint16_t porfreq;
} beeSignDevice_t;

extern beeSignDevice_t bsDevice;

bool beesignVtxInit(void);
void beesignProcess(timeUs_t currentTimeUs);
