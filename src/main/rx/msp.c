/*
 * This file is part of Cleanflight.
 *
 * Cleanflight is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Cleanflight is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Cleanflight.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <stdbool.h>
#include <stdint.h>

#include <platform.h>

#include "build/build_config.h"

#include "config/parameter_group.h"

#include "drivers/dma.h"
#include "drivers/system.h"

#include "drivers/serial.h"
#include "drivers/serial_uart.h"
#include "io/serial.h"

#include "rx/rx.h"
#include "rx/msp.h"

#include "fc/rc_controls.h"

static uint16_t mspFrame[MAX_SUPPORTED_RC_CHANNEL_COUNT];
static bool rxMspFrameDone = false;

static uint16_t rxMspReadRawRC(const rxRuntimeConfig_t *rxRuntimeConfigPtr, uint8_t chan)
{
    UNUSED(rxRuntimeConfigPtr);
    return mspFrame[chan];
}

void rxMspFrameReceive(uint16_t *frame, int channelCount)
{
    for (int i = 0; i < channelCount; i++) {
        mspFrame[i] = frame[i];
    }

    // Any channels not provided will be reset to zero
    for (int i = channelCount; i < MAX_SUPPORTED_RC_CHANNEL_COUNT; i++) {
        mspFrame[i] = 0;
    }

    rxMspFrameDone = true;
}

void rxMspChannelsReset()
{
    // Also clear MSP channels to make sure old data doesn't sneak in
    mspFrame[ROLL]  = 1500;
    mspFrame[PITCH]  = 1500;

    // It was found through testing that THROTTLE was actually YAW and YAW was THROTTLE
    // It appears that items in these arrays aren't in the same order as in rx.c
    mspFrame[THROTTLE]  = 1500;
    mspFrame[YAW] = 1000;
    for(uint8_t channel = AUX1; channel < MAX_SUPPORTED_RC_CHANNEL_COUNT; channel++)
    {
        mspFrame[channel] = 1000;
    }
}

uint8_t rxMspFrameStatus(void)
{
    if (!rxMspFrameDone) {
        return RX_FRAME_PENDING;
    }

    rxMspFrameDone = false;
    return RX_FRAME_COMPLETE;
}

void rxMspInit(const rxConfig_t *rxConfig, rxRuntimeConfig_t *rxRuntimeConfig)
{
    UNUSED(rxConfig);

    rxRuntimeConfig->channelCount = MAX_SUPPORTED_RC_CHANNEL_COUNT;
    rxRuntimeConfig->rxRefreshRate = 20000;

    rxRuntimeConfig->rcReadRawFn = rxMspReadRawRC;
    rxRuntimeConfig->rcFrameStatusFn = rxMspFrameStatus;
}
