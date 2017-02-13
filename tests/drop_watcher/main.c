/*
 * Copyright (C) 2014  René Kijewski  <rene.kijewski@fu-berlin.de>
 *
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2.1 of the License, or (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with this library; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301  USA
 */

/**
 * @ingroup   tests
 * @{
 *
 * @file
 * @brief     Print some calculations for libfixmath
 *
 * @author    René Kijewski <rene.kijewski@fu-berlin.de>
 *
 * @}
 */

#include <stdio.h>
#include "infrared.h"
#include "boards_hw.h"

#define JUST_SERIAL 1

#if JUST_SERIAL
adc_t sensors[] = {ADC_LINE(0), ADC_LINE(1), ADC_LINE(2)};
#endif

int main(void)
{
    adc_init(ADC_VDIV);
    adc_init(ADC_LINE(0));
    adc_init(ADC_LINE(1));
    adc_init(ADC_LINE(2));
    adc_init(ADC_VREF);
    gpio_init(USB_DETECT, GPIO_IN);
    gpio_init(EMITTER_PIN, GPIO_OUT);
    gpio_init(BAT_LEVEL, GPIO_OUT);
    gpio_clear(BAT_LEVEL);
    INFRARED_ON;

#if JUST_SERIAL
    uint16_t adc[3];
#endif
    while(1)
    {
#if JUST_SERIAL
    for(int i=0;i<3;i++)
    {
        adc[i] = adc_sample(sensors[i], SENSOR_RES);
    } 
    printf("%i,%i,%i\n", adc[0], adc[1], adc[2]);
    xtimer_usleep(1000);
#else
        count_drops(10000000);
#endif
    }
    return 0;
}
