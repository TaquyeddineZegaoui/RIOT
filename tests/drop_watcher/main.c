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

#define JUST_SERIAL 0
#if JUST_SERIAL
#define ENABLE_FILTER
#endif

#if JUST_SERIAL
extern adc_t sensors[];
#endif

#define TIME_DROPS 10000000

#ifdef ENABLE_FILTER

extern uint8_t atd(uint16_t value);
extern uint16_t high_filter(uint16_t adc, uint16_t adc_1, uint16_t yn_1);
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
    uint16_t adc[3]={1023,1023,1023};
#endif

#ifdef ENABLE_FILTER
    int adc_1[3] = {1023,1023,1023};
    int y[3] = {0,0,0};
#endif

    while(1)
    {
#if JUST_SERIAL
    for(int i=0;i<3;i++)
    {
#ifdef ENABLE_FILTER
        adc_1[i] = adc[i];
#endif
        adc[i] = adc_sample(sensors[i], SENSOR_RES);
    } 

#ifdef ENABLE_FILTER
    for(int i=0;i<3;i++)
    {
        y[i] = high_filter(adc[i], adc_1[i], y[i]);
    }
    printf("%i,%i,%i,", adc[0], adc[1], adc[2]);
    printf("%i,%i,%i,", y[0], y[1], y[2]);
    printf("%i,%i,%i\n", atd(y[0]), atd(y[1]), atd(y[2]));
#else
    printf("%i,%i,%i\n", adc[0], adc[1], adc[2]);
#endif
    xtimer_usleep(1000);
#else
        count_drops(TIME_DROPS);
        int drops = get_drops();
        printf("%i\n", drops);
#endif
    }
    return 0;
}
