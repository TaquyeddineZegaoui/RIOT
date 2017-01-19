/*
 * Copyright (C) 2016 Fundacion Inria Chile
 *
 * This file is subject to the terms and conditions of the GNU Lesser General
 * Public License v2.1. See the file LICENSE in the top level directory for more
 * details.
 */

/* @file        boards_hw.c
 * @brief       Specific board start-up functions, auxiliary functions and misc 
 *              used for app functions and measurement
 *
 * @author      Francisco Molina <francisco.molina@inria.cl>
*/

#include "boards_hw.h"
#include "embUnit/embUnit.h"
#include "hashes/md5.h"

#ifdef NZ32_SC151

/*!
 * Returns battery voltage
 */
uint16_t board_get_power_supply( void )
{
    int vref = 0;
    int vdiv = 0;
    uint64_t batteryVoltage = 0;

    adc_sample(ADC_VDIV, ADC_RESOLUTION);           // dummy read
    vdiv  = adc_sample(ADC_VDIV, ADC_RESOLUTION);
    vref  = adc_sample(ADC_VREF, ADC_RESOLUTION);
    xtimer_usleep(200000);                          // 200ms delay because of Vdiv first reading bug after startup
    
    vdiv  = adc_sample(ADC_VDIV, ADC_RESOLUTION);
    vref  = adc_sample(ADC_VREF, ADC_RESOLUTION);

    /* Divider bridge  VBAT <-> 0.47M -<--|-->- 0.47M <-> GND => vBat = 2 * vDiv */
    batteryVoltage =  (  FACTORY_POWER_SUPPLY * VREFINT_CAL * vdiv ) *1000 /  (  vref * ADC_MAX_VALUE );
    batteryVoltage = 2 * batteryVoltage;

    return ( uint16_t )( batteryVoltage);

    return 0;

}

/*!
 * Returns battery level calculated from battery voltage
 * 1 = Min level; 254 = max level; 255 = battery shutdown; 0 = USB power source
 */
uint8_t board_get_battery_level( void )
{
    volatile uint8_t batteryLevel = 0;
    uint16_t batteryVoltage = 0;

    if( get_board_power_source() == USB_POWER )
    {
        batteryLevel = 0;
    }
    else
    {
        batteryVoltage = board_get_power_supply( );

        if( batteryVoltage >= BATTERY_MAX_LEVEL )
        {
            batteryLevel = 254;
        }
        else if( ( batteryVoltage > BATTERY_MIN_LEVEL ) && ( batteryVoltage < BATTERY_MAX_LEVEL ) )
        {
            batteryLevel = ( ( 253 * ( batteryVoltage - BATTERY_MIN_LEVEL ) ) / ( BATTERY_MAX_LEVEL - BATTERY_MIN_LEVEL ) ) + 1;
        }
        else if( batteryVoltage <= BATTERY_SHUTDOWN_LEVEL )
        {
            batteryLevel = 255;
        }
        else // BATTERY_MIN_LEVEL
        {
            batteryLevel = 1;
        }
    }
    return batteryLevel;
}


/*!
 * Returns board power source, USB or battery
 */
uint8_t get_board_power_source( void )
{
    if( gpio_read(USB_DETECT) == 0 )
    {
        return BATTERY_POWER;
    }
    else
    {
        return USB_POWER;
    }
}

/*!
 * Returns random seed from Unique device ID
 */
uint32_t board_get_random_seed( void )
{
    return ( ( *( uint32_t* )ID1 ) ^ ( *( uint32_t* )ID2 ) ^ ( *( uint32_t* )ID3 ) );
}

/*!
 * Returns board's unique device ID
 */
void board_get_unique_id( uint8_t *id )
{
    id[7] = ( ( *( uint32_t* )ID1 )+ ( *( uint32_t* )ID3 ) ) >> 24;
    id[6] = ( ( *( uint32_t* )ID1 )+ ( *( uint32_t* )ID3 ) ) >> 16;
    id[5] = ( ( *( uint32_t* )ID1 )+ ( *( uint32_t* )ID3 ) ) >> 8;
    id[4] = ( ( *( uint32_t* )ID1 )+ ( *( uint32_t* )ID3 ) );
    id[3] = ( ( *( uint32_t* )ID2 ) ) >> 24;
    id[2] = ( ( *( uint32_t* )ID2 ) ) >> 16;
    id[1] = ( ( *( uint32_t* )ID2 ) ) >> 8;
    id[0] = ( ( *( uint32_t* )ID2 ) );
}

/*!
 * Init's ADC anf GPIO peripherials
 */
void board_init_periph( void )
{
    /* initialize all available ADC lines */
    adc_init(ADC_VDIV);
    adc_init(ADC_LINE(0));
    adc_init(ADC_LINE(1));
    adc_init(ADC_LINE(2));
    adc_init(ADC_VREF);
    gpio_init(USB_DETECT, GPIO_IN);       // Init USB_Detect
    gpio_init(EMITTER_PIN, GPIO_OUT);     // Init infrared emitter pin
    gpio_init(BAT_LEVEL, GPIO_OUT);       // Init Battary level measurement pin
    gpio_clear(BAT_LEVEL);
}

#else

/*!
 * Define dummy function's for non supported boards
 */

uint16_t board_get_power_supply( void )
{
    return 0;
}
uint8_t board_get_battery_level( void )
{
    return 0;
}
uint8_t get_board_power_source( void )
{
    return 0;
}
uint32_t board_get_random_seed( void )
{
    return 0;
}
void board_get_unique_id( uint8_t *id )
{

}
#endif

/*!
 * Generates a 4 byte node_id from DevEUI using hash md5
 */
void board_get_node_id( uint8_t *id , uint8_t id_length, uint8_t *node_id )
{
    uint8_t hash[MD5_DIGEST_LENGTH];

    /* calculate hash */
    md5(hash, (const uint8_t *) id, id_length);

    /* truncate hash*/
    node_id[3] = hash[3];
    node_id[2] = hash[2];
    node_id[1] = hash[1];
    node_id[0] = hash[0];
}

/*!
 * Checks if time structure adjustes to range, if not it overflows to next
 * time unit.
 */
void time_check(struct tm* time)
{
    if ( time->tm_sec > 59 )
    {
        time->tm_min += (time->tm_sec)/60;
        time->tm_sec %= 60;
    }
    if ( time->tm_min > 59 )
    {
        time->tm_hour += (time->tm_min)/60;
        time->tm_min %= 60;
    }
    if ( time->tm_hour > 23 )
    {
        time->tm_yday += (time->tm_hour)/24;
        time->tm_hour %= 24;
    }
    if ( time->tm_yday > 365 )
    {
        time->tm_year += (time->tm_sec)/365;
        time->tm_yday %= 365;
    }
}

/*!
 * Start's the rtc timer and lora device.
 */
void app_init (sx1276_t* dev)
{
    /* set sx1276 pointer*/
    radio_set_ptr(dev);

    /* init rtc and set time*/
    struct tm rtc_time;
    rtc_init();
    rtc_time.tm_year = 2017 - TM_YEAR_OFFSET; // years are counted from 1900
    rtc_time.tm_yday = 0;
    rtc_time.tm_hour = 0;
    rtc_time.tm_min  = 0;
    rtc_time.tm_sec  = 0;
    rtc_set_time(&rtc_time);
    rtc_time.tm_sec  += 0;
    rtc_set_alarm(&rtc_time, NULL, 0);
    rtc_clear_alarm();
}
