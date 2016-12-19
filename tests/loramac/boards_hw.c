#include "boards_hw.h"

uint16_t board_get_power_supply( void )
{
    int vref = 0;
    int vdiv = 0;
    uint64_t batteryVoltage = 0;

    adc_sample(ADC_VDIV, ADC_RESOLUTION);    // dummy read
    vdiv  = adc_sample(ADC_VDIV, ADC_RESOLUTION);
    vref  = adc_sample(ADC_VREF, ADC_RESOLUTION);

    batteryVoltage =  (  FACTORY_POWER_SUPPLY * VREFINT_CAL * vdiv ) *1000 /  (  vref * ADC_MAX_VALUE );
    //                                vDiv
    // Divider bridge  VBAT <-> 1M -<--|-->- 1M <-> GND => vBat = 2 * vDiv
    batteryVoltage = 2 * batteryVoltage;

    return ( uint16_t )( batteryVoltage);

    return 0;

}

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

    return 0;

}

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

uint32_t board_get_random_seed( void )
{
    return ( ( *( uint32_t* )ID1 ) ^ ( *( uint32_t* )ID2 ) ^ ( *( uint32_t* )ID3 ) );
}

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