#include "periph/gpio.h"
#include "random.h"
#include "common.h"

#ifdef NZ32_SC151

#include "periph/adc.h"

/*!
 * Battery level ratio (battery dependent)
 */

#define BAT_LEVEL           GPIO_PIN(PORT_A, 13)
#define USB_DETECT          GPIO_PIN(PORT_C, 4)
#define ADC_RESOLUTION      ADC_RES_12BIT
#define ADC_VREF			ADC_LINE(4)
#define ADC_VDIV			ADC_LINE(3)

/*!
 * Factory power supply
 */
#define FACTORY_POWER_SUPPLY                        3.0L

/*!
 * VREF calibration value
 */
#define VREFINT_CAL                                 ( *( uint16_t* )0x1FF800F8 )// /4 to go form 12 to 10 bits

/*!
 * ADC maximum value
 */
#define ADC_MAX_VALUE                               4096

/*!
 * Battery thresholds
 */
#define BATTERY_MAX_LEVEL                           4150 // mV
#define BATTERY_MIN_LEVEL                           3200 // mV
#define BATTERY_SHUTDOWN_LEVEL                      3100 // mV

/*!
 * Unique Devices IDs register set ( STM32L1xxx )
 */
#define         ID1                                 ( 0x1FF800D0 )
#define         ID2                                 ( 0x1FF800D4 )
#define         ID3                                 ( 0x1FF800E4 )

enum BoardPowerSource
{
    USB_POWER = 0,
    BATTERY_POWER
};

#endif

uint16_t board_get_power_supply( void );
uint8_t board_get_battery_level( void );
uint8_t get_board_power_source( void );
uint32_t board_get_random_seed( void );
void board_get_unique_id( uint8_t *id );
void board_get_node_id( uint8_t *id , uint8_t id_length, uint8_t *node_id );