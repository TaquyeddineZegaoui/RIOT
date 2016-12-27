/*!
* Headers for the infrared managment
*/

#include <stdlib.h>
#include <stdio.h>
#include "periph/gpio.h"
#include "common.h"

#ifdef NZ32_SC151

#include "periph/adc.h"

#define SENSOR_RES          ADC_RES_10BIT
#define THRESHOLD           80
#define NSENSOR           	3
#define SAMPLE_RATE         (1U* MS_IN_USEC) /* 1 ms */

/*!
* Infrared sensor pins
*/
#define EMITTER_PIN         GPIO_PIN(PORT_B,12)

#define INFRARED_ON			gpio_set(EMITTER_PIN)
#define INFRARED_OFF		gpio_clear(EMITTER_PIN)

#endif

/*!
 * Drops Struct to store time and drop amount
 */
typedef struct drops
{
	uint8_t number;
	uint16_t sampled_time;
}drops_t;

uint8_t count_drops ( uint32_t sample_time );
uint8_t get_drops( void );
uint16_t get_time( void );
