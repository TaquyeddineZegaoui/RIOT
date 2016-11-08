#ifndef LORAMAC_BOARD_DEFINITIONS_H

#include <stdio.h>
#include <stdbool.h>
#include <inttypes.h>
#include "loramac/timer.h"
#include "loramac/radio.h"
#include "loramac/utilities.h"

//TODO: Remove this line
#define REG_LR_SYNCWORD 0x39

/*!
 * Radio wakeup time from SLEEP mode
 */
#define RADIO_OSC_STARTUP                           1 // [ms]

/*!
 * Radio PLL lock and Mode Ready delay which can vary with the temperature
 */
#define RADIO_SLEEP_TO_RX                           2 // [ms]

/*!
 * Radio complete Wake-up Time with margin for temperature compensation
 */
#define RADIO_WAKEUP_TIME ( RADIO_OSC_STARTUP + RADIO_SLEEP_TO_RX )

void srand1( uint32_t seed );
void memcpyr( uint8_t *dst, const uint8_t *src, uint16_t size );
void memset1( uint8_t *dst, uint8_t value, uint16_t size );
int32_t randr( int32_t min, int32_t max );
void memcpy1( uint8_t *dst, const uint8_t *src, uint16_t size );

#endif

