/*
 * Copyright (C) 2016 Fundacion Inria Chile
 *
 * This file is subject to the terms and conditions of the GNU Lesser General
 * Public License v2.1. See the file LICENSE in the top level directory for more
 * details.
 */

/* @file        infrared.h
 * @brief       Headers for the infrared sensor
 *
 * @author      Francisco Molina <francisco.molina@inria.cl>
*/

#ifndef __INFRARED_H__
#define __INFRARED_H__

#include <stdlib.h>
#include <stdio.h>
#include "periph/gpio.h"
#include "driver_config.h"

#ifdef NZ32_SC151

#include "periph/adc.h"

#define SENSOR_RES          ADC_RES_10BIT	 
#define THRESHOLD           100				 /* For filtering */
#define NSENSOR           	3				 /* Number of RF receptors*/
#define SAMPLE_RATE         (1U* MS_IN_USEC) /* 1 ms */

/*!
* Infrared sensor adc lines
*/
#define SENSOR_0 			ADC_LINE(0)
#define SENSOR_1 			ADC_LINE(1)
#define SENSOR_2 			ADC_LINE(2)

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


/**
 * @brief Counts drops for sample_time time
 *
 * @param[in] sample_time    time one want's to measure in us
 *
 * @return                   1 if succesfull, 0 if failed
 */
uint8_t count_drops ( uint32_t sample_time );

/**
 * @brief Returns the amount of drops measured in last count_drops(..) call
 *
 * @return              amount of drop's counted, 0xFF means error
 */
uint8_t get_drops( void );

/**
 * @brief Returns actual measuring time, can differ a little from sample_time
 *
 * @return              measured time in ms
 */
uint16_t get_time( void );

#endif // __INFRARED_H__