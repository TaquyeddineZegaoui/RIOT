/*
 * Copyright (C) 2016 Fundacion Inria Chile
 *
 * This file is subject to the terms and conditions of the GNU Lesser General
 * Public License v2.1. See the file LICENSE in the top level directory for more
 * details.
 */

/* @file        infrared.c
 * @brief       Infrared sensor measurement function's
 *
 * @author      Francisco Molina <francisco.molina@inria.cl>
*/

#include "infrared.h"
#include "xtimer.h"

#define ENABLE_DEBUG  (0)
#include "debug.h"

#ifdef NZ32_SC151

/* drops structure and adc lines*/
static drops_t drops = {0, 0};
static adc_t sensors[] = {SENSOR_0, SENSOR_1, SENSOR_2};


/**
 * @brief   Extract the port number form the given identifier
 *
 */
static bool drop_detect(uint8_t *drop_bits, uint8_t last_accum_drop)
{
	/* shift's left drop bits and sums last bit accum_drop*/
	*drop_bits = ((*drop_bits) << 1) | (last_accum_drop & 1);
	/* see's if sequence is 0x07, ore that there is a reading 3 times in a row*/
	return (*drop_bits & 7) == 0x07;
}

/**
 * @brief   Threshold analog too discrete
 */
static uint8_t atd(uint16_t value)
{
	return (value > THRESHOLD);
}

/**
 * @brief   High filter for adc reading, emulates a differentiator
 */
static uint16_t high_filter(uint16_t adc, uint16_t adc_1, uint16_t yn_1)
{
 
  double cte_in = 0.99502;
  double cte_out = 0.99004;
  uint16_t y;
 
  y = cte_in*(adc-adc_1)+cte_out*yn_1;
  
 return y;
}

/**
 * @brief Counts drops for sample_time time
 *
 * @param[in] sample_time    time one want's to measure in us
 *
 * @return                   1 if succesfull, 0 if failed
 */
uint8_t count_drops (uint32_t sample_time)
{

	/* Deliver precise measured time*/
    uint64_t start_time =  xtimer_now_usec64();
    xtimer_ticks32_t start_ticks =  xtimer_now();
    uint64_t actual_time = xtimer_now_usec64();

    /* Filter variables*/
    uint16_t x_0[NSENSOR] = {1023,1023,1023};
    uint16_t x_1[NSENSOR] = {1023,1023,1023};
    uint16_t y[NSENSOR]   = {0,0,0};

    /* Aux Variables*/
    uint8_t drop_counts = 0;
	uint8_t drop_bool   = 0;
	uint8_t drop_agreg  = 0;
	uint8_t debouncer = 0x00;
	uint8_t drop_bits = 0x00;

	/* Sample Each Sensor initial values, 10 ms*/
     while( (actual_time - start_time) < 10*1000 )
    {
    	INFRARED_ON;

	    /* Sample Each Sensor */
        for (uint8_t i = 0; i < NSENSOR; i++) 
        {
			x_1[i] = x_0[i];
            x_0[i] = adc_sample(sensors[i], SENSOR_RES);
        }

        INFRARED_OFF;

		/* Reset agregated dropś*/
        drop_agreg  = 0;

		/* Agregated measurement*/
		for(uint8_t i = 0; i < NSENSOR; i++)
		{
			y[i] = high_filter(x_0[i], x_1[i], y[i]);
			drop_bool = atd(y[i]);
			drop_agreg |= drop_bool;
		}

		/* Filter output drops */
		drop_detect(&drop_bits, drop_agreg);

		/* Update time*/
		actual_time = xtimer_now_usec64();
		if((actual_time - start_time) < sample_time)
			xtimer_periodic_wakeup(&start_ticks, SAMPLE_RATE );
	}

    /* Deliver precise measured time*/
    start_time =  xtimer_now_usec64();
    start_ticks =  xtimer_now();
    actual_time = xtimer_now_usec64();

    DEBUG("START\n");

    /* Start actual measurement*/
    while( (actual_time - start_time) < sample_time )
    {
    	INFRARED_ON;

	    /* Sample Each Sensor */
        for (uint8_t i = 0; i < NSENSOR; i++) 
        {
			x_1[i] = x_0[i];
            x_0[i] = adc_sample(sensors[i], SENSOR_RES);
        }

        INFRARED_OFF;

		/* Reset agregated dropś*/
        drop_agreg  = 0;

		/* Agregated measurement*/
		for(uint8_t i = 0; i < NSENSOR; i++)
		{
			y[i] = high_filter(x_0[i], x_1[i], y[i]);
			drop_bool = atd(y[i]);
			drop_agreg |= drop_bool;
		}

		debouncer = ((debouncer << 1) | drop_detect(&drop_bits, drop_agreg)) & 0x1F;

		/* Filter output drops */
		if(debouncer == 0x01)
		{
			drop_counts++;
			DEBUG("Drop: %d \n",drop_counts);
		}
		
		/* If overflow, return failure*/
		if(drop_counts > 255)
			return 0;

		/* Update time*/
		actual_time = xtimer_now_usec64();
		if((actual_time - start_time) < sample_time)
			xtimer_periodic_wakeup(&start_ticks, SAMPLE_RATE );
	}

	drops.sampled_time = (actual_time - start_time)*1000;
	drops.number = drop_counts;

	DEBUG("STOP\n");

    return 1;
}

/**
 * @brief   Returns drop counts
 */
uint8_t get_drops( void )
{
	return drops.number;
}

/**
 * @brief   Returns sampled time
 */
uint16_t get_time( void )
{
	return drops.sampled_time;
}

#else

uint8_t count_drops ( uint32_t sample_time )
{
	return 0;
}
uint8_t get_drops( void )
{
	return 0;
}
uint16_t get_time( void )
{
	return 0;
}

#endif