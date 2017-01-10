#include "infrared.h"
#include "xtimer.h"

#ifdef NZ32_SC151

/* drops structure and adc lines*/
static drops_t drops = {0, 0};

adc_t sensors[] = {ADC_LINE(0), ADC_LINE(1), ADC_LINE(2)};

static bool drop_detect(uint8_t *drop_bits, uint8_t last_accum_drop)
{
	/* shift's left drop bits and sums last bit accum_drop*/
	*drop_bits = ((*drop_bits) << 1) | (last_accum_drop & 1);
	/* see's if sequence is 0x01*/
	return (*drop_bits & 7) == 0x07;
}

static uint8_t atd(uint16_t value)
{
	return (value > THRESHOLD);
}

static uint16_t high_filter(uint16_t adc, uint16_t adc_1, uint16_t yn_1)
{
 
  double cte_in = 0.99502;
  double cte_out = 0.99004;
  uint16_t y;
 
  y = cte_in*(adc-adc_1)+cte_out*yn_1;
  
 return y;
}

uint8_t count_drops (uint32_t sample_time)
{

	INFRARED_ON;

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
	    /* Sample Each Sensor */
        for (uint8_t i = 0; i < NSENSOR; i++) 
        {
			x_1[i] = x_0[i];
            x_0[i] = adc_sample(sensors[i], SENSOR_RES);
        }

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

    //puts("start");

    while( (actual_time - start_time) < sample_time )
    {
	    /* Sample Each Sensor */
        for (uint8_t i = 0; i < NSENSOR; i++) 
        {
			x_1[i] = x_0[i];
            x_0[i] = adc_sample(sensors[i], SENSOR_RES);
        }

        //printf("%d %d %d \n", x_0[2], x_0[1], x_0[0]);
		
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
			//printf("Drop: %d \n",drop_counts);
		}
		
		if(drop_counts > 255)
			return 0;

		/* Update time*/
		actual_time = xtimer_now_usec64();
		if((actual_time - start_time) < sample_time)
			xtimer_periodic_wakeup(&start_ticks, SAMPLE_RATE );
	}

	drops.sampled_time = (actual_time - start_time)*1000;
	drops.number = drop_counts;

	//puts("stop");

    INFRARED_OFF;

    return 1;
}

uint8_t get_drops( void )
{
	return drops.number;
}


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