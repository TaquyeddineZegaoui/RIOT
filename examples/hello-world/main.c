
/**
 * @ingroup     examples
 * @{
 *
 * @file
 * @brief       Hello World application
 *
 * @author      Kaspar Schleiser <kaspar@schleiser.de>
 * @author      Ludwig Knüpfer <ludwig.knuepfer@fu-berlin.de>
 *
 * @}
 */

#include <stdio.h>
#include "rtctimers.h"
#include "periph/rtc.h"
#include "xtimer.h"
#include "arch/lpm_arch.h"

#define TM_YEAR_OFFSET              (1900)

#if RTC_NUMOF < 1
#error "No RTC found. See the board specific periph_conf.h."
#endif

void cb(void *arg)
{
    (void)arg;

    puts("Alarm!");

    struct tm time;
    rtc_get_alarm(&time);
    time.tm_sec  += 20;
    if ( time.tm_sec > 60 )
        rtc_clear_alarm();
    time.tm_min  += 2;
    if ( time.tm_sec > 60 )
        rtc_clear_alarm(); 
    rtc_set_alarm(&time, cb, 0);
}

int main(void)
{
	lpm_arch_init();
    puts("Hello World!");
    printf("You are running RIOT on a(n) %s board.\n", RIOT_BOARD);

    rtc_init();
 
    struct tm time;
    time.tm_year = 2016 - TM_YEAR_OFFSET; // years are counted from 1900
    time.tm_mon  = 0; // 0 = January, 11 = December
    time.tm_mday = 16;
    time.tm_hour = 14;
    time.tm_min  = 15;
    time.tm_sec  = 15;
    rtc_set_time(&time);

    time.tm_sec  += 10;

    rtc_set_alarm(&time, cb, 0);
    lpm_set(LPM_POWERDOWN);
    printf("HELLO! %s\n", "Paula");
    puts("Wake"); 

    while(1)
    {
        puts("Wake"); 
        xtimer_usleep(10000000);
    }

    return 0;
}
