/*
 / _____)             _              | |
( (____  _____ ____ _| |_ _____  ____| |__
 \____ \| ___ |    (_   _) ___ |/ ___)  _ \
 _____) ) ____| | | || |_| ____( (___| | | |
(______/|_____)_|_|_| \__)_____)\____)_| |_|
    (C)2013 Semtech
 
Description: Timer objects and scheduling management
 
License: Revised BSD License, see LICENSE.TXT file include in the project
 
Maintainer: Miguel Luis and Gregory Cristian
*/

#include "net/lorawan/board_definitions.h"
#include "xtimer.h"

/*!
 * This flag is used to make sure we have looped through the main several time to avoid race issues
 */
volatile uint8_t HasLoopedThroughMain = 0;


void TimerInit( TimerEvent_t *obj, void ( *cb )( void ), kernel_pid_t target_pid)
{
    obj->dev.target = 0;
    //obj->running = 0;
    obj->dev.callback = (xtimer_callback_t) cb;
    obj->pid = target_pid;
}

/*void TimerReset( TimerEvent_t *obj, uint8_t opt)
{
    TimerStop(obj);
    TimerStart(obj, opt);
}*/
 
/*void TimerStart( TimerEvent_t *obj, uint8_t opt)
{
    obj->running = 1;
    if(opt)
        xtimer_set(&(obj->dev), obj->timeout);
    else
        xtimer_set_msg (&(obj->dev), obj->timeout, &(obj->msg), obj->pid);
}
*/
 
void TimerStop( TimerEvent_t *obj )
{
    //obj->running = 0;
    xtimer_remove(&(obj->dev));
}
 
/*
void TimerSetValue( TimerEvent_t *obj, uint32_t value, uint8_t message_ct)
{
    xtimer_ticks32_t ticks = xtimer_ticks_from_usec(value*1000);

    if(obj->running)
        xtimer_remove(&(obj->dev));
    obj->timeout = ticks.ticks32;
    obj->msg.type = message_ct;
}
*/

/*TimerTime_t TimerGetCurrentTime( void )
{
    uint64_t CurrentTime = xtimer_now_usec64();
    return ( ( TimerTime_t )CurrentTime );
}*/
 
/*TimerTime_t TimerGetElapsedTime( TimerTime_t savedTime )
{
    uint64_t CurrentTime = xtimer_now_usec64();
    return ( TimerTime_t )( CurrentTime - savedTime );
}*/
 
/*TimerTime_t TimerGetFutureTime( TimerTime_t eventInFuture )
{
    uint64_t CurrentTime = xtimer_now_usec64();
    return ( TimerTime_t )( CurrentTime + eventInFuture );
}*/

void TimerLowPowerHandler(uint32_t time)
{
    if( HasLoopedThroughMain < 5 )
        {
            HasLoopedThroughMain++;
        }
        else
        {
            HasLoopedThroughMain = 0;
            xtimer_usleep (time*1000);
        }   
}
