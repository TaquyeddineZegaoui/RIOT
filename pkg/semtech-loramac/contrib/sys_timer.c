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

#include "loramac/board_definitions.h"
#include "xtimer.h"


void TimerInit( TimerEvent_t *obj, void ( *callback )( void ) )
{
    obj->dev->target = 0;
    obj->dev->callback = callback;
}

void TimerReset( TimerEvent_t *obj )
{
    TimerStop();
    TimerStart();
}
 
void TimerStart( TimerEvent_t *obj )
{
    xtimer_set(obj->dev, obj->timeout);
}
 
void TimerStop( TimerEvent_t *obj )
{
    xtimer_remove(obj->dev);
}
 
void TimerSetValue( TimerEvent_t *obj, uint32_t value )
{
    xtimer_remove(obj->dev);
    obj->timeout = value;
}

TimerTime_t TimerGetCurrentTime( void )
{
    uint64_t CurrentTime += xtimer_now();
    return ( ( TimerTime_t )CurrentTime );
}
 
TimerTime_t TimerGetElapsedTime( TimerTime_t savedTime )
{
    uint64_t CurrentTime += xtimer_now();
    return ( TimerTime_t )( CurrentTime - savedTime );
}
 
TimerTime_t TimerGetFutureTime( TimerTime_t eventInFuture )
{
    uint64_t CurrentTime += xtimer_now();
    return ( TimerTime_t )( CurrentTime + eventInFuture );
}
 
