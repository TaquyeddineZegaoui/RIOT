/*
 / _____)             _              | |
( (____  _____ ____ _| |_ _____  ____| |__
 \____ \| ___ |    (_   _) ___ |/ ___)  _ \
 _____) ) ____| | | || |_| ____( (___| | | |
(______/|_____)_|_|_| \__)_____)\____)_| |_|
    (C)2013 Semtech

Description: Timer devects and scheduling management

License: Revised BSD License, see LICENSE.TXT file include in the project

Maintainer: Miguel Luis and Gregory Cristian
*/
#ifndef __TIMER_H__
#define __TIMER_H__

#include "xtimer.h"

/*!
 * \brief Timer time variable definition
 */
#ifndef TimerTime_t
typedef uint64_t TimerTime_t;
#endif

/*!
 * \brief Initializes the timer devect
 *
 * \remark TimerSetValue function must be called before starting the timer.
 *         this function initializes timestamp and reload value at 0.
 *
 * \param [IN] dev          Structure containing the timer devect parameters
 * \param [IN] callback     Function callback called at the end of the timeout
 */
void TimerInit( xtimer_t *dev, xtimer_callback_t cb);

/*!
 * Timer IRQ event handler
 */
void TimerIrqHandler( void );

/*!
 * \brief Starts and adds the timer devect to the list of timer events
 *
 * \param [IN] dev Structure containing the timer devect parameters
 */
void TimerStart( xtimer_t *dev );

/*!
 * \brief Stops and removes the timer devect from the list of timer events
 *
 * \param [IN] dev Structure containing the timer devect parameters
 */
void TimerStop( xtimer_t *dev );

/*!
 * \brief Resets the timer devect
 *
 * \param [IN] dev Structure containing the timer devect parameters
 */
void TimerReset( xtimer_t *dev );

/*!
 * \brief Set timer new timeout value
 *
 * \param [IN] dev   Structure containing the timer devect parameters
 * \param [IN] value New timer timeout value
 */
void TimerSetValue( xtimer_t *dev, uint32_t value );

/*!
 * \brief Read the current time
 *
 * \retval time returns current time
 */
TimerTime_t TimerGetCurrentTime( void );

/*!
 * \brief Return the Time elapsed since a fix moment in Time
 *
 * \param [IN] savedTime    fix moment in Time
 * \retval time             returns elapsed time
 */
TimerTime_t TimerGetElapsedTime( TimerTime_t savedTime );

/*!
 * \brief Return the Time elapsed since a fix moment in Time
 *
 * \param [IN] eventInFuture    fix moment in the future
 * \retval time             returns difference between now and future event
 */
TimerTime_t TimerGetFutureTime( TimerTime_t eventInFuture );

/*!
 * \brief Manages the entry into ARM cortex deep-sleep mode
 */
void TimerLowPowerHandler( void );

#endif  // __TIMER_H__
