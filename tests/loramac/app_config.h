/*
 * Copyright (C) 2016 Fundacion Inria Chile
 *
 * This file is subject to the terms and conditions of the GNU Lesser General
 * Public License v2.1. See the file LICENSE in the top level directory for more
 * details.
 */

/* @file        app_config.h
 * @brief       application specific definitions, Makefile specified
 *
 * @author      Francisco Molina <francisco.molina@inria.cl>
*/

#ifndef __DROP_WATCHER_CONFIG_H__
#define __DROP_WATCHER_CONFIG_H__

/*
* amount of time app measures drop's
*/
#ifndef DROP_TIME
    #define DROP_TIME                                   (4*SEC_IN_USEC)
#endif

/*!
 * Join requests trials duty cycle.
 */
#ifndef OVER_THE_AIR_ACTIVATION_DUTYCYCLE_SEC
    #define OVER_THE_AIR_ACTIVATION_DUTYCYCLE_SEC           10 // 10 [s]
#endif
#ifndef OVER_THE_AIR_ACTIVATION_DUTYCYCLE_MIN
    #define OVER_THE_AIR_ACTIVATION_DUTYCYCLE_MIN           0 // 0 [min]
#endif

/*!
 * Defines the application data transmission duty cycle. 5s, value in [min].
 */
#ifndef APP_TX_DUTYCYCLE_SEC
    #define APP_TX_DUTYCYCLE_SEC                            10
#endif
#ifndef APP_TX_DUTYCYCLE_MIN
    #define APP_TX_DUTYCYCLE_MIN                            0
#endif
#ifndef APP_TX_DUTYCYCLE_HOUR
    #define APP_TX_DUTYCYCLE_HOUR                           0
#endif


/*!
 * Defines a random delay for application data transmission duty cycle. 1s,
 * value in [ms].
 */
#define APP_TX_DUTYCYCLE_RND                            1

/*!
 * LoRaWAN confirmed messages
 */
#ifndef LORAWAN_CONFIRMED_MSG_ON 
    #define LORAWAN_CONFIRMED_MSG_ON                    false
#endif

/*!
 * LoRaWAN Adaptive Data Rate
 *
 * \remark Please note that when ADR is enabled the end-device should be static
 */
#ifndef LORAWAN_ADR_ON 
    #define LORAWAN_ADR_ON                              0
#endif

/*!
 * LoRaWAN application port
 */
#ifndef LORAWAN_APP_PORT 
    #define LORAWAN_APP_PORT                            2
#endif

/*!
 * Enumeration appi status
 */
typedef enum drop_watcher_status
{
    /*!
     * Service performed successfully
     */
    APP_OK = 0xAA,
    /*!
     * An error occured during the drop count
     */
    COUNT_FAIL = 0xFF,

}drop_watcher_status_t;

#endif // __DROP_WATCHER_CONFIG_H__