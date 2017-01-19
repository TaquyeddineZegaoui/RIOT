/*
 * Copyright (C) 2016 Fundacion Inria Chile
 *
 * This file is subject to the terms and conditions of the GNU Lesser General
 * Public License v2.1. See the file LICENSE in the top level directory for more
 * details.
 */

/* @file        boards_hw.c
 * @brief       Specific board headers
 *
 * @author      Francisco Molina <francisco.molina@inria.cl>
*/

#ifndef __BOARDS_HW_H__
#define __BOARDS_HW_H__


#include "random.h"
#include "periph/gpio.h"
#include "periph/rtc.h"
#include "periph/adc.h"
#include "net/lorawan/board_definitions.h" 

#include "driver_config.h"
#include "infrared.h"

#ifdef NZ32_SC151

#define TM_YEAR_OFFSET              1900

/*!
 * Battery level ratio (battery dependent)
 */

#define BAT_LEVEL           GPIO_PIN(PORT_A, 13)
#define USB_DETECT          GPIO_PIN(PORT_C, 4)
#define ADC_RESOLUTION      ADC_RES_12BIT
#define ADC_VREF			ADC_LINE(4)
#define ADC_VDIV			ADC_LINE(3)

/*!
 * Factory power supply
 */
#define FACTORY_POWER_SUPPLY                        3.0L

/*!
 * VREF calibration value
 */
#define VREFINT_CAL                                 ( *( uint16_t* )0x1FF800F8 )// /4 to go form 12 to 10 bits

/*!
 * ADC maximum value
 */
#define ADC_MAX_VALUE                               4096

/*!
 * Battery thresholds
 */
#define BATTERY_MAX_LEVEL                           4150 // mV
#define BATTERY_MIN_LEVEL                           3200 // mV
#define BATTERY_SHUTDOWN_LEVEL                      3100 // mV

/*!
 * Unique Devices IDs register set ( STM32L1xxx )
 */
#define         ID1                                 ( 0x1FF800D0 )
#define         ID2                                 ( 0x1FF800D4 )
#define         ID3                                 ( 0x1FF800E4 )

/*!
* Type of power source for device
*/
enum BoardPowerSource
{
    USB_POWER = 0,
    BATTERY_POWER
};

#endif

/**
 * @brief Measures board battery voltage
 *
 * @return                   battery voltage in mV
 */
uint16_t board_get_power_supply( void );

/**
 * @brief Measures battery charge level
 *
 * @return                   battery charge level, min 1, max 254
 */
uint8_t board_get_battery_level( void );

/**
 * @brief Checks for device power source
 *
 * @return                   BATTERY_POWER or USB_POWER
 */
uint8_t get_board_power_source( void );

/**
 * @brief Calaculates random seed from Unique device ID
 *
 * @return                   uint32_t random number seed
 */
uint32_t board_get_random_seed( void );

/**
 * @brief Get's board unique id from nz32-sc151 unique id register's
 *
 * @param[in] id             pointers to buffer that will contain the DevEUI
 */
void board_get_unique_id( uint8_t *id );

/**
 * @brief Generates a 4 byte node_id from DevEUI using hash md5
 *
 * @param[in] id             DevEUI or identifier to calculate hash
 *            id_length      length of the identifier
 *            node_id        pointer to array to contain node_id
 */
void board_get_node_id( uint8_t *id , uint8_t id_length, uint8_t *node_id );

/**
 * @brief Init's ADC anf GPIO peripherials
 */
void board_init_periph( void );

/**
 * @brief Checks if time structure adjustes to range, if not it overflows to next
 *        time unit.
 *
 * @param[in] time         time structure
 */
void time_check(struct tm* time);

/**
 * @brief Start's the rtc timer and lora device.
 *
 * @param[in] dev         sx1276 object pointer
 */
void app_init (sx1276_t* dev);
#endif //__BOARDS_HW_H_