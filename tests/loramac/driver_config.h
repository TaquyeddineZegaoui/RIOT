/*
 * Copyright (C) 2016 Fundacion Inria Chile
 *
 * This file is subject to the terms and conditions of the GNU Lesser General
 * Public License v2.1. See the file LICENSE in the top level directory for more
 * details.
 */

/* @file        driver_config.h
 * @brief       sx1276 driver port specification, compatible with nz32-sc151 and samr21-xpro
 *
 * @author      Francisco Molina <francisco.molina@inria.cl>
*/

#ifndef DRIVER_CONFIG_H_
#define DRIVER_CONFIG_H_

#include "sx1276.h"
#include "board.h"

#ifdef AT86RF2XX_PARAMS_BOARD
	#define SAMR21_XPRO
#else
	#define NZ32_SC151
#endif

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @name SX1276 configuration
 * @{
 */

#ifdef NZ32_SC151

#define SX1276_PARAM_DIO0 		GPIO_PIN(PORT_B, 0)
#define SX1276_PARAM_DIO1 		GPIO_PIN(PORT_B, 1)
#define SX1276_PARAM_DIO2 		GPIO_PIN(PORT_C, 6)
#define SX1276_PARAM_DIO3 		GPIO_PIN(PORT_A, 10)

#define SX1276_PARAM_RESET 		GPIO_PIN(PORT_A, 9)


/** SX1276 SPI */

#define USE_SPI_0

#ifdef USE_SPI_1
#define SX1276_PARAM_SPI SPI_1
#define SX1276_PARAM_SPI_NSS GPIO_PIN(PORT_C, 8)
#define SX1276_PARAM_SPI_MODE SPI_CONF_FIRST_RISING
#define SX1276_PARAM_SPI_SPEED SPI_SPEED_1MHZ
#endif

#ifdef USE_SPI_0
#define SX1276_PARAM_SPI SPI_0
#define SX1276_PARAM_SPI_NSS GPIO_PIN(PORT_C, 8)
#define SX1276_PARAM_SPI_MODE SPI_CONF_FIRST_RISING
#define SX1276_PARAM_SPI_SPEED SPI_SPEED_1MHZ
#endif

#endif

#ifdef SAMR21_XPRO

#define SX1276_PARAM_DIO0 GPIO_PIN(PA, 13)
#define SX1276_PARAM_DIO1 GPIO_PIN(PA, 7)
#define SX1276_PARAM_DIO2 GPIO_PIN(PA, 6)
#define SX1276_PARAM_DIO3 GPIO_PIN(PA, 18)

#define SX1276_PARAM_RESET GPIO_PIN(PA, 28)

/** SX1276 SPI */

#define USE_SPI_1

#ifdef USE_SPI_1
#define SX1276_PARAM_SPI SPI_1
#define SX1276_PARAM_SPI_NSS GPIO_PIN(PA, 19)
#define SX1276_PARAM_SPI_MODE SPI_CONF_FIRST_RISING
#define SX1276_PARAM_SPI_SPEED SPI_SPEED_1MHZ
#endif

#ifdef USE_SPI_0
#define SX1276_PARAM_SPI SPI_0
#define SX1276_PARAM_SPI_NSS GPIO_PIN(PA, 4)
#define SX1276_PARAM_SPI_MODE SPI_CONF_FIRST_RISING
#define SX1276_PARAM_SPI_SPEED SPI_SPEED_1MHZ
#endif

#endif


void init_radio(void);

#ifdef __cplusplus
}
#endif

#endif /* DRIVER_CONFIG_H_ */
/** @} */
