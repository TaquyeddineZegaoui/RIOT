/*
 * Copyright (C) Unwired Devices [info@unwds.com]
 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 */

/**
 * @ingroup     tests
 * @{
 *
 * @file
 * @brief   Common header for sx1276 tests
 *
 * @author  Eugene P. [ep@unwds.com]
 */
#ifndef COMMON_H_
#define COMMON_H_

#include <stdint.h>

#include "sx1276.h"

#ifdef __cplusplus
extern "C" {
#endif


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

sx1276_t sx1276;

void init_radio(void);

/**
 * @brief   Application-internal functions and variables for sx1276 tests
 * @internal
 * @{
 */

int test1(int argc, char **argv);
int test2(int argc, char **argv);
/**
 * @}
 */

#ifdef __cplusplus
}
#endif

#endif /* COMMON_H_ */
/** @} */
