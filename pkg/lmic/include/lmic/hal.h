/*******************************************************************************
 * Copyright (c) 2015 Matthijs Kooijman
 * All rights reserved. This program and the accompanying materials
 * are made available under the terms of the Eclipse Public License v1.0
 * which accompanies this distribution, and is available at
 * http://www.eclipse.org/legal/epl-v10.html
 *
 * This the HAL to run LMIC on top of the Arduino environment.
 *******************************************************************************/
#ifndef _hal_hal_h_
#define _hal_hal_h_
#include "periph/gpio.h"

#define NUMBER_OF_DIOS 3
static const int NUM_DIO = NUMBER_OF_DIOS;

typedef struct lmic_pinmap_t {
    gpio_t nss;
    gpio_t rxtx;
    gpio_t rst;
    gpio_t dio[NUMBER_OF_DIOS];
} lmic_pinmap;

// Declared here, to be defined an initialized by the application
extern const lmic_pinmap lmic_pins;

#endif // _hal_hal_h_
