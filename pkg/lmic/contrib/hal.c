/*******************************************************************************
 * Copyright (c) 2015 Matthijs Kooijman
 * All rights reserved. This program and the accompanying materials
 * are made available under the terms of the Eclipse Public License v1.0
 * which accompanies this distribution, and is available at
 * http://www.eclipse.org/legal/epl-v10.html
 *
 * This the HAL to run LMIC on top of the Arduino environment.
 *******************************************************************************/

#include "lmic.h"
#include "hal.h"
#include "lmic/hal.h"
#include <stdio.h>
#include "xtimer.h"
#include "periph/gpio.h"
#include "periph/spi.h"

extern const lmic_pinmap lmic_pins;
static void hal_io_init (void) {
    // NSS and DIO0 are required, DIO1 is required for LoRa, DIO2 for FSK
/*
    ASSERT(lmic_pins.nss != LMIC_UNUSED_PIN);
    ASSERT(lmic_pins.dio[0] != LMIC_UNUSED_PIN);
    ASSERT(lmic_pins.dio[1] != LMIC_UNUSED_PIN || lmic_pins.dio[2] != LMIC_UNUSED_PIN);

    pinMode(lmic_pins.nss, OUTPUT);
    if (lmic_pins.rxtx != LMIC_UNUSED_PIN)
        pinMode(lmic_pins.rxtx, OUTPUT);
    if (lmic_pins.rst != LMIC_UNUSED_PIN)
        pinMode(lmic_pins.rst, OUTPUT);

    pinMode(lmic_pins.dio[0], INPUT);
    if (lmic_pins.dio[1] != LMIC_UNUSED_PIN)
        pinMode(lmic_pins.dio[1], INPUT);
    if (lmic_pins.dio[2] != LMIC_UNUSED_PIN)
        pinMode(lmic_pins.dio[2], INPUT);
*/
    
    gpio_init(lmic_pins.nss, GPIO_OUT);
    gpio_init(lmic_pins.dio[0], GPIO_IN);
    gpio_init(lmic_pins.dio[1], GPIO_IN);
    gpio_init(lmic_pins.rst, GPIO_OUT);
}

// nothing to do here
void hal_pin_rxtx (u1_t val) {
}

// set radio RST pin to given value (or keep floating!)
void hal_pin_rst (u1_t val) {
    if(val == 0 || val == 1) { // drive pin
        gpio_init(lmic_pins.rst, GPIO_OUT);
        gpio_write(lmic_pins.rst, val);
    } else { // keep pin floating
        gpio_init(lmic_pins.rst, GPIO_OUT);
        gpio_set(lmic_pins.rst);
    }
}

static bool dio_states[NUMBER_OF_DIOS] = {0};

static void hal_io_check(void) {
    uint8_t i;
    for (i = 0; i < NUM_DIO; ++i) {
        if (dio_states[i] != (gpio_read(lmic_pins.dio[i]) > 0)) {
            dio_states[i] = !dio_states[i];
            if (dio_states[i])
                radio_irq_handler(i);
        }
    }
}

// -----------------------------------------------------------------------------
// SPI

static void hal_spi_init (void) {
    spi_acquire(SPI_0);
    spi_init_master(SPI_0, SPI_CONF_FIRST_RISING, SPI_SPEED_1MHZ);
    spi_release(SPI_0);
}

void hal_pin_nss (u1_t val) {
    gpio_write(lmic_pins.nss, val);
}

// perform SPI transaction with radio
u1_t hal_spi (u1_t out) {
    char in;
    spi_transfer_byte(SPI_0, out, &in);
    u1_t res = in;
    return res;
}

// -----------------------------------------------------------------------------
// TIME

static void hal_time_init (void) {
    // Nothing to do
}

u4_t hal_ticks (void) {
    return (xtimer_now()).ticks32 >> 4;
}

// Returns the number of ticks until time. Negative values indicate that
// time has already passed.
static s4_t delta_time(u4_t time) {
    return (s4_t)(time - hal_ticks());
}

void hal_waitUntil (u4_t time) {
    s4_t delta = delta_time(time);
    xtimer_ticks32_t ticks;
    ticks.ticks32 = delta << 4;
    xtimer_usleep(xtimer_usec_from_ticks(ticks));
}

// check and rewind for target time
u1_t hal_checkTimer (u4_t time) {
    // No need to schedule wakeup, since we're not sleeping
    return delta_time(time) <= 0;
}

static uint8_t irqlevel = 0;

void hal_disableIRQs (void) {
    //noInterrupts();
    irqlevel++;
}

void hal_enableIRQs (void) {
    if(--irqlevel == 0) {
        //interrupts();

        // Instead of using proper interrupts (which are a bit tricky
        // and/or not available on all pins on AVR), just poll the pin
        // values. Since os_runloop disables and re-enables interrupts,
        // putting this here makes sure we check at least once every
        // loop.
        //
        // As an additional bonus, this prevents the can of worms that
        // we would otherwise get for running SPI transfers inside ISRs
        hal_io_check();
    }
}

void hal_sleep (void) {
    // Not implemented
}

// -----------------------------------------------------------------------------

#if defined(LMIC_PRINTF_TO)
static int uart_putchar (char c, FILE *)
{
    LMIC_PRINTF_TO.write(c) ;
    return 0 ;
}

void hal_printf_init(void) {
    // create a FILE structure to reference our UART output function
    static FILE uartout;
    memset(&uartout, 0, sizeof(uartout));

    // fill in the UART file descriptor with pointer to writer.
    fdev_setup_stream (&uartout, uart_putchar, NULL, _FDEV_SETUP_WRITE);

    // The uart is the standard output device STDOUT.
    stdout = &uartout ;
}
#endif // defined(LMIC_PRINTF_TO)

void hal_init (void) {
    // configure radio I/O and interrupt handler
    hal_io_init();
    // configure radio SPI
    hal_spi_init();
    // configure timer and interrupt handler
    hal_time_init();
#if defined(LMIC_PRINTF_TO)
    // printf support
    hal_printf_init();
#endif
}

void hal_failed (void) {
#if defined(LMIC_FAILURE_TO)
    LMIC_FAILURE_TO.println("FAILURE ");
    LMIC_FAILURE_TO.print(file);
    LMIC_FAILURE_TO.print(':');
    LMIC_FAILURE_TO.println(line);
    LMIC_FAILURE_TO.flush();
#endif
    hal_disableIRQs();
    while(1);
}
