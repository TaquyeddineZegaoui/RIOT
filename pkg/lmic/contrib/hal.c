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

// -----------------------------------------------------------------------------
// I/O

#define CFG_eu868 1
//#define CFG_us915 1
// This is the SX1272/SX1273 radio, which is also used on the HopeRF
// RFM92 boards.
//#define CFG_sx1272_radio 1
// This is the SX1276/SX1277/SX1278/SX1279 radio, which is also used on
// the HopeRF RFM95 boards.
#define CFG_sx1276_radio 1

// 16 μs per tick
// LMIC requires ticks to be 15.5μs - 100 μs long
#define US_PER_OSTICK_EXPONENT 4
#define US_PER_OSTICK (1 << US_PER_OSTICK_EXPONENT)
#undef OSTICKS_PER_SEC
#define OSTICKS_PER_SEC (1000000 / US_PER_OSTICK)

// Set this to 1 to enable some basic debug output (using printf) about
// RF settings used during transmission and reception. Set to 2 to
// enable more verbose output. Make sure that printf is actually
// configured (e.g. on AVR it is not by default), otherwise using it can
// cause crashing.
#define LMIC_DEBUG_LEVEL 0

// Enable this to allow using printf() to print to the given serial port
// (or any other Print object). This can be easy for debugging. The
// current implementation only works on AVR, though.
//#define LMIC_PRINTF_TO Serial

// Any runtime assertion failures are printed to this serial port (or
// any other Print object). If this is unset, any failures just silently
// halt execution.
//#define LMIC_FAILURE_TO Serial

// Uncomment this to disable all code related to joining
//#define DISABLE_JOIN
// Uncomment this to disable all code related to ping
//#define DISABLE_PING
// Uncomment this to disable all code related to beacon tracking.
// Requires ping to be disabled too
//#define DISABLE_BEACONS

// Uncomment these to disable the corresponding MAC commands.
// Class A
//#define DISABLE_MCMD_DCAP_REQ // duty cycle cap
//#define DISABLE_MCMD_DN2P_SET // 2nd DN window param
//#define DISABLE_MCMD_SNCH_REQ // set new channel
// Class B
//#define DISABLE_MCMD_PING_SET // set ping freq, automatically disabled by DISABLE_PING
//#define DISABLE_MCMD_BCNI_ANS // next beacon start, automatical disabled by DISABLE_BEACON

// In LoRaWAN, a gateway applies I/Q inversion on TX, and nodes do the
// same on RX. This ensures that gateways can talk to nodes and vice
// versa, but gateways will not hear other gateways and nodes will not
// hear other nodes. By uncommenting this macro, this inversion is
// disabled and this node can hear other nodes. If two nodes both have
// this macro set, they can talk to each other (but they can no longer
// hear gateways). This should probably only be used when debugging
// and/or when talking to the radio directly (e.g. like in the "raw"
// example).
//#define DISABLE_INVERT_IQ_ON_RX

// This allows choosing between multiple included AES implementations.
// Make sure exactly one of these is uncommented.
//
// This selects the original AES implementation included LMIC. This
// implementation is optimized for speed on 32-bit processors using
// fairly big lookup tables, but it takes up big amounts of flash on the
// AVR architecture.
// #define USE_ORIGINAL_AES
//
// This selects the AES implementation written by Ideetroon for their
// own LoRaWAN library. It also uses lookup tables, but smaller
// byte-oriented ones, making it use a lot less flash space (but it is
// also about twice as slow as the original).
#define USE_IDEETRON_AES
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
    spi_acquire(SPI_1);
    spi_init_master(SPI_1, SPI_CONF_FIRST_RISING, SPI_SPEED_1MHZ);
    spi_release(SPI_1);
}

void hal_pin_nss (u1_t val) {
    gpio_write(lmic_pins.nss, val);
}

// perform SPI transaction with radio
u1_t hal_spi (u1_t out) {
    char in;
    spi_transfer_byte(SPI_1, out, &in);
    u1_t res = in;
    return res;
}

// -----------------------------------------------------------------------------
// TIME

static void hal_time_init (void) {
    // Nothing to do
}

u4_t hal_ticks (void) {
    // Because micros() is scaled down in this function, micros() will
    // overflow before the tick timer should, causing the tick timer to
    // miss a significant part of its values if not corrected. To fix
    // this, the "overflow" serves as an overflow area for the micros()
    // counter. It consists of three parts:
    //  - The US_PER_OSTICK upper bits are effectively an extension for
    //    the micros() counter and are added to the result of this
    //    function.
    //  - The next bit overlaps with the most significant bit of
    //    micros(). This is used to detect micros() overflows.
    //  - The remaining bits are always zero.
    //
    // By comparing the overlapping bit with the corresponding bit in
    // the micros() return value, overflows can be detected and the
    // upper bits are incremented. This is done using some clever
    // bitwise operations, to remove the need for comparisons and a
    // jumps, which should result in efficient code. By avoiding shifts
    // other than by multiples of 8 as much as possible, this is also
    // efficient on AVR (which only has 1-bit shifts).
    static uint8_t overflow = 0;

    // Scaled down timestamp. The top US_PER_OSTICK_EXPONENT bits are 0,
    // the others will be the lower bits of our return value.
    uint32_t scaled = xtimer_now_usec() >> US_PER_OSTICK_EXPONENT;
    // Most significant byte of scaled
    uint8_t msb = scaled >> 24;
    // Mask pointing to the overlapping bit in msb and overflow.
    const uint8_t mask = (1 << (7 - US_PER_OSTICK_EXPONENT));
    // Update overflow. If the overlapping bit is different
    // between overflow and msb, it is added to the stored value,
    // so the overlapping bit becomes equal again and, if it changed
    // from 1 to 0, the upper bits are incremented.
    overflow += (msb ^ overflow) & mask;

    // Return the scaled value with the upper bits of stored added. The
    // overlapping bit will be equal and the lower bits will be 0, so
    // bitwise or is a no-op for them.
    return scaled | ((uint32_t)overflow << 24);

    // 0 leads to correct, but overly complex code (it could just return
    // micros() unmodified), 8 leaves no room for the overlapping bit.
    //static_assert(US_PER_OSTICK_EXPONENT > 0 && US_PER_OSTICK_EXPONENT < 8, "Invalid US_PER_OSTICK_EXPONENT value");
}

// Returns the number of ticks until time. Negative values indicate that
// time has already passed.
static s4_t delta_time(u4_t time) {
    return (s4_t)(time - hal_ticks());
}

void hal_waitUntil (u4_t time) {
    s4_t delta = delta_time(time);
    // From delayMicroseconds docs: Currently, the largest value that
    // will produce an accurate delay is 16383.
    while (delta > (16000 / US_PER_OSTICK)) {
        xtimer_usleep(16*1000);
        delta -= (16000 / US_PER_OSTICK);
    }
    if (delta > 0)
        xtimer_usleep(delta * US_PER_OSTICK);
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
