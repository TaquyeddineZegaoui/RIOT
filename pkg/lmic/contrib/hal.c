/*
 * Copyright (c) 2014-2016 IBM Corporation.
 * All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions are met:
 *  * Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *  * Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *  * Neither the name of the <organization> nor the
 *    names of its contributors may be used to endorse or promote products
 *    derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL <COPYRIGHT HOLDER> BE LIABLE FOR ANY
 * DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include "lmic.h"
#include "board.h"
#include "periph/spi.h"
#include "periph/gpio.h"
#include "stm32l1xx.h"
#include <stdio.h>
#include <string.h>

// -----------------------------------------------------------------------------
// I/O

#define SX1276_DIO0 GPIO_PIN(PORT_B, 0)
#define SX1276_DIO1 GPIO_PIN(PORT_B, 1)
#define SX1276_DIO2 GPIO_PIN(PORT_C, 6)

#define SX1276_RESET GPIO_PIN(PORT_A, 9)

/** SX1276 SPI */

#define USE_SPI_0

#ifdef USE_SPI_1
#define SX1276_SPI SPI_1
#define SX1276_SPI_NSS GPIO_PIN(PORT_C, 8)
#define SX1276_SPI_MODE SPI_CONF_FIRST_RISING
#define SX1276_SPI_SPEED SPI_SPEED_1MHZ
#endif

#ifdef USE_SPI_0
#define SX1276_SPI SPI_0
#define SX1276_SPI_NSS GPIO_PIN(PORT_C, 8)
#define SX1276_SPI_MODE SPI_CONF_FIRST_RISING
#define SX1276_SPI_SPEED SPI_SPEED_1MHZ
#endif

// HAL state
static struct {
    unsigned int cpsr;
    u4_t ticks;
} HAL;

/**
 * IRQ handlers
 */
void sx1276_on_dio0_isr(void *arg)
{
 // invoke radio handler (on IRQ!)
        radio_irq_handler(0);
}

void sx1276_on_dio1_isr(void *arg)
{
  // invoke radio handler (on IRQ!)
        radio_irq_handler(1);
}

void sx1276_on_dio2_isr(void *arg)
{
 // invoke radio handler (on IRQ!)
        radio_irq_handler(2);
}


// -----------------------------------------------------------------------------
// I/O

static void hal_io_init (void) {
    gpio_init_int(SX1276_DIO0, GPIO_IN, GPIO_RISING, sx1276_on_dio0_isr, NULL);
    gpio_init_int(SX1276_DIO1, GPIO_IN, GPIO_RISING, sx1276_on_dio1_isr, NULL);
    gpio_init_int(SX1276_DIO2, GPIO_IN, GPIO_RISING, sx1276_on_dio2_isr, NULL);
}

// set radio NSS pin to given value
void hal_pin_nss (u1_t val) {
    gpio_write (SX1276_SPI_NSS, val);
}

// set radio RST pin to given value (or keep floating!)
void hal_pin_rst (u1_t val) {
    if(val == 0 || val == 1) { // drive pin
        gpio_init(SX1276_RESET, GPIO_OUT);
        gpio_write (SX1276_RESET, val);
    } else { // keep pin floating
        gpio_init(SX1276_RESET, GPIO_OD);
        gpio_set(SX1276_RESET);
    }
}

extern void radio_irq_handler(u1_t dio);

// -----------------------------------------------------------------------------
// SPI

// for sx1272 and 1276

static void hal_spi_init (void) {
    int res;

    /* Setup SPI for SX1276 */
    spi_acquire(SX1276_SPI);
    res = spi_init_master(SX1276_SPI, SPI_CONF_FIRST_RISING, SPI_SPEED_1MHZ);
    spi_release(SX1276_SPI);

    if (res < 0) {
        printf("sx1276: error initializing SPI_%i device (code %i)\n",
                SX1276_SPI, res);
        return;
    }

    res = gpio_init(SX1276_SPI_NSS, GPIO_OUT);
    if (res < 0) {
        printf("sx1276: error initializing GPIO_%ld as CS line (code %i)\n",
               (long)SX1276_SPI_NSS, res);
        return;
    }

    gpio_set(SX1276_SPI_NSS);
}

// perform SPI transaction with radio
u1_t hal_spi (u1_t out) {
    char in;
    spi_transfer_byte(SX1276_SPI, (char) out, &in);
    return (u1_t) in; // in
}

#ifdef CFG_lmic_clib

// -----------------------------------------------------------------------------
// TIME

static void hal_time_init (void) {
#ifndef CFG_clock_HSE
    PWR->CR |= PWR_CR_DBP; // disable write protect
    RCC->CSR |= RCC_CSR_LSEON; // switch on low-speed oscillator @32.768kHz
    while( (RCC->CSR & RCC_CSR_LSERDY) == 0 ); // wait for it...
#endif
    
    RCC->APB2ENR   |= RCC_APB2ENR_TIM9EN;     // enable clock to TIM9 peripheral 
    RCC->APB2LPENR |= RCC_APB2LPENR_TIM9LPEN; // enable clock to TIM9 peripheral also in low power mode
    RCC->APB2RSTR  |= RCC_APB2RSTR_TIM9RST;   // reset TIM9 interface
    RCC->APB2RSTR  &= ~RCC_APB2RSTR_TIM9RST;  // reset TIM9 interface

    TIM9->PSC  = (640 - 1); // HSE_CLOCK_HWTIMER_PSC-1);  XXX: define HSE_CLOCK_HWTIMER_PSC somewhere

    NVIC->IP[TIM9_IRQn] = 0x70; // interrupt priority
    NVIC->ISER[TIM9_IRQn>>5] = 1<<(TIM9_IRQn&0x1F);  // set enable IRQ

    // enable update (overflow) interrupt
    TIM9->DIER |= TIM_DIER_UIE;
    
    // Enable timer counting
    TIM9->CR1 = TIM_CR1_CEN;
}

u4_t hal_ticks (void) {
    hal_disableIRQs();
    u4_t t = HAL.ticks;
    u2_t cnt = TIM9->CNT;
    if( (TIM9->SR & TIM_SR_UIF) ) {
        // Overflow before we read CNT?
        // Include overflow in evaluation but
        // leave update of state to ISR once interrupts enabled again
        cnt = TIM9->CNT;
        t++;
    }
    hal_enableIRQs();
    return (t<<16)|cnt;
}

// return modified delta ticks from now to specified ticktime (0 for past, FFFF for far future)
static u2_t deltaticks (u4_t time) {
    u4_t t = hal_ticks();
    s4_t d = time - t;
    if( d<=0 ) return 0;    // in the past
    if( (d>>16)!=0 ) return 0xFFFF; // far ahead
    return (u2_t)d;
}

void hal_waitUntil (u4_t time) {
    while( deltaticks(time) != 0 ); // busy wait until timestamp is reached
}

// check and rewind for target time
u1_t hal_checkTimer (u4_t time) {
    u2_t dt;
    TIM9->SR &= ~TIM_SR_CC2IF; // clear any pending interrupts
    if((dt = deltaticks(time)) < 5) { // event is now (a few ticks ahead)
        TIM9->DIER &= ~TIM_DIER_CC2IE; // disable IE
        return 1;
    } else { // rewind timer (fully or to exact time))
        TIM9->CCR[2] = TIM9->CNT + dt;   // set comparator
        TIM9->DIER |= TIM_DIER_CC2IE;  // enable IE
        TIM9->CCER |= TIM_CCER_CC2E;   // enable capture/compare uint 2
        return 0;
    }
}
  
void TIM9_IRQHandler (void) {
    if(TIM9->SR & TIM_SR_UIF) { // overflow
        HAL.ticks++;
    }
    if((TIM9->SR & TIM_SR_CC2IF) && (TIM9->DIER & TIM_DIER_CC2IE)) { // expired
        // do nothing, only wake up cpu
    }
    TIM9->SR = 0; // clear IRQ flags
}

// -----------------------------------------------------------------------------
// IRQ

void hal_disableIRQs (void) {
    HAL.cpsr = irq_disable();
}

void hal_enableIRQs (void) {
    irq_restore(HAL.cpsr);
}

void hal_sleep (void) {
    // low power sleep mode
#ifndef CFG_no_low_power_sleep_mode
    PWR->CR |= PWR_CR_LPSDSR;
#endif
    // suspend execution until IRQ, regardless of the CPSR I-bit
    __WFI();
}

// -----------------------------------------------------------------------------

void hal_init (void) {
    memset(&HAL, 0x00, sizeof(HAL));
    hal_disableIRQs();

    // configure radio I/O and interrupt handler
    hal_io_init();
    // configure radio SPI
    hal_spi_init();
    // configure timer and interrupt handler
    hal_time_init();

    hal_enableIRQs();
}

void hal_failed (void) {
    // HALT...
    hal_disableIRQs();
    hal_sleep();
    while(1);
}

#endif // CFG_lmic_clib
