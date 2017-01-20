/*
 * Copyright (C) 2015 Freie Universität Berlin
 * Copyright (C) 2015 Hamburg University of Applied Sciences
 *
 * This file is subject to the terms and conditions of the GNU Lesser General
 * Public License v2.1. See the file LICENSE in the top level directory for more
 * details.
 */

/**
 * @ingroup     cpu_stm32l1
 * @{
 *
 * @file
 * @brief       Low-level GPIO driver implementation
 *
 * @author      Hauke Petersen <hauke.petersen@fu-berlin.de>
 * @author      Katja Kirstein <katja.kirstein@haw-hamburg.de>
 *
 * @}
 */

#include "cpu.h"
#include "periph/gpio.h"
#include "periph_conf.h"

/**
 * @brief   Number of available external interrupt lines
 */
#define GPIO_ISR_CHAN_NUMOF             (16U)

static uint32_t tmpreg;

/**
 * @brief   Hold one callback function pointer for each interrupt line
 */
static gpio_isr_ctx_t exti_chan[GPIO_ISR_CHAN_NUMOF];

/**
 * @brief   Extract the port base address from the given pin identifier
 */
static inline GPIO_TypeDef *_port(gpio_t pin)
{
    return (GPIO_TypeDef *)(pin & ~(0x0f));
}

/**
 * @brief   Extract the port number form the given identifier
 *
 * The port number is extracted by looking at bits 10, 11, 12, 13 of the base
 * register addresses.
 */
static inline int _port_num(gpio_t pin)
{
    return ((pin >> 10) & 0x0f);
}

/**
 * @brief   Extract the pin number from the last 4 bit of the pin identifier
 */
static inline int _pin_num(gpio_t pin)
{
    return (pin & 0x0f);
}

int gpio_init(gpio_t pin, gpio_mode_t mode)
{
    GPIO_TypeDef *port = _port(pin);
    int pin_num = _pin_num(pin);

    /* enable clock */
    periph_clk_en(AHB, (RCC_AHBENR_GPIOAEN << _port_num(pin)));

    /* set mode */
    tmpreg = port->MODER;
    tmpreg &= ~(0x3 << (2 * pin_num));
    tmpreg |=  ((mode & 0x3) << (2 * pin_num));
    port->MODER = tmpreg;
    /* set pull resistor configuration */
    tmpreg = port->PUPDR;
    tmpreg &= ~(0x3 << (2 * pin_num));
    tmpreg |=  (((mode >> 2) & 0x3) << (2 * pin_num));
    port->PUPDR = tmpreg;
    /* set output mode */
    tmpreg = port->OTYPER;
    tmpreg &= ~(1 << pin_num);
    tmpreg |=  (((mode >> 4) & 0x1) << pin_num);
    port->OTYPER = tmpreg;
    /* finally set pin speed to maximum and reset output */
    port->OSPEEDR |= (3 << (2 * pin_num));
#if defined (STM32L1XX_HD) || defined (STM32L1XX_XL)
    port->BRR = (1 << pin_num);
#endif
    return 0;
}

int gpio_init_int(gpio_t pin, gpio_mode_t mode, gpio_flank_t flank,
                   gpio_cb_t cb, void *arg)
{
    int pin_num = _pin_num(pin);
    int port_num = _port_num(pin);

    /* configure and save exti configuration struct */
    exti_chan[pin_num].cb = cb;
    exti_chan[pin_num].arg = arg;
    /* enable the SYSCFG clock */
    periph_clk_en(APB2, RCC_APB2ENR_SYSCFGEN);
    /* initialize pin as input */
    gpio_init(pin, mode);
    /* enable global pin interrupt */
    if (pin_num < 5) {
        NVIC_EnableIRQ(EXTI0_IRQn + pin_num);
    }
    else if (pin_num < 10) {
        NVIC_EnableIRQ(EXTI9_5_IRQn);
    }
    else {
        NVIC_EnableIRQ(EXTI15_10_IRQn);
    }
    /* configure the active edge(s) */
    switch (flank) {
        case GPIO_RISING:
            EXTI->RTSR |= (1 << pin_num);
            EXTI->FTSR &= ~(1 << pin_num);
            break;
        case GPIO_FALLING:
            EXTI->RTSR &= ~(1 << pin_num);
            EXTI->FTSR |= (1 << pin_num);
            break;
        case GPIO_BOTH:
            EXTI->RTSR |= (1 << pin_num);
            EXTI->FTSR |= (1 << pin_num);
            break;
    }
    /* enable specific pin as exti sources */
    tmpreg = SYSCFG->EXTICR[pin_num >> 2];
    tmpreg &= ~(0xf << ((pin_num & 0x03) * 4));
    tmpreg |= (port_num << ((pin_num & 0x03) * 4));
    SYSCFG->EXTICR[pin_num >> 2] = tmpreg;
    /* clear any pending requests */
    EXTI->PR = (1 << pin_num);
    /* enable interrupt for EXTI line */
    EXTI->IMR |= (1 << pin_num);
    return 0;
}

void gpio_init_af(gpio_t pin, gpio_af_t af)
{
    GPIO_TypeDef *port = _port(pin);
    uint32_t pin_num = _pin_num(pin);

    /* set pin to AF mode */
    tmpreg = port->MODER;
    tmpreg &= ~(3 << (2 * pin_num));
    tmpreg |= (2 << (2 * pin_num));
    port->MODER = tmpreg;
    /* set selected function */
    tmpreg = port->AFR[(pin_num > 7) ? 1 : 0];
    tmpreg &= ~(0xf << ((pin_num & 0x07) * 4));
    tmpreg |= (af << ((pin_num & 0x07) * 4));
    port->AFR[(pin_num > 7) ? 1 : 0] = tmpreg;
}

void gpio_init_analog(gpio_t pin)
{
    /* enable clock, needed as this function can be used without calling
     * gpio_init first */
    periph_clk_en(AHB, (RCC_AHBENR_GPIOAEN << _port_num(pin)));
    /* set to analog mode */
    _port(pin)->MODER |= (0x3 << (2 * _pin_num(pin)));
}

void gpio_irq_enable(gpio_t pin)
{
    int pin_num = _pin_num(pin);
    int port_num = _port_num(pin);
    
    /* check if IRQ is actually configured for this pin and port */
    if (!((SYSCFG->EXTICR[pin_num >> 2] & (0xf << ((pin_num & 0x03) * 4))) ^ (port_num << ((pin_num & 0x03) * 4)))) {
        EXTI->IMR |= (1 << _pin_num(pin));
    }
}

void gpio_irq_disable(gpio_t pin)
{
    int pin_num = _pin_num(pin);
    int port_num = _port_num(pin);
    
    /* check if IRQ is actually configured for this pin and port */
    if (!((SYSCFG->EXTICR[pin_num >> 2] & (0xf << ((pin_num & 0x03) * 4))) ^ (port_num << ((pin_num & 0x03) * 4)))) {
        EXTI->IMR &= ~(1 << pin_num);
    }
}

int gpio_read(gpio_t pin)
{
    GPIO_TypeDef *port = _port(pin);
    uint32_t pin_num = _pin_num(pin);

    uint8_t port_mode = (port->MODER & (3 << (pin_num * 2))) >> (pin_num * 2);
    
    if (port_mode == 1) {   /* if configured as output */
        return (port->ODR & (1 << pin_num)) >> pin_num;      /* read output data reg */
    }
    if (port_mode == 0) {
        return (port->IDR & (1 << pin_num)) >> pin_num;      /* else read input data reg */
    }
    
    /* configured as AF or AIN */
    return -1;
}

void gpio_set(gpio_t pin)
{
    _port(pin)->BSRRL = (1 << _pin_num(pin));
}

void gpio_clear(gpio_t pin)
{
    _port(pin)->BSRRH = (1 << _pin_num(pin));
}

void gpio_toggle(gpio_t pin)
{
    if (gpio_read(pin)) {
        gpio_clear(pin);
    }
    else {
        gpio_set(pin);
    }
}

void gpio_write(gpio_t pin, int value)
{
    if (value) {
        gpio_set(pin);
    }
    else {
        gpio_clear(pin);
    }
}

void isr_exti(void)
{
    /* only generate interrupts against lines which have their IMR set */
    uint32_t pending_isr = (EXTI->PR & EXTI->IMR);
    for (int i = 0; i < GPIO_ISR_CHAN_NUMOF; i++) {
        if (pending_isr & (1 << i)) {
            EXTI->PR = (1 << i);                /* clear by writing a 1 */
            exti_chan[i].cb(exti_chan[i].arg);
        }
    }
    cortexm_isr_end();
}