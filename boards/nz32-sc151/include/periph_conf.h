/*
 * Copyright (C) 2015 Hamburg University of Applied Sciences
 *
 * This file is subject to the terms and conditions of the GNU Lesser General
 * Public License v2.1. See the file LICENSE in the top level directory for more
 * details.
 */

/**
 * @ingroup     boards_nz32-sc151
 * @{
 *
 * @file
 * @brief       Peripheral MCU configuration for the limifrog-v1 board
 *
 * @author      Katja Kirstein <katja.kirstein@haw-hamburg.de>
 */

#ifndef PERIPH_CONF_H_
#define PERIPH_CONF_H_

#include "periph_cpu.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @name Clock system configuration
 * @{
 **/
#define CLOCK_HSI           (16000000U)         /* internal oscillator */
#define CLOCK_CORECLOCK     (32000000U)         /* desired core clock frequency */

/* configuration of PLL prescaler and multiply values */
/* CORECLOCK := HSI / CLOCK_PLL_DIV * CLOCK_PLL_MUL */
#define CLOCK_PLL_DIV       RCC_CFGR_PLLDIV2
#define CLOCK_PLL_MUL       RCC_CFGR_PLLMUL4
/* configuration of peripheral bus clock prescalers */
#define CLOCK_AHB_DIV       RCC_CFGR_HPRE_DIV1      /* AHB clock -> 32MHz */
#define CLOCK_APB2_DIV      RCC_CFGR_PPRE2_DIV1     /* APB2 clock -> 32MHz */
#define CLOCK_APB1_DIV      RCC_CFGR_PPRE1_DIV1     /* APB1 clock -> 32MHz */
/* configuration of flash access cycles */
#define CLOCK_FLASH_LATENCY FLASH_ACR_LATENCY

/* bus clocks for simplified peripheral initialization, UPDATE MANUALLY! */
#define CLOCK_AHB           (CLOCK_CORECLOCK / 1)
#define CLOCK_APB1          (CLOCK_CORECLOCK / 4)
#define CLOCK_APB2          (CLOCK_CORECLOCK / 2)  
/** @} */

/**
 * @brief   ADC configuration
 * @{
 */
#define ADC_CONFIG {            \
    { GPIO_PIN(PORT_A, 6), 6 },\
    { GPIO_PIN(PORT_A, 7), 7 },\
    { GPIO_PIN(PORT_A, 4), 4 },\
    { GPIO_PIN(PORT_B, 0), 8 },\
    { GPIO_PIN(PORT_B, 1), 9 },\
    { GPIO_PIN(PORT_B, 12), 18 }\
}

#define ADC_NUMOF           (6)
/** @} */

/**
 * @brief   DAC configuration
 * @{
 */
#define DAC_CONFIG {            \
    { GPIO_PIN(PORT_A, 4), 1},  \
    { GPIO_PIN(PORT_A, 5), 2},  \
}

#define DAC_NUMOF           (2)
/** @} */

/**
 * @name PWM configuration
 * @{
 */
#define PWM_NUMOF         (PWM_0_EN )
#define PWM_0_EN          1


static const pwm_conf_t pwm_config[PWM_NUMOF] = {
    {
        .tim      = 2,
        .port     = GPIOC,
        .rcc_mask = RCC_AHBENR_GPIOCEN,
        .CH0      = 6,
        .CH1      = 7,
        .CH2      = 8,
        .CH3      = 9,
        .AF       = 2
    }
};
/** @} */

/**
 * @name Timer configuration
 * @{
 */
#define TIMER_NUMOF         (TIMER_0_EN + TIMER_1_EN +TIMER_2_EN)
#define TIMER_0_EN          1
#define TIMER_1_EN          1
#define TIMER_2_EN          1
#define TIMER_IRQ_PRIO      1

static const timer_conf_t timer_config[TIMER_NUMOF] = {
    {
        .dev      = TIM2,
        .channels = 4,
        .freq     = (CLOCK_APB1 * 2),
        .rcc_mask = RCC_APB1ENR_TIM2EN,
        .bus      = APB1,
        .irqn     = TIM2_IRQn,
        .priority = TIMER_IRQ_PRIO
    },
    {
        .dev      = TIM5,
        .channels = 4,
        .freq     = (CLOCK_APB1 * 2),
        .rcc_mask = RCC_APB1ENR_TIM5EN,
        .bus      = APB1,
        .irqn     = TIM5_IRQn,
        .priority = TIMER_IRQ_PRIO
    },
    {
        .dev      = TIM3,
        .channels = 4,
        .freq     = (CLOCK_APB1 * 2),
        .rcc_mask = RCC_APB1ENR_TIM3EN,
        .bus      = APB1,
        .irqn     = TIM3_IRQn,
        .priority = TIMER_IRQ_PRIO
    }
};

#define TIMER_0_ISR         isr_tim2
#define TIMER_1_ISR         isr_tim5
#define TIMER_2_ISR         isr_tim3
/** @} */

/**
 * @brief UART configuration
 * @{
 */
#define UART_NUMOF          (UART_0_EN + UART_1_EN + UART_2_EN )
#define UART_0_EN           1
#define UART_1_EN           1
#define UART_2_EN           1
#define UART_IRQ_PRIO       1

/* UART 0 device configuration */
#define UART_0_DEV          USART3
#define UART_0_CLKEN()      (RCC->APB1ENR |= RCC_APB1ENR_USART3EN)
#define UART_0_CLK          (CLOCK_CORECLOCK)
#define UART_0_IRQ          USART3_IRQn
#define UART_0_ISR          isr_usart3
#define UART_0_BUS_FREQ     32000000
/* UART 0 pin configuration */
#define UART_0_RX_PIN       GPIO_PIN(PORT_C, 11)
#define UART_0_TX_PIN       GPIO_PIN(PORT_C, 10)
#define UART_0_AF           GPIO_AF7

/* UART 1 device configuration */
#define UART_1_DEV          USART2       
#define UART_1_CLKEN()      (RCC->APB1ENR |= RCC_APB1ENR_USART2EN)
#define UART_1_CLK          (CLOCK_CORECLOCK)
#define UART_1_IRQ          USART2_IRQn
#define UART_1_ISR          isr_usart2
#define UART_1_BUS_FREQ     32000000
/* UART 1 pin configuration */
#define UART_1_RX_PIN       GPIO_PIN(PORT_A, 2)
#define UART_1_TX_PIN       GPIO_PIN(PORT_A, 1)
#define UART_1_AF           GPIO_AF7
/** @} */

/* UART 1 device configuration */
#define UART_2_DEV          USART1       
#define UART_2_CLKEN()      (RCC->APB2ENR |= RCC_APB2ENR_USART1EN)
#define UART_2_CLK          (CLOCK_CORECLOCK)
#define UART_2_IRQ          USART1_IRQn
#define UART_2_ISR          isr_usart1
#define UART_2_BUS_FREQ     32000000
/* UART 1 pin configuration */
#define UART_2_RX_PIN       GPIO_PIN(PORT_A, 10)
#define UART_2_TX_PIN       GPIO_PIN(PORT_A, 9)
#define UART_2_AF           GPIO_AF7
/** @} */

/**
 * @brief SPI configuration
 * @{
 */
#define SPI_NUMOF           (SPI_0_EN + SPI_1_EN + SPI_2_EN)
#define SPI_0_EN            1
#define SPI_1_EN            1
#define SPI_2_EN            0

/* SPI 0 device configuration */
#define SPI_0_DEV           SPI1  
#define SPI_0_CLKEN()       (RCC->APB2ENR |= RCC_APB2ENR_SPI1EN)
#define SPI_0_CLKDIS()      (RCC->APB2ENR &= ~(RCC_APB2ENR_SPI1EN))
#define SPI_0_IRQ           SPI1_IRQn
#define SPI_0_ISR           isr_spi1
/* SPI 0 pin configuration */
#define SPI_0_PORT_CLKEN()  (RCC->AHBENR |= RCC_AHBENR_GPIOBEN)
#define SPI_0_PORT          GPIOB
#define SPI_0_PIN_SCK       3
#define SPI_0_PIN_MOSI      5
#define SPI_0_PIN_MISO      4
#define SPI_0_PIN_AF        5

/* SPI 1 device configuration */
#define SPI_1_DEV           SPI2          
#define SPI_1_CLKEN()       (RCC->APB1ENR |= RCC_APB1ENR_SPI2EN)
#define SPI_1_CLKDIS()      (RCC->APB1ENR &= ~(RCC_APB1ENR_SPI2EN))
#define SPI_1_IRQ           SPI2_IRQn
#define SPI_1_ISR           isr_spi2
/* SPI 1 pin configuration */
#define SPI_1_PORT_CLKEN()  (RCC->AHBENR |= RCC_AHBENR_GPIOBEN)
#define SPI_1_PORT          GPIOB
#define SPI_1_PIN_SCK       13
#define SPI_1_PIN_MOSI      15
#define SPI_1_PIN_MISO      14
#define SPI_1_PIN_AF        5
/** @} */

/**
 * @name I2C configuration
  * @{
 */
#define I2C_NUMOF           (I2C_0_EN + I2C_1_EN)
#define I2C_0_EN            1
#define I2C_1_EN            1
#define I2C_IRQ_PRIO        1
#define I2C_APBCLK          (36000000U)          // Configurable from 2MHz to 50Mhz, steps of 2Mhz     

/* I2C 0 device configuration */
#define I2C_0_EVT_ISR       isr_i2c1_ev
#define I2C_0_ERR_ISR       isr_i2c1_er

/* I2C 1 device configuration */
#define I2C_1_EVT_ISR       isr_i2c2_ev
#define I2C_1_ERR_ISR       isr_i2c2_er

static const i2c_conf_t i2c_config[] = {
    /* device, port, scl-, sda-pin-number, I2C-AF, ER-IRQn, EV-IRQn */
    {I2C1, GPIO_PIN(PORT_B,  8), GPIO_PIN(PORT_B,  9),
     GPIO_OD_PU, GPIO_AF4, I2C1_ER_IRQn, I2C1_EV_IRQn},
    {I2C2, GPIO_PIN(PORT_B, 10), GPIO_PIN(PORT_B, 11),
     GPIO_OD_PU, GPIO_AF4, I2C2_ER_IRQn, I2C2_EV_IRQn},
};

/** @} */

/**
 * @name RTC configuration
 * @{
 */
#define RTC_NUMOF           (1U)
/** @} */

#ifdef __cplusplus
}
#endif

#endif /* PERIPH_CONF_H_ */
/** @} */
