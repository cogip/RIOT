/*
 * Copyright (C) 2016 Inria
 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 */

/**
 * @defgroup    boards_nucleo-f446 STM32 Nucleo-F446
 * @ingroup     boards_common_nucleo64
 * @brief       Support for the STM32 Nucleo-F446
 * @{
 *
 * @file
 * @name        Peripheral MCU configuration for the nucleo-f446 board
 *
 * @author      Alexandre Abadie <alexandre.abadie@inria.fr>
 */

#ifndef PERIPH_CONF_H
#define PERIPH_CONF_H

#include "periph_cpu.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @name    Clock settings
 *
 * @note    This is auto-generated from
 *          `cpu/stm32_common/dist/clk_conf/clk_conf.c`
 * @{
 */

#ifndef COGIP2018_CPU
#error "Please add 'CFLAGS=-DCOGIP2018_CPU=446 (or 401, 411) to select build variant"
#error "ex.   CFLAGS=-DCOGIP2018_CPU=446 BOARD=cogip2018-f4xx make"
#endif

#if COGIP2018_CPU == 446 /* ************************************************ */
/* give the target core clock (HCLK) frequency [in Hz],
 * maximum: 180MHz */
#define CLOCK_CORECLOCK     (180000000U)
/* 0: no external high speed crystal available
 * else: actual crystal frequency [in Hz] */
#define CLOCK_HSE           (0)
/* 0: no external low speed crystal available,
 * 1: external crystal available (always 32.768kHz) */
#define CLOCK_LSE           (1)
/* peripheral clock setup */
#define CLOCK_AHB_DIV       RCC_CFGR_HPRE_DIV1
#define CLOCK_AHB           (CLOCK_CORECLOCK / 1)
#define CLOCK_APB1_DIV      RCC_CFGR_PPRE1_DIV4     /* max 45MHz */
#define CLOCK_APB1          (CLOCK_CORECLOCK / 4)
#define CLOCK_APB2_DIV      RCC_CFGR_PPRE2_DIV2     /* max 90MHz */
#define CLOCK_APB2          (CLOCK_CORECLOCK / 2)

/* Main PLL factors */
#define CLOCK_PLL_M          (4)
#define CLOCK_PLL_N          (180)
#define CLOCK_PLL_P          (4)
#define CLOCK_PLL_Q          (0)

/* PLL SAI configuration */
#define CLOCK_ENABLE_PLL_SAI (1)
#define CLOCK_PLL_SAI_M      (4)
#define CLOCK_PLL_SAI_N      (192)
#define CLOCK_PLL_SAI_P      (8)
#define CLOCK_PLL_SAI_Q      (0)

/* Use alternative source for 48MHz clock */
#define CLOCK_USE_ALT_48MHZ  (1)

#elif COGIP2018_CPU == 401 /* ********************************************** */
/* give the target core clock (HCLK) frequency [in Hz],
 * maximum: 84MHz */
#define CLOCK_CORECLOCK     (84000000U)
/* 0: no external high speed crystal available
 * else: actual crystal frequency [in Hz] */
#define CLOCK_HSE           (8000000U)
/* 0: no external low speed crystal available,
 * 1: external crystal available (always 32.768kHz) */
#define CLOCK_LSE           (1)
/* peripheral clock setup */
#define CLOCK_AHB_DIV       RCC_CFGR_HPRE_DIV1
#define CLOCK_AHB           (CLOCK_CORECLOCK / 1)
#define CLOCK_APB1_DIV      RCC_CFGR_PPRE1_DIV2     /* max 42MHz */
#define CLOCK_APB1          (CLOCK_CORECLOCK / 2)
#define CLOCK_APB2_DIV      RCC_CFGR_PPRE2_DIV1     /* max 84MHz */
#define CLOCK_APB2          (CLOCK_CORECLOCK / 1)

/* Main PLL factors */
#define CLOCK_PLL_M          (4)
#define CLOCK_PLL_N          (168)
#define CLOCK_PLL_P          (4)
#define CLOCK_PLL_Q          (7)

#elif COGIP2018_CPU == 411

/* give the target core clock (HCLK) frequency [in Hz],
 * maximum: 100MHz */
#define CLOCK_CORECLOCK     (96000000U)
/* 0: no external high speed crystal available
 * else: actual crystal frequency [in Hz] */
#define CLOCK_HSE           (8000000U)
/* 0: no external low speed crystal available,
 * 1: external crystal available (always 32.768kHz) */
#define CLOCK_LSE           (1)
/* peripheral clock setup */
#define CLOCK_AHB_DIV       RCC_CFGR_HPRE_DIV1
#define CLOCK_AHB           (CLOCK_CORECLOCK / 1)
#define CLOCK_APB1_DIV      RCC_CFGR_PPRE1_DIV2     /* max 50MHz */
#define CLOCK_APB1          (CLOCK_CORECLOCK / 2)
#define CLOCK_APB2_DIV      RCC_CFGR_PPRE2_DIV1     /* max 100MHz */
#define CLOCK_APB2          (CLOCK_CORECLOCK / 1)

/* Main PLL factors */
#define CLOCK_PLL_M          (4)
#define CLOCK_PLL_N          (192)
#define CLOCK_PLL_P          (4)
#define CLOCK_PLL_Q          (8)

#else /* ********************************************** */
#error "Bad COGIP2018_CPU flag, should be 'CFLAGS=-DCOGIP2018_CPU=446' (or 401, 411) to select build variant"
#error "ex.   CFLAGS=-DCOGIP2018_CPU=446 BOARD=cogip2018-f4xx make"

#endif /* ******************************************************************* */
/** @} */

/**
 * @name   Timer configuration
 * @{
 */
static const timer_conf_t timer_config[] = {
    {
        .dev      = TIM5,
        .max      = 0xffffffff,
        .rcc_mask = RCC_APB1ENR_TIM5EN,
        .bus      = APB1,
        .irqn     = TIM5_IRQn
    }
};

#define TIMER_0_ISR         isr_tim5

#define TIMER_NUMOF         (sizeof(timer_config) / sizeof(timer_config[0]))
/** @} */

/**
 * @name   UART configuration
 * @{
 */
static const uart_conf_t uart_config[] = {
    {
        .dev        = USART2,
        .rcc_mask   = RCC_APB1ENR_USART2EN,
        .rx_pin     = GPIO_PIN(PORT_A, 3),
        .tx_pin     = GPIO_PIN(PORT_A, 2),
        .rx_af      = GPIO_AF7,
        .tx_af      = GPIO_AF7,
        .bus        = APB1,
        .irqn       = USART2_IRQn,
#ifdef UART_USE_DMA
        .dma_stream = 6,
        .dma_chan   = 4
#endif
    },
    {
        .dev        = USART6,
        .rcc_mask   = RCC_APB2ENR_USART6EN,
        .rx_pin     = GPIO_PIN(PORT_A, 12),
        .tx_pin     = GPIO_PIN(PORT_A, 11),
        .rx_af      = GPIO_AF8,
        .tx_af      = GPIO_AF8,
        .bus        = APB2,
        .irqn       = USART6_IRQn,
#ifdef UART_USE_DMA
        .dma_stream = 6,
        .dma_chan   = 4
#endif
    }
};

#define UART_0_ISR          (isr_usart2)
#define UART_0_DMA_ISR      (isr_dma1_stream6)
#define UART_1_ISR          (isr_usart6)
#define UART_1_DMA_ISR      (isr_dma1_stream6)

#define UART_NUMOF          (sizeof(uart_config) / sizeof(uart_config[0]))
/** @} */

/**
 * @name    PWM configuration
 * @{
 */
static const pwm_conf_t pwm_config[] = {
    {
        .dev      = TIM2,
        .rcc_mask = RCC_APB1ENR_TIM2EN,
        .chan     = { { .pin = GPIO_PIN(PORT_A, 5), .cc_chan = 0},
                      { .pin = GPIO_PIN(PORT_B, 3), .cc_chan = 1},
                      { .pin = GPIO_UNDEF,          .cc_chan = 2},
                      { .pin = GPIO_UNDEF,          .cc_chan = 3} },
        .af       = GPIO_AF1,
        .bus      = APB1
    },
    {
        .dev      = TIM1,
        .rcc_mask = RCC_APB2ENR_TIM1EN,
        .chan     = { { .pin = GPIO_UNDEF,          .cc_chan = 0},
                      { .pin = GPIO_UNDEF,          .cc_chan = 1},
                      { .pin = GPIO_UNDEF,          .cc_chan = 2},
                      { .pin = GPIO_PIN(PORT_A, 11),.cc_chan = 3} },
        .af       = GPIO_AF1,
        .bus      = APB2
    },
};

#define PWM_NUMOF           (sizeof(pwm_config) / sizeof(pwm_config[0]))
/** @} */

/**
 * @name    QDEC configuration
 * @{
 */
static const qdec_conf_t qdec_config[] = {
    {
        .dev      = TIM3,
        .max      = 0xffffffff,
        .rcc_mask = RCC_APB1ENR_TIM3EN,
        .chan     = { { .pin = GPIO_PIN(PORT_C, 6), .cc_chan = 0 },
                      { .pin = GPIO_PIN(PORT_C, 7), .cc_chan = 1 } },
        .af       = GPIO_AF2,
        .bus      = APB1,
        .irqn     = TIM3_IRQn
    },
    {
        .dev      = TIM4,
        .max      = 0xffffffff,
        .rcc_mask = RCC_APB1ENR_TIM4EN,
        .chan     = { { .pin = GPIO_PIN(PORT_B, 6), .cc_chan = 0 },
                      { .pin = GPIO_PIN(PORT_B, 7), .cc_chan = 1 } },
        .af       = GPIO_AF2,
        .bus      = APB1,
        .irqn     = TIM4_IRQn
    },
};

#define QDEC_0_ISR         isr_tim3
#define QDEC_1_ISR         isr_tim4

#define QDEC_NUMOF           (sizeof(qdec_config) / sizeof(qdec_config[0]))
/** @} */

/**
 * @name I2C configuration
 * @{
 */
//#define I2C_NUMOF           (1U)
//#define I2C_0_EN            1
//#define I2C_IRQ_PRIO        1
//#define I2C_APBCLK          (CLOCK_APB1)
//
///* I2C 0 device configuration */
//#define I2C_0_DEV           I2C1
//#define I2C_0_CLKEN()       (periph_clk_en(APB1, RCC_APB1ENR_I2C1EN))
//#define I2C_0_CLKDIS()      (periph_clk_dis(APB1, RCC_APB1ENR_I2C1EN))
//#define I2C_0_EVT_IRQ       I2C1_EV_IRQn
//#define I2C_0_EVT_ISR       isr_i2c1_ev
//#define I2C_0_ERR_IRQ       I2C1_ER_IRQn
//#define I2C_0_ERR_ISR       isr_i2c1_er
///* I2C 0 pin configuration */
//#define I2C_0_SCL_PORT      GPIOB
//#define I2C_0_SCL_PIN       8
//#define I2C_0_SCL_AF        4
//#define I2C_0_SCL_CLKEN()   (periph_clk_en(AHB1, RCC_AHB1ENR_GPIOBEN))
//#define I2C_0_SDA_PORT      GPIOB
//#define I2C_0_SDA_PIN       9
//#define I2C_0_SDA_AF        4
//#define I2C_0_SDA_CLKEN()   (periph_clk_en(AHB1, RCC_AHB1ENR_GPIOBEN))
/** @} */

/**
 * @name   ADC configuration
 *
 * Note that we do not configure all ADC channels,
 * and not in the STM32F446 order.  Instead, we
 * just define 6 ADC channels, for the Nucleo
 * Arduino header pins A0-A5
 *
 * @{
 */
#define ADC_NUMOF          (8U)
#define ADC_CONFIG {             \
    {GPIO_PIN(PORT_B, 0), 0, 8}, \
    {GPIO_PIN(PORT_B, 1), 0, 9}, \
    {GPIO_PIN(PORT_C, 0), 0, 10}, \
    {GPIO_PIN(PORT_C, 1), 0, 11}, \
    {GPIO_PIN(PORT_C, 2), 0, 12}, \
    {GPIO_PIN(PORT_C, 3), 0, 13}, \
    {GPIO_PIN(PORT_C, 4), 0, 14}, \
    {GPIO_PIN(PORT_C, 5), 0, 15}, \
}
/** @} */

#ifdef __cplusplus
}
#endif

#endif /* PERIPH_CONF_H */
/** @} */
