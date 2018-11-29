/*
 * Copyright (C) 2016-2017 Freie Universität Berlin
 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 */

/**
 * @defgroup    boards_common_nucleo64 STM32 Nucleo-64
 * @ingroup     boards
 * @brief       Support for STM32 Nucleo-64 boards
 * @{
 *
 * @file
 * @brief       Common pin definitions and board configuration options
 *
 * @author      Hauke Petersen <hauke.petersen@fu-berlin.de>
 * @author      Sebastian Meiling <s@mlng.net>
 */

#ifndef BOARD_H
#define BOARD_H

#include "board_nucleo.h"
#include "arduino_pinmap.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @name    LED pin definitions and handlers
 * @{
 */
#if defined(CPU_MODEL_STM32F302R8) || defined(CPU_MODEL_STM32L433RC)
#define LED0_PORT           GPIOB
#define LED0_PIN            GPIO_PIN(PORT_B, 13)
#define LED0_MASK           (1 << 13)
#else
#define LED0_PORT           GPIOA
#define LED0_PIN            GPIO_PIN(PORT_A, 5)
#define LED0_MASK           (1 << 5)
#endif

#define LED0_ON             (LED0_PORT->BSRR = LED0_MASK)
#define LED0_OFF            (LED0_PORT->BSRR = (LED0_MASK << 16))
#define LED0_TOGGLE         (LED0_PORT->ODR  ^= LED0_MASK)
/** @} */

/**
 * @name    User button
 * @{
 */
#define BTN0_PIN            GPIO_PIN(PORT_C, 13)
#ifdef CPU_MODEL_STM32L433RC
#define BTN0_MODE           GPIO_IN_PD
#else
#define BTN0_MODE           GPIO_IN_PU
#endif
/** @} */

#ifndef CC110X_PARAM_SPI
#define CC110X_PARAM_SPI            SPI_DEV(0)
#endif

#ifndef CC110X_PARAM_CS
#define CC110X_PARAM_CS             GPIO_PIN(PORT_A, 4)
#endif

#ifndef CC110X_PARAM_GDO0
//#define CC110X_PARAM_GDO0           GPIO_PIN(0, 27)
#endif

#ifndef CC110X_PARAM_GDO1
//#define CC110X_PARAM_GDO1           GPIO_PIN(1, 23)
#endif

#ifndef CC110X_PARAM_GDO2
#define CC110X_PARAM_GDO2           GPIO_PIN(PORT_B, 0)
#endif

#ifndef CC110X_PARAMS
#define CC110X_PARAMS               { \
                                        .spi  = CC110X_PARAM_SPI,  \
                                        .cs   = CC110X_PARAM_CS,   \
                                        .gdo0 = CC110X_PARAM_GDO0, \
                                        .gdo1 = CC110X_PARAM_GDO1, \
                                        .gdo2 = CC110X_PARAM_GDO2, \
                                    }
#endif

#ifdef __cplusplus
}
#endif

#endif /* BOARD_H */
/** @} */
