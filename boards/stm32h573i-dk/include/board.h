/*
 * SPDX-FileCopyrightText: 2026 Cogip
 * SPDX-License-Identifier: LGPL-2.1-only
 */

#pragma once

/**
 * @ingroup     boards_stm32h573i-dk
 * @{
 *
 * @file
 * @brief       Board specific definitions for the STM32H573I-DK board
 *
 * @author      Gilles DOFFE <g.doffe@gmail.com>
 */

#include "cpu.h"
#include "periph_conf.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @name    On-board LEDs (per UM3140)
 * @{
 */
#define LED0_PIN_NUM        9        /**< LD1, green */
#define LED0_PORT           GPIO_PORT_I
#define LED0_PORT_NUM       PORT_I

#define LED1_PIN_NUM        8        /**< LD2, orange */
#define LED1_PORT           GPIO_PORT_I
#define LED1_PORT_NUM       PORT_I

#define LED2_PIN_NUM        1        /**< LD3, red */
#define LED2_PORT           GPIO_PORT_F
#define LED2_PORT_NUM       PORT_F

#define LED3_PIN_NUM        4        /**< LD4, blue */
#define LED3_PORT           GPIO_PORT_F
#define LED3_PORT_NUM       PORT_F
/** @} */

/**
 * @name    User button (B1)
 * @{
 */
#define BTN0_PIN            GPIO_PIN(PORT_C, 13)
#define BTN0_MODE           GPIO_IN
/** @} */

#ifdef __cplusplus
}
#endif

#include "stm32_leds.h"

/** @} */
