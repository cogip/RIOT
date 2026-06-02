/*
 * SPDX-FileCopyrightText: 2026 Cogip
 * SPDX-License-Identifier: LGPL-2.1-only
 */

#pragma once

/**
 * @ingroup     cpu_stm32
 * @{
 *
 * @file
 * @brief       Default STM32H5 clock configuration
 *
 * The STM32H5 family runs the system clock from one of:
 *  - HSI (high-speed internal RC, 64 MHz)
 *  - HSE (high-speed external crystal)
 *  - CSI (low-power internal RC, 4 MHz)
 *  - PLL1 sourced by HSI, HSE or CSI (default: 250 MHz target)
 *
 * @author      Gilles DOFFE <g.doffe@gmail.com>
 */

#include "kernel_defines.h"
#include "macros/units.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @name    H5 system clock source selection
 * @{
 */
#ifndef CONFIG_USE_CLOCK_PLL
#if IS_ACTIVE(CONFIG_USE_CLOCK_HSE) || IS_ACTIVE(CONFIG_USE_CLOCK_HSI)
#define CONFIG_USE_CLOCK_PLL            0
#else
#define CONFIG_USE_CLOCK_PLL            1   /* PLL by default */
#endif
#endif

#if IS_ACTIVE(CONFIG_USE_CLOCK_PLL) && \
    (IS_ACTIVE(CONFIG_USE_CLOCK_HSE) || IS_ACTIVE(CONFIG_USE_CLOCK_HSI))
#error "Cannot select PLL together with HSE or HSI as system clock"
#endif

#if IS_ACTIVE(CONFIG_USE_CLOCK_HSE) && IS_ACTIVE(CONFIG_USE_CLOCK_HSI)
#error "Cannot select both HSE and HSI as system clock"
#endif
/** @} */

/**
 * @name    Oscillator default frequencies
 * @{
 */
#ifndef CONFIG_CLOCK_HSI
#define CONFIG_CLOCK_HSI                MHZ(64)
#endif
#ifndef CONFIG_CLOCK_HSE
#define CONFIG_CLOCK_HSE                MHZ(8)
#endif
#ifndef CONFIG_CLOCK_CSI
#define CONFIG_CLOCK_CSI                MHZ(4)
#endif
/** @} */

/**
 * @name    PLL input source selection
 * @{
 */
#ifndef CONFIG_CLOCK_PLL_SRC_HSE
#if IS_ACTIVE(CONFIG_BOARD_HAS_HSE) && !IS_ACTIVE(CONFIG_CLOCK_PLL_SRC_HSI) && \
    !IS_ACTIVE(CONFIG_CLOCK_PLL_SRC_CSI)
#define CONFIG_CLOCK_PLL_SRC_HSE        1
#else
#define CONFIG_CLOCK_PLL_SRC_HSE        0
#endif
#endif

#ifndef CONFIG_CLOCK_PLL_SRC_HSI
#if !IS_ACTIVE(CONFIG_CLOCK_PLL_SRC_HSE) && !IS_ACTIVE(CONFIG_CLOCK_PLL_SRC_CSI)
#define CONFIG_CLOCK_PLL_SRC_HSI        1
#else
#define CONFIG_CLOCK_PLL_SRC_HSI        0
#endif
#endif

#ifndef CONFIG_CLOCK_PLL_SRC_CSI
#define CONFIG_CLOCK_PLL_SRC_CSI        0
#endif

#if IS_ACTIVE(CONFIG_CLOCK_PLL_SRC_HSE)
#define CLOCK_PLL_SRC                   (CONFIG_CLOCK_HSE)
#elif IS_ACTIVE(CONFIG_CLOCK_PLL_SRC_CSI)
#define CLOCK_PLL_SRC                   (CONFIG_CLOCK_CSI)
#else
#define CLOCK_PLL_SRC                   (CONFIG_CLOCK_HSI)
#endif
/** @} */

/**
 * @name    PLL1 dividers
 *
 * SYSCLK = ((PLL_SRC / PLL_M) * PLL_N) / PLL_P
 *
 * Constraints (RM0481):
 *  - VCO input:  PLL_SRC / M  must be in [1 MHz, 16 MHz]
 *  - VCO output: VCO_IN * N   must be in [128 MHz, 560 MHz] (wide VCO)
 *  - SYSCLK max: 250 MHz at VOS0
 *
 * Default targets 250 MHz from HSI (64 MHz):
 *  - M = 8  -> VCO_IN  = 8 MHz
 *  - N = 250 -> VCO_OUT = 500 MHz (wide VCO range)
 *  - P = 2  -> SYSCLK  = 250 MHz
 * @{
 */
#ifndef CONFIG_CLOCK_PLL_M
#if IS_ACTIVE(CONFIG_CLOCK_PLL_SRC_HSE) && (CONFIG_CLOCK_HSE == MHZ(8))
#define CONFIG_CLOCK_PLL_M              (1)
#elif IS_ACTIVE(CONFIG_CLOCK_PLL_SRC_HSE) && (CONFIG_CLOCK_HSE == MHZ(25))
#define CONFIG_CLOCK_PLL_M              (5)
#elif IS_ACTIVE(CONFIG_CLOCK_PLL_SRC_CSI)
#define CONFIG_CLOCK_PLL_M              (1)
#else /* HSI 64 MHz */
#define CONFIG_CLOCK_PLL_M              (8)
#endif
#endif

#ifndef CONFIG_CLOCK_PLL_N
#if IS_ACTIVE(CONFIG_CLOCK_PLL_SRC_HSE) && (CONFIG_CLOCK_HSE == MHZ(8))
#define CONFIG_CLOCK_PLL_N              (62)    /* VCO = 496 MHz, SYSCLK = 248 MHz */
#elif IS_ACTIVE(CONFIG_CLOCK_PLL_SRC_HSE) && (CONFIG_CLOCK_HSE == MHZ(25))
#define CONFIG_CLOCK_PLL_N              (100)   /* VCO = 500 MHz, SYSCLK = 250 MHz */
#elif IS_ACTIVE(CONFIG_CLOCK_PLL_SRC_CSI)
#define CONFIG_CLOCK_PLL_N              (125)   /* VCO = 500 MHz, SYSCLK = 250 MHz */
#else
#define CONFIG_CLOCK_PLL_N              (250)   /* VCO = 500 MHz, SYSCLK = 250 MHz */
#endif
#endif

#ifndef CONFIG_CLOCK_PLL_P
#define CONFIG_CLOCK_PLL_P              (2)
#endif
#ifndef CONFIG_CLOCK_PLL_Q
#define CONFIG_CLOCK_PLL_Q              (2)
#endif
#ifndef CONFIG_CLOCK_PLL_R
#define CONFIG_CLOCK_PLL_R              (2)
#endif
/** @} */

/**
 * @name    Derived clocks
 * @{
 */
#if IS_ACTIVE(CONFIG_USE_CLOCK_HSI)
#define CLOCK_CORECLOCK                 (CONFIG_CLOCK_HSI)
#elif IS_ACTIVE(CONFIG_USE_CLOCK_HSE)
#define CLOCK_CORECLOCK                 (CONFIG_CLOCK_HSE)
#elif IS_ACTIVE(CONFIG_USE_CLOCK_PLL)
#define CLOCK_CORECLOCK \
        ((CLOCK_PLL_SRC / CONFIG_CLOCK_PLL_M) * CONFIG_CLOCK_PLL_N) / CONFIG_CLOCK_PLL_P
#endif

#define CLOCK_CORECLOCK_MAX             MHZ(250)
#if CLOCK_CORECLOCK > CLOCK_CORECLOCK_MAX
#error "SYSCLK exceeds the STM32H5 maximum of 250 MHz"
#endif

#ifndef CONFIG_CLOCK_AHB_DIV
#define CONFIG_CLOCK_AHB_DIV            (1)
#endif
#define CLOCK_AHB                       (CLOCK_CORECLOCK / CONFIG_CLOCK_AHB_DIV)

#ifndef CONFIG_CLOCK_APB1_DIV
#define CONFIG_CLOCK_APB1_DIV           (1)
#endif
#define CLOCK_APB1                      (CLOCK_AHB / CONFIG_CLOCK_APB1_DIV)

#ifndef CONFIG_CLOCK_APB2_DIV
#define CONFIG_CLOCK_APB2_DIV           (1)
#endif
#define CLOCK_APB2                      (CLOCK_AHB / CONFIG_CLOCK_APB2_DIV)

#ifndef CONFIG_CLOCK_APB3_DIV
#define CONFIG_CLOCK_APB3_DIV           (1)
#endif
#define CLOCK_APB3                      (CLOCK_AHB / CONFIG_CLOCK_APB3_DIV)
/** @} */

#ifdef __cplusplus
}
#endif

/** @} */
