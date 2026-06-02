/*
 * SPDX-FileCopyrightText: 2026 Cogip
 * SPDX-License-Identifier: LGPL-2.1-only
 */

/**
 * @ingroup     cpu_stm32
 * @{
 *
 * @file
 * @brief       Implementation of STM32 clock configuration for H5 family
 *
 * @author      Gilles DOFFE <g.doffe@gmail.com>
 * @}
 */

#include "cpu.h"
#include "stmclk.h"
#include "periph_conf.h"

/* SYSCLK source select values for RCC_CFGR1_SW field */
#define RCC_CFGR1_SW_HSI            (0x0UL)
#define RCC_CFGR1_SW_CSI            (RCC_CFGR1_SW_0)
#define RCC_CFGR1_SW_HSE            (RCC_CFGR1_SW_1)
#define RCC_CFGR1_SW_PLL1           (RCC_CFGR1_SW_1 | RCC_CFGR1_SW_0)
#define RCC_CFGR1_SWS_HSI           (0x0UL)
#define RCC_CFGR1_SWS_CSI           (RCC_CFGR1_SWS_0)
#define RCC_CFGR1_SWS_HSE           (RCC_CFGR1_SWS_1)
#define RCC_CFGR1_SWS_PLL1          (RCC_CFGR1_SWS_1 | RCC_CFGR1_SWS_0)

/* Resolve PLL1 input source */
#if IS_ACTIVE(CONFIG_USE_CLOCK_PLL) && IS_ACTIVE(CONFIG_CLOCK_PLL_SRC_HSE) && \
    IS_ACTIVE(CONFIG_BOARD_HAS_HSE)
#define PLL1_SRC                    (RCC_PLL1CFGR_PLL1SRC_1 | RCC_PLL1CFGR_PLL1SRC_0)
#elif IS_ACTIVE(CONFIG_USE_CLOCK_PLL) && IS_ACTIVE(CONFIG_CLOCK_PLL_SRC_HSI)
#define PLL1_SRC                    (RCC_PLL1CFGR_PLL1SRC_1)
#elif IS_ACTIVE(CONFIG_USE_CLOCK_PLL) && IS_ACTIVE(CONFIG_CLOCK_PLL_SRC_CSI)
#define PLL1_SRC                    (RCC_PLL1CFGR_PLL1SRC_0)
#else
#define PLL1_SRC                    (0)
#endif

/* PLL1 dividers */
#ifndef CONFIG_CLOCK_PLL_M
#define CONFIG_CLOCK_PLL_M          (1)
#endif
#ifndef CONFIG_CLOCK_PLL_N
#define CONFIG_CLOCK_PLL_N          (16)
#endif
#ifndef CONFIG_CLOCK_PLL_P
#define CONFIG_CLOCK_PLL_P          (2)
#endif
#ifndef CONFIG_CLOCK_PLL_Q
#define CONFIG_CLOCK_PLL_Q          (2)
#endif
#ifndef CONFIG_CLOCK_PLL_R
#define CONFIG_CLOCK_PLL_R          (2)
#endif

#define PLL1_M                      ((CONFIG_CLOCK_PLL_M) << RCC_PLL1CFGR_PLL1M_Pos)
#define PLL1_N                      ((CONFIG_CLOCK_PLL_N - 1) << RCC_PLL1DIVR_PLL1N_Pos)
#define PLL1_P                      ((CONFIG_CLOCK_PLL_P - 1) << RCC_PLL1DIVR_PLL1P_Pos)
#define PLL1_Q                      ((CONFIG_CLOCK_PLL_Q - 1) << RCC_PLL1DIVR_PLL1Q_Pos)
#define PLL1_R                      ((CONFIG_CLOCK_PLL_R - 1) << RCC_PLL1DIVR_PLL1R_Pos)

/* AHB / APB prescalers */
#if CONFIG_CLOCK_AHB_DIV == 1
#define CLOCK_HPRE                  (0)
#elif CONFIG_CLOCK_AHB_DIV == 2
#define CLOCK_HPRE                  (RCC_CFGR2_HPRE_3)
#elif CONFIG_CLOCK_AHB_DIV == 4
#define CLOCK_HPRE                  (RCC_CFGR2_HPRE_3 | RCC_CFGR2_HPRE_0)
#elif CONFIG_CLOCK_AHB_DIV == 8
#define CLOCK_HPRE                  (RCC_CFGR2_HPRE_3 | RCC_CFGR2_HPRE_1)
#elif CONFIG_CLOCK_AHB_DIV == 16
#define CLOCK_HPRE                  (RCC_CFGR2_HPRE_3 | RCC_CFGR2_HPRE_1 | RCC_CFGR2_HPRE_0)
#else
#define CLOCK_HPRE                  (0)
#endif

#if CONFIG_CLOCK_APB1_DIV == 1
#define CLOCK_PPRE1                 (0)
#elif CONFIG_CLOCK_APB1_DIV == 2
#define CLOCK_PPRE1                 (RCC_CFGR2_PPRE1_2)
#elif CONFIG_CLOCK_APB1_DIV == 4
#define CLOCK_PPRE1                 (RCC_CFGR2_PPRE1_2 | RCC_CFGR2_PPRE1_0)
#elif CONFIG_CLOCK_APB1_DIV == 8
#define CLOCK_PPRE1                 (RCC_CFGR2_PPRE1_2 | RCC_CFGR2_PPRE1_1)
#elif CONFIG_CLOCK_APB1_DIV == 16
#define CLOCK_PPRE1                 (RCC_CFGR2_PPRE1_2 | RCC_CFGR2_PPRE1_1 | RCC_CFGR2_PPRE1_0)
#else
#define CLOCK_PPRE1                 (0)
#endif

#if CONFIG_CLOCK_APB2_DIV == 1
#define CLOCK_PPRE2                 (0)
#elif CONFIG_CLOCK_APB2_DIV == 2
#define CLOCK_PPRE2                 (RCC_CFGR2_PPRE2_2)
#elif CONFIG_CLOCK_APB2_DIV == 4
#define CLOCK_PPRE2                 (RCC_CFGR2_PPRE2_2 | RCC_CFGR2_PPRE2_0)
#elif CONFIG_CLOCK_APB2_DIV == 8
#define CLOCK_PPRE2                 (RCC_CFGR2_PPRE2_2 | RCC_CFGR2_PPRE2_1)
#elif CONFIG_CLOCK_APB2_DIV == 16
#define CLOCK_PPRE2                 (RCC_CFGR2_PPRE2_2 | RCC_CFGR2_PPRE2_1 | RCC_CFGR2_PPRE2_0)
#else
#define CLOCK_PPRE2                 (0)
#endif

#if !defined(CONFIG_CLOCK_APB3_DIV) || CONFIG_CLOCK_APB3_DIV == 1
#define CLOCK_PPRE3                 (0)
#elif CONFIG_CLOCK_APB3_DIV == 2
#define CLOCK_PPRE3                 (RCC_CFGR2_PPRE3_2)
#elif CONFIG_CLOCK_APB3_DIV == 4
#define CLOCK_PPRE3                 (RCC_CFGR2_PPRE3_2 | RCC_CFGR2_PPRE3_0)
#elif CONFIG_CLOCK_APB3_DIV == 8
#define CLOCK_PPRE3                 (RCC_CFGR2_PPRE3_2 | RCC_CFGR2_PPRE3_1)
#elif CONFIG_CLOCK_APB3_DIV == 16
#define CLOCK_PPRE3                 (RCC_CFGR2_PPRE3_2 | RCC_CFGR2_PPRE3_1 | RCC_CFGR2_PPRE3_0)
#else
#define CLOCK_PPRE3                 (0)
#endif

/* Flash wait states for VOS0 at given core clock.
 * From RM0481 table "Flash latency".
 */
#if CLOCK_AHB <= MHZ(42)
#define FLASH_WAITSTATES            FLASH_ACR_LATENCY_0WS
#elif CLOCK_AHB <= MHZ(84)
#define FLASH_WAITSTATES            FLASH_ACR_LATENCY_1WS
#elif CLOCK_AHB <= MHZ(126)
#define FLASH_WAITSTATES            FLASH_ACR_LATENCY_2WS
#elif CLOCK_AHB <= MHZ(168)
#define FLASH_WAITSTATES            FLASH_ACR_LATENCY_3WS
#elif CLOCK_AHB <= MHZ(210)
#define FLASH_WAITSTATES            FLASH_ACR_LATENCY_4WS
#elif CLOCK_AHB <= MHZ(250)
#define FLASH_WAITSTATES            FLASH_ACR_LATENCY_5WS
#else
#error "Unsupported core clock for STM32H5 flash wait state lookup"
#endif

/* Determine which oscillators must be enabled */
#if IS_ACTIVE(CONFIG_USE_CLOCK_PLL)
#define CLOCK_ENABLE_PLL            1
#else
#define CLOCK_ENABLE_PLL            0
#endif

#if IS_ACTIVE(CONFIG_USE_CLOCK_HSE) || \
    (IS_ACTIVE(CLOCK_ENABLE_PLL) && IS_ACTIVE(CONFIG_CLOCK_PLL_SRC_HSE))
#define CLOCK_ENABLE_HSE            1
#else
#define CLOCK_ENABLE_HSE            0
#endif

#if IS_ACTIVE(CLOCK_ENABLE_HSE) && !IS_ACTIVE(CONFIG_BOARD_HAS_HSE)
#error "HSE is required by the clock configuration but not provided by the board."
#endif

#if IS_ACTIVE(CONFIG_USE_CLOCK_HSI) || \
    (IS_ACTIVE(CLOCK_ENABLE_PLL) && IS_ACTIVE(CONFIG_CLOCK_PLL_SRC_HSI))
#define CLOCK_ENABLE_HSI            1
#else
#define CLOCK_ENABLE_HSI            0
#endif

void stmclk_init_sysclk(void)
{
    /* disable any interrupts. Global interrupts could be enabled if this is
     * called from some kind of bootloader. */
    unsigned is = irq_disable();
    RCC->CIER = 0;

    /* enable HSI clock for the duration of initialization */
    stmclk_enable_hsi();

    /* use HSI as system clock while we do any further configuration */
    RCC->CFGR1 = (RCC->CFGR1 & ~RCC_CFGR1_SW) | RCC_CFGR1_SW_HSI;
    while ((RCC->CFGR1 & RCC_CFGR1_SWS) != RCC_CFGR1_SWS_HSI) {}

    /* configure AHB/APB prescalers */
    RCC->CFGR2 = (CLOCK_HPRE | CLOCK_PPRE1 | CLOCK_PPRE2 | CLOCK_PPRE3);

    /* select the highest voltage scaling range (VOS0) to allow 250 MHz */
    PWR->VOSCR = (PWR->VOSCR & ~PWR_VOSCR_VOS) |
                 (PWR_VOSCR_VOS_1 | PWR_VOSCR_VOS_0);
    while (!(PWR->VOSSR & PWR_VOSSR_VOSRDY)) {}

    /* configure flash wait states */
    FLASH->ACR = (FLASH->ACR & ~FLASH_ACR_LATENCY) | FLASH_WAITSTATES;
    while ((FLASH->ACR & FLASH_ACR_LATENCY) != FLASH_WAITSTATES) {}

    /* disable all active clocks except HSI -> resets the clk configuration */
    RCC->CR = RCC_CR_HSION;
    while (!(RCC->CR & RCC_CR_HSIRDY)) {}

    /* Enable HSE only when it is used */
    if (IS_ACTIVE(CLOCK_ENABLE_HSE)) {
        RCC->CR |= RCC_CR_HSEON;
        while (!(RCC->CR & RCC_CR_HSERDY)) {}
    }

    /* Enable PLL1 only when it is used */
    if (IS_ACTIVE(CLOCK_ENABLE_PLL)) {
        RCC->PLL1CFGR = (PLL1_SRC | PLL1_M |
                         RCC_PLL1CFGR_PLL1RGE_1 |
                         RCC_PLL1CFGR_PLL1PEN |
                         RCC_PLL1CFGR_PLL1QEN |
                         RCC_PLL1CFGR_PLL1REN);
        RCC->PLL1DIVR = (PLL1_N | PLL1_P | PLL1_Q | PLL1_R);
        RCC->PLL1FRACR = 0;
        RCC->CR |= RCC_CR_PLL1ON;
        while (!(RCC->CR & RCC_CR_PLL1RDY)) {}
    }

    /* Switch SYSCLK to the configured source */
    if (IS_ACTIVE(CONFIG_USE_CLOCK_PLL)) {
        RCC->CFGR1 = (RCC->CFGR1 & ~RCC_CFGR1_SW) | RCC_CFGR1_SW_PLL1;
        while ((RCC->CFGR1 & RCC_CFGR1_SWS) != RCC_CFGR1_SWS_PLL1) {}
    }
    else if (IS_ACTIVE(CONFIG_USE_CLOCK_HSE)) {
        RCC->CFGR1 = (RCC->CFGR1 & ~RCC_CFGR1_SW) | RCC_CFGR1_SW_HSE;
        while ((RCC->CFGR1 & RCC_CFGR1_SWS) != RCC_CFGR1_SWS_HSE) {}
    }
    /* default: keep HSI as SYSCLK */

    if (!IS_ACTIVE(CLOCK_ENABLE_HSI)) {
        /* disable HSI only if not used */
        stmclk_disable_hsi();
    }

    irq_restore(is);
}
