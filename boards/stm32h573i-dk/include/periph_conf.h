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
 * @brief       Peripheral configuration for the STM32H573I-DK board
 *
 * The STM32H573I-DK exposes a single Cortex-M33 core running at up to 250 MHz.
 * HSE is fed by the ST-LINK V3 MCO at 25 MHz (HSE bypass mode); LSE is provided
 * by an on-board 32.768 kHz crystal.
 *
 * @author      Gilles DOFFE <g.doffe@gmail.com>
 */

#ifndef CONFIG_BOARD_HAS_LSE
#define CONFIG_BOARD_HAS_LSE        1
#endif

#ifndef CONFIG_BOARD_HAS_HSE
#define CONFIG_BOARD_HAS_HSE        1
#endif

#ifndef CONFIG_CLOCK_HSE
#define CONFIG_CLOCK_HSE            MHZ(25)
#endif

#include "periph_cpu.h"
#include "clk_conf.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @name    Timer configuration
 * @{
 */
static const timer_conf_t timer_config[] = {
    {
        .dev      = TIM2,
        .max      = 0xffffffff,
        .rcc_mask = RCC_APB1LENR_TIM2EN,
        .bus      = APB1,
        .irqn     = TIM2_IRQn,
    },
};

#define TIMER_0_ISR         isr_tim2

#define TIMER_NUMOF         ARRAY_SIZE(timer_config)
/** @} */

/**
 * @name    UART configuration
 *
 * USART1 is wired to the on-board ST-LINK V3 Virtual COM Port on PA9 (TX) /
 * PA10 (RX) per the STM32H573I-DK BSP (COM1).
 * @{
 */
static const uart_conf_t uart_config[] = {
    {
        .dev        = USART1,
        .rcc_mask   = RCC_APB2ENR_USART1EN,
        .rx_pin     = GPIO_PIN(PORT_A, 10),
        .tx_pin     = GPIO_PIN(PORT_A, 9),
        .rx_af      = GPIO_AF7,
        .tx_af      = GPIO_AF7,
        .bus        = APB2,
        .irqn       = USART1_IRQn,
    },
};

#define UART_0_ISR          (isr_usart1)

#define UART_NUMOF          ARRAY_SIZE(uart_config)
/** @} */

/**
 * @name    Ethernet configuration
 *
 * The STM32H573I-DK ships with a LAN8742A 10/100 PHY wired to the MAC in
 * RMII mode, MDIO address 0x00 (per UM3140). Pin map matches the ST BSP
 * `stm32h573i_discovery.h`: TXD0/TXD1/TX_EN on port G, the RX pair and
 * MDC on port C, MDIO / REF_CLK / CRS_DV on port A. All on AF11.
 * @{
 */
#include "mii.h"

static const eth_conf_t eth_config = {
    .mode = RMII,
    .speed = MII_BMCR_SPEED_100 | MII_BMCR_FULL_DPLX,
    .dma = 0,
    .dma_chan = 0,
    .phy_addr = 0x00,
    .pins = {
        GPIO_PIN(PORT_G, 13),       /* RMII_TXD0     */
        GPIO_PIN(PORT_G, 12),       /* RMII_TXD1     */
        GPIO_PIN(PORT_G, 11),       /* RMII_TX_EN    */
        GPIO_PIN(PORT_C, 4),        /* RMII_RXD0     */
        GPIO_PIN(PORT_C, 5),        /* RMII_RXD1     */
        GPIO_PIN(PORT_A, 7),        /* RMII_CRS_DV   */
        GPIO_PIN(PORT_C, 1),        /* ETH_MDC       */
        GPIO_PIN(PORT_A, 2),        /* ETH_MDIO      */
        GPIO_PIN(PORT_A, 1),        /* RMII_REF_CLK  */
    }
};
/** @} */

#ifdef __cplusplus
}
#endif

/** @} */
