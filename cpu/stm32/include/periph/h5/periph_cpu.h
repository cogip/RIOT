/*
 * SPDX-FileCopyrightText: 2026 Cogip
 * SPDX-License-Identifier: LGPL-2.1-only
 */

#pragma once

/**
 * @ingroup         cpu_stm32
 * @{
 *
 * @file
 * @brief           STM32H5 CPU specific definitions for internal peripheral handling
 *
 * @author          Gilles DOFFE <g.doffe@gmail.com>
 */

#ifdef __cplusplus
extern "C" {
#endif

#ifndef DOXYGEN

/**
 * @brief   Starting address of the ROM bootloader
 *          See application note AN5392 for details.
 */
#define STM32_BOOTLOADER_ADDR   (0x0BF87000)

/**
 * @name   Constants for internal VBAT ADC line
 * @{
 */
#define VBAT_ADC_RES        ADC_RES_12BIT
#define VBAT_ADC_MAX        4095
/** @} */

#endif /* ndef DOXYGEN */

#ifdef __cplusplus
}
#endif

/** @} */
