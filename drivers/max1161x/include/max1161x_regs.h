/*
 * Copyright (C) 2025 Mathis LECRIVAIN <lecrivain.mathis@gmail.com>
 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 */

/**
 * @ingroup     drivers_max1161x
 * @{
 *
 * @file
 * @brief       Register definition for MAX1161X devices
 *
 * @author      Mathis Lécrivain <lecrivain.mathis@gmail.com>
 */

#ifndef MAX1161X_REGS_H
#define MAX1161X_REGS_H

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @name    Setup Byte Register bits
 * @{
 */
#define MAX1161X_SETUP_REG_MASK (1 << 7)    /**< Setup byte identifier (always 1) */
#define MAX1161X_SETUP_SEL_MASK (0x07 << 4) /**< Reference and power selection */
#define MAX1161X_SETUP_CLK_MASK (1 << 3)    /**< Clock source selection */
#define MAX1161X_SETUP_BIP_MASK (1 << 2)    /**< Bipolar/Unipolar mode */
#define MAX1161X_SETUP_RST_MASK (1 << 1)    /**< Reset configuration */

#define MAX1161X_SETUP_REG          (1 << 7) /**< Setup byte identifier */
#define MAX1161X_SETUP_CLK_INTERNAL (0 << 3) /**< Internal clock */
#define MAX1161X_SETUP_CLK_EXTERNAL (1 << 3) /**< External clock */
#define MAX1161X_SETUP_UNIPOLAR     (0 << 2) /**< Unipolar mode (0 to VREF) */
#define MAX1161X_SETUP_BIPOLAR      (1 << 2) /**< Bipolar mode (-VREF/2 to +VREF/2) */
#define MAX1161X_SETUP_RST_CONFIG   (0 << 1) /**< Reset configuration */
#define MAX1161X_SETUP_NO_ACTION    (1 << 1) /**< No reset action */
/** @} */

/**
 * @name    Setup Byte SEL[2:0] values
 * @{
 */
#define MAX1161X_SETUP_SEL_VDD            (0 << 4) /**< Use VDD as reference */
#define MAX1161X_SETUP_SEL_EXT_REF        (1 << 4) /**< Use external VREF */
#define MAX1161X_SETUP_SEL_INT_REF        (2 << 4) /**< Internal 2.048V reference */
#define MAX1161X_SETUP_SEL_INT_REF_RST    (3 << 4) /**< Internal ref with reset */
#define MAX1161X_SETUP_SEL_INT_REF_PD     (4 << 4) /**< Internal ref + auto-shutdown */
#define MAX1161X_SETUP_SEL_INT_REF_RST_PD (5 << 4) /**< Internal ref + reset + auto-shutdown */
/** @} */

/**
 * @name    Configuration Byte Register bits
 * @{
 */
#define MAX1161X_CONFIG_REG_MASK  (1 << 7)    /**< Config byte identifier (always 0) */
#define MAX1161X_CONFIG_SCAN_MASK (0x03 << 5) /**< Scan mode selection */
#define MAX1161X_CONFIG_CS_MASK   (0x0F << 1) /**< Channel selection */
#define MAX1161X_CONFIG_SGL_MASK  (1 << 0)    /**< Single-ended/Differential mode */

#define MAX1161X_CONFIG_REG          (0 << 7) /**< Config byte identifier */
#define MAX1161X_CONFIG_DIFFERENTIAL (0 << 0) /**< Differential mode */
#define MAX1161X_CONFIG_SINGLE_ENDED (1 << 0) /**< Single-ended mode */
/** @} */

/**
 * @name    Configuration Byte SCAN[1:0] values
 * @{
 */
#define MAX1161X_CONFIG_SCAN_SINGLE        (0 << 5) /**< Single conversion of selected channel */
#define MAX1161X_CONFIG_SCAN_0_TO_N        (1 << 5) /**< Scan from CH0 to selected channel */
#define MAX1161X_CONFIG_SCAN_REPEAT_SINGLE (2 << 5) /**< Repeated conversion of selected channel */
#define MAX1161X_CONFIG_SCAN_REPEAT_0_TO_N (3 << 5) /**< Repeated scan CH0 to selected channel */
/** @} */

/**
 * @name    Channel selection values
 * @{
 */
#define MAX1161X_CONFIG_CH0 (0 << 1) /**< Channel 0 */
#define MAX1161X_CONFIG_CH1 (1 << 1) /**< Channel 1 */
#define MAX1161X_CONFIG_CH2 (2 << 1) /**< Channel 2 */
#define MAX1161X_CONFIG_CH3 (3 << 1) /**< Channel 3 */
#define MAX1161X_CONFIG_CH4 (4 << 1) /**< Channel 4 */
#define MAX1161X_CONFIG_CH5 (5 << 1) /**< Channel 5 */
#define MAX1161X_CONFIG_CH6 (6 << 1) /**< Channel 6 */
#define MAX1161X_CONFIG_CH7 (7 << 1) /**< Channel 7 */
/** @} */

/**
 * @name    Common configurations
 * @{
 */
#define MAX1161X_SETUP_DEFAULT                                                                                         \
	(MAX1161X_SETUP_REG | MAX1161X_SETUP_SEL_INT_REF | MAX1161X_SETUP_CLK_INTERNAL | MAX1161X_SETUP_UNIPOLAR |     \
	 MAX1161X_SETUP_NO_ACTION)

#define MAX1161X_CONFIG_DEFAULT                                                                                        \
	(MAX1161X_CONFIG_REG | MAX1161X_CONFIG_SCAN_SINGLE | MAX1161X_CONFIG_CH0 | MAX1161X_CONFIG_SINGLE_ENDED)
/** @} */

/**
 * @name    Data format constants
 * @{
 */
#define MAX1161X_DATA_RESOLUTION  12   /**< 12-bit resolution */
#define MAX1161X_DATA_MAX_VALUE   4095 /**< Maximum ADC value (2^12 - 1) */
#define MAX1161X_DATA_MSB_SHIFT   4    /**< Left shift for MSB */
#define MAX1161X_DATA_LSB_SHIFT   4    /**< Right shift for LSB */
#define MAX1161X_VREF_INTERNAL_MV 2048 /**< Internal reference voltage in mV */
/** @} */

/**
 * @name    Timing constants
 * @{
 */
#define MAX1161X_CONVERSION_TIME_US    9   /**< Typical conversion time in µs */
#define MAX1161X_VREF_SETTLING_TIME_US 100 /**< Reference settling time in µs */
/** @} */

#ifdef __cplusplus
}
#endif

#endif /* MAX1161X_REGS_H */
       /** @} */
