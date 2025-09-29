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
 * @brief       Default configuration for MAX1161X devices
 *
 * @author      Mathis Lécrivain <lecrivain.mathis@gmail.com>
 * @}
 */

#ifndef MAX1161X_PARAMS_H
#define MAX1161X_PARAMS_H

#include "board.h"
#include "saul_reg.h"
#include "max1161x.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @defgroup drivers_max1161x_config   MAX1161X driver compile configurations
 * @ingroup drivers_max1161x
 * @ingroup config_drivers_sensors
 * @{
 */

/** @brief  I2C device to use */
#ifndef MAX1161X_PARAM_I2C
#define MAX1161X_PARAM_I2C (I2C_DEV(0))
#endif

/** @brief  I2C address */
#ifndef MAX1161X_PARAM_ADDR
#define MAX1161X_PARAM_ADDR (MAX1161X_I2C_ADDRESS)
#endif

/** @brief  Resolution in bits */
#ifndef MAX1161X_PARAM_BITS
#define MAX1161X_PARAM_BITS (MAX1161X_RES_DEFAULT)
#endif

/** @brief  Conversion interval */
#ifndef MAX1161X_PARAM_CYCLE
#define MAX1161X_PARAM_CYCLE (MAX1161X_CYCLE_DISABLED)
#endif

/** @brief  Alert gpio pin */
#ifndef MAX1161X_PARAM_ALERT_PIN
#define MAX1161X_PARAM_ALERT_PIN (GPIO_UNDEF)
#endif

/** @brief  Low limit for the alert */
#ifndef MAX1161X_PARAM_LOW_LIMIT
#define MAX1161X_PARAM_LOW_LIMIT (0)
#endif

/** @brief  High limit for the alert */
#ifndef MAX1161X_PARAM_HIGH_LIMIT
#define MAX1161X_PARAM_HIGH_LIMIT (0)
#endif

/** @brief  Hysteresis for the alert */
#ifndef MAX1161X_PARAM_HYSTERESIS
#define MAX1161X_PARAM_HYSTERESIS (0)
#endif
/** @} */

/**
 * @brief   MAX1161X driver configuration structures
 */
#ifndef MAX1161X_PARAMS
#define MAX1161X_PARAMS                                                                                                \
	{                                                                                                              \
		.i2c = MAX1161X_PARAM_I2C,                                                                             \
		.addr = MAX1161X_PARAM_ADDR,                                                                           \
		.bits = MAX1161X_PARAM_BITS,                                                                           \
		.cycle = MAX1161X_PARAM_CYCLE,                                                                         \
		.alert_pin = MAX1161X_PARAM_ALERT_PIN,                                                                 \
		.low_limit = MAX1161X_PARAM_LOW_LIMIT,                                                                 \
		.high_limit = MAX1161X_PARAM_HIGH_LIMIT,                                                               \
		.hysteresis = MAX1161X_PARAM_HYSTERESIS,                                                               \
	}
#endif

/**
 * @brief   MAX1161X driver SAUL registry information structures
 */
#ifndef MAX1161X_SAUL_INFO
#define MAX1161X_SAUL_INFO {.name = "max1161x"}
#endif

/**
 * @brief   MAX1161X configuration
 */
static const max1161x_params_t max1161x_params[] = {MAX1161X_PARAMS};

/**
 * @brief   Additional meta information to keep in the SAUL registry
 */
static const saul_reg_info_t max1161x_saul_info[] = {MAX1161X_SAUL_INFO};

#ifdef __cplusplus
}
#endif

#endif /* MAX1161X_PARAMS_H */
