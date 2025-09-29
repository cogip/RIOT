/*
 * Copyright (C) 2025 Mathis LECRIVAIN <lecrivain.mathis@gmail.com>
 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 */

/**
 * @defgroup   drivers_max1161x MAX1161X ADC device driver
 * @ingroup    drivers_sensors
 * @ingroup    drivers_saul
 * @brief      I2C Analog-to-Digital Converter device driver
 *
 * This driver works with max11612, max11613, max11614, max11615, max11616 and max11617 versions.
 *
 * This driver provides @ref drivers_saul capabilities.
 * @{
 *
 * @file
 * @brief      MAX1161X ADC device driver
 *
 * @author     Mathis Lécrivain <lecrivain.mathis@gmail.com>
 */

#ifndef MAX1161X_H
#define MAX1161X_H

#ifdef __cplusplus
extern "C" {
#endif

#include "periph/i2c.h"

/**
 * @brief MAX1161X default address default resolution for device variants
 */
#if defined(MODULE_MAX11612) || defined(MODULE_MAX11613)
#define MAX1161X_I2C_ADDRESS (0x34)
#elif defined(MODULE_MAX11614) || defined(MODULE_MAX11615)
#define MAX1161X_I2C_ADDRESS (0x33)
#elif defined(MODULE_MAX11616) || defined(MODULE_MAX11617)
#define MAX1161X_I2C_ADDRESS (0x35)
#else
#define MAX1161X_I2C_ADDRESS (-1)
#error "MAX1161X: Failed to select default address: unknown MAX1161X device variant!"
#endif

/**
 * @brief MAX1161X channel number resolution for device variants
 */
#if defined(MODULE_MAX11612) || defined(MODULE_MAX11613)
#define MAX1161X_NUM_CHANNELS (4)
#elif defined(MODULE_MAX11614) || defined(MODULE_MAX11615)
#define MAX1161X_NUM_CHANNELS (8)
#elif defined(MODULE_MAX11616) || defined(MODULE_MAX11617)
#define MAX1161X_NUM_CHANNELS (12)
#else
#define MAX1161X_RES_DEFAULT (-1)
#error "MAX1161X: Failed to select channel number: unknown MAX1161X device variant!"
#endif

/**
 * @brief Reference voltage. Table 6. Reference Voltage, AIN_/REF, and REF Format.
 */
typedef enum {
	MAX1161X_REF_VDD_AIN_NC_OFF = 0b0000,
	MAX1161X_REF_EXT_RIN_IN_OFF = 0b0010,
	MAX1161X_REF_INT_AIN_NC_OFF = 0b0100,
	MAX1161X_REF_INT_AIN_NC_ON = 0b0101,
	MAX1161X_REF_INT_ROUT_OUT_OFF = 0b0110,
	MAX1161X_REF_INT_ROUT_OUT_ON = 0b0111,

	/**< Default reference */
	MAX1161X_REF_DEFAULT = MAX1161X_REF_VDD_AIN_NC_OFF,
} max1161x_ref_t;

/**
 * @brief Clock Modes.
 */
typedef enum {
	MAX1161X_CLOCK_INT = 0b0,
	MAX1161X_CLOCK_EXT = 0b1,

	/**< Default clock*/
	MAX1161X_CLOCK_DEFAULT = MAX1161X_CLOCK_INT,
} max1161x_clock_t;

/**
 * @brief unipolar/bipolar selection.
 */
typedef enum {
	MAX1161X_UNIPOLAR = 0b0,
	MAX1161X_BIPOLAR = 0b1
} max1161x_unibipolar_t;

/**
 * @brief Scan mode. Table 5. Scanning Configuration.
 */
typedef enum {
	MAX1161X_SCAN_FROM_ZERO = 0b00,
	MAX1161X_SCAN_EIGHT_TIMES = 0b01,
	MAX1161X_SCAN_UPPER = 0b10,
	MAX1161X_SCAN_NONE = 0b11,

	/**< Default clock*/
	MAX1161X_SCAN_DEFAULT = MAX1161X_SCAN_FROM_ZERO,
} max1161x_scan_t;

/**
 * @brief Channel selection.
 *
 * @note Both Single-Ended and Differential mode share the same enum.
 *       CS[0,3] values are the same for both modes, but have different meanings.
 *       The correct meaning is selected by the SGL/DIF bit in the config byte.
 */
typedef enum {
	/**< Table 3. Channel Selection in Single-Ended Mode (SGL/DIF = 1). */
	MAX1161X_CHANNEL_CH0 = 0b0000,  /**< CH0, REF */
	MAX1161X_CHANNEL_CH1 = 0b0001,  /**< CH1, REF */
	MAX1161X_CHANNEL_CH2 = 0b0010,  /**< CH2, REF */
	MAX1161X_CHANNEL_CH3 = 0b0011,  /**< CH3, REF */
	MAX1161X_CHANNEL_CH4 = 0b0100,  /**< CH4, REF */
	MAX1161X_CHANNEL_CH5 = 0b0101,  /**< CH5, REF */
	MAX1161X_CHANNEL_CH6 = 0b0110,  /**< CH6, REF */
	MAX1161X_CHANNEL_CH7 = 0b0111,  /**< CH7, REF */
	MAX1161X_CHANNEL_CH8 = 0b1000,  /**< CH8, REF */
	MAX1161X_CHANNEL_CH9 = 0b1001,  /**< CH9, REF */
	MAX1161X_CHANNEL_CH10 = 0b1010, /**< CH10, REF */
	MAX1161X_CHANNEL_CH11 = 0b1011, /**< CH11, REF */

	/**< Table 4. Channel Selection in Differential Mode (SGL/DIF = 0). */
	MAX1161X_CHANNEL_CH0_CH1 = 0b0000,   /**< +CH0, -CH1 */
	MAX1161X_CHANNEL_CH1_CH0 = 0b0001,   /**< +CH1, -CH0 */
	MAX1161X_CHANNEL_CH2_CH3 = 0b0010,   /**< +CH2, -CH3 */
	MAX1161X_CHANNEL_CH3_CH2 = 0b0011,   /**< +CH3, -CH2 */
	MAX1161X_CHANNEL_CH4_CH5 = 0b0100,   /**< +CH4, -CH5 */
	MAX1161X_CHANNEL_CH5_CH4 = 0b0101,   /**< +CH5, -CH4 */
	MAX1161X_CHANNEL_CH6_CH7 = 0b0110,   /**< +CH6, -CH7 */
	MAX1161X_CHANNEL_CH7_CH6 = 0b0111,   /**< +CH7, -CH6 */
	MAX1161X_CHANNEL_CH8_CH9 = 0b1000,   /**< +CH8, -CH9 */
	MAX1161X_CHANNEL_CH9_CH8 = 0b1001,   /**< +CH9, -CH8 */
	MAX1161X_CHANNEL_CH10_CH11 = 0b1010, /**< +CH10, -CH11 */
	MAX1161X_CHANNEL_CH11_CH10 = 0b1011, /**< +CH11, -CH10 */

	/**< Default channel*/
	MAX1161X_CHANNEL_DEFAULT = MAX1161X_CHANNEL_CH0,
} max1161x_channel_t;

/**
 * @brief Single-ended/Differential selection.
 */
typedef enum {
	MAX1161X_DIFFERENTIAL = 0b0,
	MAX1161X_SINGLE_ENDED = 0b1
} max1161x_sgldiff_t;

/**
 * @brief   Named return values TODO:
 */
enum {
	MAX1161X_OK = 0,     /**< everything was fine */
	MAX1161X_NOI2C = -1, /**< I2C communication failed */
	MAX1161X_NODEV = -2, /**< no MAX1161X device found on the bus */
	MAX1161X_NODATA = -3 /**< no data available */
};

/**
 * @brief MAX1161X params
 */
typedef struct max1161x_params {
	i2c_t i2c;                        /**< i2c device */
	uint8_t addr;                     /**< i2c address */
	max1161x_ref_t ref;               /**< reference selection */
	max1161x_clock_t clock;           /**< clock source selection */
	max1161x_unibipolar_t unibipolar; /**< unipolar/bipolar selection */
	max1161x_scan_t scan;             /**< scan mode selection */
	max1161x_channel_t channel;       /**< Selected channel*/
	max1161x_sgldiff_t sgldiff;       /**< single-ended/differential selection */
} max1161x_params_t;

/**
 * @brief  MAX1161X setup byte.
 *
 * @note For internal use only.
 */
typedef union {
	struct __attribute__((packed)) {
		uint8_t reserved: 1;   /**< Reserved. This bit can be set to 1 or 0. */
		uint8_t reset: 1;      /**< 1 = no action, 0 = resets the configuration register to default. */
		uint8_t unibipolar: 1; /**< 1 = bipolar, 0 = unipolar. Defaults to 0 at power-up. */
		uint8_t clock: 1;      /**< 1 = external clock, 0 = internal clock. Defaults to 0 at power-up. */
		uint8_t sel: 3;        /**< Three bits select the reference voltage and the state of AIN_/REF. */
		uint8_t reg: 1;        /**< 1 = setup byte. This bit should be 1. */
	};
	uint8_t value; /**< raw setup byte value */
} max1161x_setup_byte_t;

/**
 * @brief  MAX1161X config byte.
 *
 * @note For internal use only.
 */
typedef union {
	struct __attribute__((packed)) {
		uint8_t sgldiff: 1; /**< 1 = single-ended, 0 = differential. Defaults to 1 at power-up. */
		uint8_t cs: 4;      /**< Channel select bits. Default to 0000 at power-up. */
		uint8_t scan: 2;    /**< Scan select bits. Default to 00 at power-up. */
		uint8_t reg: 1;     /**< 0 = config byte. This bit should be 0. */
	};
	uint8_t value; /**< raw config byte value */
} max1161x_config_byte_t;

/**
 * @brief   MAX1161X device descriptor
 */
typedef struct max1161x {
	max1161x_params_t params;           /**< device driver configuration */
	max1161x_setup_byte_t setup_byte;   /**< device setup byte */
	max1161x_config_byte_t config_byte; /**< device config byte */
} max1161x_t;

/**
 * @brief Initialize an MAX1161X ADC device
 *
 * @param[in,out] dev  device descriptor
 * @param[in] params   device configuration
 *
 * @return zero on successful initialization, non zero on error
 */
int max1161x_init(max1161x_t *dev, const max1161x_params_t *params);

/**
 * @brief Reset an MAX1161X ADC device
 *
 * @param[in,out] dev  device descriptor
 * @param[in] params  device configuration
 *
 * @return zero on successful initialization, non zero on error
 */
int max1161x_reset(max1161x_t *dev, const max1161x_params_t *params);

/**
 * @brief Read a raw ADC value as configured in driver parameters
 *
 * @param[in] dev   device descriptor
 * @param[out] raw  read value
 *
 * @return zero on successful read, non zero on error
 */
int max1161x_read_raw(const max1161x_t *dev, int16_t *raw);

/**
 * @brief Read a raw ADC value from a specific channel
 *
 * @note The device must be configured in single channel mode (no scan).
 *
 * @warning The selected channel remains selected after the read operation.
 *
 * @param[in] dev   device descriptor
 * @param[in] chan  channel to read
 * @param[out] raw  read value
 *
 * @return zero on successful read, non zero on error
 */
int max1161x_read_channel_raw(const max1161x_t *dev, max1161x_channel_t chan, int16_t *raw);

#ifdef __cplusplus
}
#endif

#endif /* MAX1161X_H */
       /** @} */
