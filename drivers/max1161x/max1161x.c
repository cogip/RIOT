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
 * @brief       MAX1161X ADC device driver
 *
 * @author      Mathis Lécrivain <lecrivain.mathis@gmail.com>
 * @}
 */

#include <assert.h>

#include "max1161x.h"
#include "max1161x_params.h"
#include "max1161x_regs.h"

#include "periph/i2c.h"
#include "periph/gpio.h"

#define ENABLE_DEBUG 0
#include "debug.h"

#define DEV  (dev->params.i2c)
#define ADDR (dev->params.addr)

int max1161x_init(max1161x_t *dev, const max1161x_params_t *params)
{
	int status;
	assert(dev && params);

	dev->params = *params;

	/* Prepare setup byte. */
	dev->setup_byte.reset = 1;
	dev->setup_byte.unipolar = params->unipolar;
	dev->setup_byte.clock = params->clock;
	dev->setup_byte.sel = params->sel;
	dev->setup_byte.reg = 1;

	/* Prepare config byte. */
	dev->config_byte.sgldiff = params->sgldiff;
	dev->config_byte.cs = params->channel;
	dev->config_byte.scan = params->scan;
	dev->config_byte.reg = 0;

	i2c_acquire(DEV);
	/* Write setup byte. */
	status = i2c_write_bytes(DEV, ADDR, dev->setup_byte.value, sizeof(dev->setup_byte.value));
	if (status < 0) {
		i2c_release(DEV);
		DEBUG("[max1161x] init - error: unable to write setup byte\n");
		return MAX1161X_NODEV; // TODO:
	}

	/* Write configuration byte. */
	status = i2c_write_bytes(DEV, ADDR, dev->config_byte.value, sizeof(dev->config_byte.value));
	if (status < 0) {
		i2c_release(DEV);
		DEBUG("[max1161x] init - error: unable to write config byte\n");
		return MAX1161X_NODEV; // TODO:
	}
	i2c_release(DEV);

	return 0;
}

int max1161x_reset(max1161x_t *dev, const max1161x_params_t *params)
{
	int status;

	assert(dev && params);

	/* Reset setup byte and write setup byte. */
	dev->setup_byte.reset = 0; /** Also resets the configuration register to default */
	dev->setup_byte.unipolar = MAX1161X_UNIPOLAR;
	dev->setup_byte.clock = MAX1161X_CLOCK_DEFAULT;
	dev->setup_byte.sel = MAX1161X_REF_DEFAULT;
	dev->setup_byte.reg = 1;

	i2c_acquire(DEV);
	status = i2c_write_bytes(DEV, ADDR, dev->setup_byte.value, sizeof(dev->setup_byte.value));
	i2c_release(DEV);
	if (status < 0) {
		DEBUG("[max1161x] init - error: unable to write setup byte\n", reg);
		return MAX1161X_NODEV; // TODO:
	}

	/* Reset config byte. No need to write it as `setup_byte.reset` flag
	 * internally reset the configuration register.
	 */
	dev->config_byte.sgldiff = MAX1161X_SINGLE_ENDED;
	dev->config_byte.cs = MAX1161X_CHANNEL_DEFAULT;
	dev->config_byte.scan = MAX1161X_SCAN_DEFAULT;
	dev->config_byte.reg = 0;
}

int max1161x_read_raw(const max1161x_t *dev, int16_t *raw)
{
	uint8_t buf[2];

	assert(dev && raw);

	if (dev->config_byte.scan != MAX1161X_SCAN_NONE && dev->config_byte.scan != MAX1161X_SCAN_EIGHT_TIMES) {
		DEBUG("[max1161x] read - error: invalid scan mode for single channel read operation\n");
		return MAX1161X_NODEV; // TODO:
	}

	i2c_acquire(DEV);
	int status = i2c_read_byte(DEV, ADDR, buf, sizeof(buf), 0);
	i2c_release(DEV);
	if (status < 0) {
		return MAX1161X_NOI2C;
	}

	*raw = ((buf[0] & 0x0F) << 8 | buf[1]) & 0x0FFF;

	return MAX1161X_OK;
}

int max1161x_read_channel_raw(const max1161x_t *dev, max1161x_channel_t chan, int16_t *raw)
{
	assert(dev && raw);

	if (dev->config_byte.scan != MAX1161X_SCAN_NONE && dev->config_byte.scan != MAX1161X_SCAN_EIGHT_TIMES) {
		DEBUG("[max1161x] read - error: invalid scan mode for single channel read operation\n");
		return MAX1161X_NODEV; // TODO:
	}

	/* Update config byte. */
	dev->config_byte.cs = chan;

	/* Write configuration byte. */
	i2c_acquire(DEV);
	int status = i2c_write_bytes(DEV, ADDR, dev->config_byte.value, sizeof(dev->config_byte.value));
	i2c_release(DEV);
	if (status < 0) {
		DEBUG("[max1161x] init - error: unable to write config byte\n");
		return MAX1161X_NODEV; // TODO:
	}

	return max1161x_read_raw(dev, raw)
}

int max1161x_add_channel(max1161x_t *dev, max1161x_channel_t chan)
{
	assert(dev);

	/* Update config byte. */
	dev->config_byte.cs |= (chan & 0b111);

	/* Write configuration byte. */
	i2c_acquire(DEV);
	int status = i2c_write_bytes(DEV, ADDR, dev->config_byte.value, sizeof(dev->config_byte.value));
	i2c_release(DEV);
	if (status < 0) {
		DEBUG("[max1161x] init - error: unable to write config byte\n");
		return MAX1161X_NODEV; // TODO:
	}
}

int max1161x_remove_channel(max1161x_t *dev, max1161x_channel_t chan)
{
	assert(dev);

	/* Update config byte. */
	dev->config_byte.cs &= ~(chan & 0b111);

	/* Write configuration byte. */
	i2c_acquire(DEV);
	int status = i2c_write_bytes(DEV, ADDR, dev->config_byte.value, sizeof(dev->config_byte.value));
	i2c_release(DEV);
	if (status < 0) {
		DEBUG("[max1161x] init - error: unable to write config byte\n");
		return MAX1161X_NODEV; // TODO:
	}
}
