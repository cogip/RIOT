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
 * @brief       ADCxx1C adaption to the RIOT actuator/sensor interface
 *
 * @author      Mathis Lécrivain <lecrivain.mathis@gmail.com>
 *
 * @}
 */

#include <string.h>
#include <stdio.h>

#include "saul.h"
#include "max1161x.h"

static int read_adc(const void *dev, phydat_t *res)
{
	if (max1161x_read_raw((const max1161x_t *)dev, &res->val[0])) {
		return -ECANCELED;
	}

	res->unit = UNIT_NONE;
	res->scale = 0;

	return 1;
}

const saul_driver_t max1161x_saul_driver = {
	.read = read_adc,
	.write = saul_write_notsup,
	.type = SAUL_SENSE_ANALOG,
};
