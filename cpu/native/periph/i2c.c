/*
 * Copyright (C) 2017 Kaspar Schleiser <kaspar@schleiser.de>
 *               2014 FU Berlin
 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 */

/**
 * @ingroup     cpu_stm32f4
 * @ingroup     drivers_periph_i2c
 * @{
 *
 * @file
 * @brief       Low-level I2C driver implementation
 *
 * @note This implementation only implements the 7-bit addressing mode.
 *
 * @author      Peter Kietzmann <peter.kietzmann@haw-hamburg.de>
 * @author      Hauke Petersen <hauke.petersen@fu-berlin.de>
 * @author      Thomas Eichinger <thomas.eichinger@fu-berlin.de>
 * @author      Kaspar Schleiser <kaspar@schleiser.de>
 *
 * @}
 */

#include <stdint.h>

#include "cpu.h"
#include "irq.h"
#include "mutex.h"
#include "periph_conf.h"
#include "periph/i2c.h"

#define ENABLE_DEBUG    (0)
#include "debug.h"

/* static function definitions */
#if 0
static void _i2c_init(I2C_TypeDef *i2c, int ccr);
static void _toggle_pins(GPIO_TypeDef *port_scl, GPIO_TypeDef *port_sda, int pin_scl, int pin_sda);
static void _pin_config(GPIO_TypeDef *port_scl, GPIO_TypeDef *port_sda, int pin_scl, int pin_sda);
static void _start(I2C_TypeDef *dev, uint8_t address, uint8_t rw_flag);
static inline void _clear_addr(I2C_TypeDef *dev);
static inline void _write(I2C_TypeDef *dev, const uint8_t *data, int length);
static inline void _stop(I2C_TypeDef *dev);
#endif


void i2c_init(i2c_t dev)
{
	(void)dev;
}

#if 0
static void _i2c_init(I2C_TypeDef *i2c, int ccr)
{
    /* disable device and set ACK bit */
    i2c->CR1 = I2C_CR1_ACK;
    /* configure I2C clock */
    i2c->CR2 = (I2C_APBCLK / 1000000);
    i2c->CCR = ccr;
    i2c->TRISE = (I2C_APBCLK / 1000000) + 1;
    /* configure device */
    i2c->OAR1 = 0;              /* makes sure we are in 7-bit address mode */
    /* enable device */
    i2c->CR1 |= I2C_CR1_PE;
}

static void _pin_config(GPIO_TypeDef *port_scl, GPIO_TypeDef *port_sda, int pin_scl, int pin_sda)
{
    /* Set GPIOs to AF mode */
    port_scl->MODER &= ~(3 << (2 * pin_scl));
    port_scl->MODER |= (2 << (2 * pin_scl));
    port_sda->MODER &= ~(3 << (2 * pin_sda));
    port_sda->MODER |= (2 << (2 * pin_sda));

    /* Set speed high*/
    port_scl->OSPEEDR |= (3 << (2 * pin_scl));
    port_sda->OSPEEDR |= (3 << (2 * pin_sda));

    /* Set to push-pull configuration open drain*/
    port_scl->OTYPER |= (1 << pin_scl);
    port_sda->OTYPER |= (1 << pin_sda);

    /* Enable pull-up resistors */
    port_scl->PUPDR &= ~(3 << (2 * pin_scl));
    port_scl->PUPDR |= (1 << (2 * pin_scl));
    port_sda->PUPDR &= ~(3 << (2 * pin_sda));
    port_sda->PUPDR |= (1 << (2 * pin_sda));

    /* Configure GPIOs to for the I2C alternate function */
    if (pin_scl < 8) {
        port_scl->AFR[0] &= ~(0xf << (4 * pin_scl));
        port_scl->AFR[0] |= (I2C_0_SCL_AF << (4 * pin_scl));
    }
    else {
        port_scl->AFR[1] &= ~(0xf << (4 * (pin_scl - 8)));
        port_scl->AFR[1] |= (I2C_0_SCL_AF << (4 * (pin_scl - 8)));
    }

    if (pin_sda < 8) {
        port_sda->AFR[0] &= ~(0xf << (4 * pin_sda));
        port_sda->AFR[0] |= (I2C_0_SDA_AF << (4 * pin_sda));
    }
    else {
        port_sda->AFR[1] &= ~(0xf << (4 * (pin_sda - 8)));
        port_sda->AFR[1] |= (I2C_0_SDA_AF << (4 * (pin_sda - 8)));
    }
}

static void _toggle_pins(GPIO_TypeDef *port_scl, GPIO_TypeDef *port_sda, int pin_scl, int pin_sda)
{
    /* Set GPIOs to General purpose output mode mode */
    port_scl->MODER &= ~(3 << (2 * pin_scl));
    port_scl->MODER |= (1 << (2 * pin_scl));
    port_sda->MODER &= ~(3 << (2 * pin_sda));
    port_sda->MODER |= (1 << (2 * pin_sda));

    /* Set speed high*/
    port_scl->OSPEEDR |= (3 << (2 * pin_scl));
    port_sda->OSPEEDR |= (3 << (2 * pin_sda));

    /* Set to push-pull configuration open drain*/
    port_scl->OTYPER |= (1 << pin_scl);
    port_sda->OTYPER |= (1 << pin_sda);

    /* set both to high */
    port_scl->ODR |= (1 << pin_scl);
    port_sda->ODR |= (1 << pin_sda);
    /* set SDA to low */
    port_sda->ODR &= ~(1 << pin_sda);
    /* set SCL to low */
    port_scl->ODR &= ~(1 << pin_scl);
    /* set SCL to high */
    port_scl->ODR |= (1 << pin_scl);
    /* set SDA to high */
    port_sda->ODR |= (1 << pin_sda);
}
#endif

int i2c_acquire(i2c_t dev)
{
	(void) dev;
	return 0;
}

int i2c_release(i2c_t dev)
{
	(void) dev;
	return 0;
}

int i2c_read_bytes(i2c_t dev, uint16_t address, void *data, size_t length,
                   uint8_t flags)
{
	(void) dev;
	(void) address;
	(void) data;
	(void) length;
	(void) flags;
	return 0;
}

int i2c_read_regs(i2c_t dev, uint16_t address, uint16_t reg, void *data,
                  size_t length, uint8_t flags)
{
	(void)dev;
	(void)address;
	(void)reg;
	(void)data;
	(void) length;
	(void) flags;
	return 0;
}

int i2c_write_bytes(i2c_t dev, uint16_t address, const void *data,
                    size_t length, uint8_t flags)
{
	(void) dev;
	(void) address;
	(void) data;
	(void) length;
	(void) flags;
	return 0;
}

int i2c_write_regs(i2c_t dev, uint16_t address, uint16_t reg, const void *data,
                   size_t length, uint8_t flags)
{
	(void)dev;
	(void)address;
	(void)reg;
	(void)data;
	(void)length;
	(void) flags;
	return 0;
}

