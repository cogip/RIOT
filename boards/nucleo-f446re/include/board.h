/*
 * Copyright (C) 2018 Gilles DOFFE <g.doffe@gmail.com>
 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 */

/**
 * @defgroup    boards_nucleo-f446re STM32 Nucleo-F446RE
 * @ingroup     boards_common_nucleo64
 * @brief       Support for the STM32 Nucleo-F446RE
 * @{
 *
 * @file
 * @brief       Common pin definitions and board configuration options
 *
 * @author      Gilles DOFFE <g.doffe@gmail.com>
 */

#ifndef BOARD_H
#define BOARD_H

#include "board_nucleo.h"
#include "motor_driver.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Describe DC motors with PWM channel and GPIOs
 */
static const motor_driver_config_t motor_driver_config[] = {
    {
        .mode            = MOTOR_DRIVER_1_DIR,
        .mode_brake      = MOTOR_BRAKE_LOW,
        .pwm_dev         = 1,
        .pwm_frequency   = 20000U,
        .pwm_resolution  = 2250U,
        .nb_motors       = 2,
        .motors          = {
            {
                .pwm_channel            = 0,
                .gpio_enable            = GPIO_UNDEF,
                .gpio_dir0              = GPIO_PIN(PORT_A, 11),
                .gpio_dir1_or_brake     = GPIO_UNDEF,
                .gpio_dir_reverse       = 0,
                .gpio_enable_invert     = 0,
                .gpio_brake_invert      = 0,
            },
            {
                .pwm_channel            = 2,
                .gpio_enable            = GPIO_UNDEF,
                .gpio_dir0              = GPIO_PIN(PORT_A, 12),
                .gpio_dir1_or_brake     = GPIO_UNDEF,
                .gpio_dir_reverse       = 0,
                .gpio_enable_invert     = 0,
                .gpio_brake_invert      = 0,
            },
        },
        .cb = NULL,
    },
};

#define MOTOR_DRIVER_NUMOF           (sizeof(motor_driver_config) / sizeof(motor_driver_config[0]))

#ifdef __cplusplus
}
#endif

#endif /* BOARD_H */
/** @} */
