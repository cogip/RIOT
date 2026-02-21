/*
 * Copyright (C) 2020 Mesotic SAS
 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 */

#pragma once

/**
 * @defgroup    sys_riotboot_magic     Magic values for riotboot
 * @ingroup     sys
 * @{
 *
 * @file
 * @brief       Magic values and reset API for riotboot
 *
 * @author      Dylan Laduranty <dylan.laduranty@mesotic.com>
 * @author      Mathis Lécrivain <lecrivain.mathis@gmail.com>
 *
 * @}
 */

#include "riotboot/hdr.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @name RAM magic values for riotboot
 * @{
 */
#ifndef RIOTBOOT_MAGIC_ADDR
#  define RIOTBOOT_MAGIC_ADDR (CPU_RAM_BASE + CPU_RAM_SIZE - 4) /**< default magic address */
#endif
#ifndef RIOTBOOT_MAGIC_NUMBER
#  define RIOTBOOT_MAGIC_NUMBER RIOTBOOT_MAGIC /**< default magic value */
#endif
/** @} */

/**
 * @brief   Reboot into the riotboot bootloader
 *
 * Writes @ref RIOTBOOT_MAGIC to @ref RIOTBOOT_MAGIC_ADDR and
 * triggers a reboot. This is transport-agnostic and works for
 * USB, serial, CAN, or any other transport.
 */
void riotboot_reset_to_bootloader(void);

#ifdef __cplusplus
}
#endif
