/*
 * SPDX-FileCopyrightText: 2018 Inria
 * SPDX-License-Identifier: LGPL-2.1-only
 */

/**
 * @ingroup     cpu_stm32
 * @{
 *
 * @file
 * @brief       Low-level flash lock/unlock implementation
 *
 * @author      Alexandre Abadie <alexandre.abadie@inria.fr>
 * @author      Oleg Artamonov <oleg@unwds.com>
 *
 * @}
 */

#include "cpu.h"

#define ENABLE_DEBUG           0
#include "debug.h"

#if defined(CPU_FAM_STM32L0) || defined(CPU_FAM_STM32L1)
/* Data EEPROM and control register unlock keys */
#define FLASH_KEY1             ((uint32_t)0x89ABCDEF)
#define FLASH_KEY2             ((uint32_t)0x02030405)
#define CNTRL_REG              (FLASH->PECR)
#define CNTRL_REG_LOCK         (FLASH_PECR_PELOCK)
#define KEY_REG                (FLASH->PEKEYR)
#elif defined(CPU_FAM_STM32L5) || defined(CPU_FAM_STM32U5)
#define FLASH_KEY1             ((uint32_t)0x45670123)
#define FLASH_KEY2             ((uint32_t)0xCDEF89AB)
#define CNTRL_REG              (FLASH->NSCR)
#if defined(CPU_FAM_STM32U5)
#define CNTRL_REG_LOCK         (FLASH_NSCR_LOCK)
#define KEY_REG                (FLASH->NSKEYR)
#define FLASH_SR_EOP           (FLASH_NSSR_EOP)
#else
#define CNTRL_REG_LOCK         (FLASH_NSCR_NSLOCK)
#define KEY_REG                (FLASH->NSKEYR)
#define FLASH_SR_EOP           (FLASH_NSSR_NSEOP)
#endif
#elif defined(CPU_FAM_STM32H5)
/* STM32H5 (TrustZone disabled): use the non-secure flash registers. The
 * control/status/key registers are NSCR/NSSR/NSKEYR while the bitfield
 * macros keep the generic FLASH_CR_ and FLASH_SR_ names. */
#define FLASH_KEY1             ((uint32_t)0x45670123)
#define FLASH_KEY2             ((uint32_t)0xCDEF89AB)
#define CNTRL_REG              (FLASH->NSCR)
#define CNTRL_REG_LOCK         (FLASH_CR_LOCK)
#define KEY_REG                (FLASH->NSKEYR)
#else
#if defined(CPU_FAM_STM32L4) || defined(CPU_FAM_STM32WB) || \
    defined(CPU_FAM_STM32G4) || defined(CPU_FAM_STM32G0) || \
    defined(CPU_FAM_STM32F2) || defined(CPU_FAM_STM32F4) || \
    defined(CPU_FAM_STM32F7) || defined(CPU_FAM_STM32WL) || \
    defined(CPU_FAM_STM32C0)
#define FLASH_KEY1             ((uint32_t)0x45670123)
#define FLASH_KEY2             ((uint32_t)0xCDEF89AB)
#endif
#define CNTRL_REG              (FLASH->CR)
#define CNTRL_REG_LOCK         (FLASH_CR_LOCK)
#define KEY_REG                (FLASH->KEYR)
#endif

#if defined(CPU_FAM_STM32G0) || defined(CPU_FAM_STM32C0)
#define FLASH_SR_BSY           (FLASH_SR_BSY1)
#endif

#if defined(CPU_FAM_STM32L5)
#define FLASH_SR_BSY            (FLASH_NSSR_NSBSY)
#define FLASH_SR_REG            (FLASH->NSSR)
#elif defined(CPU_FAM_STM32U5)
#define FLASH_SR_BSY            (FLASH_NSSR_BSY)
#define FLASH_SR_REG            (FLASH->NSSR)
#elif defined(CPU_FAM_STM32H5)
/* FLASH_SR_BSY is provided natively by the CMSIS header (NSSR bit) */
#define FLASH_SR_REG            (FLASH->NSSR)
#else
#define FLASH_SR_REG            (FLASH->SR)
#endif

/* Flash status register error flags */
#if defined(CPU_FAM_STM32G4)
#  define FLASH_SR_ERRORS        (FLASH_SR_OPERR | FLASH_SR_PROGERR | \
                                  FLASH_SR_WRPERR | FLASH_SR_PGAERR | \
                                  FLASH_SR_SIZERR | FLASH_SR_PGSERR | \
                                  FLASH_SR_MISERR | FLASH_SR_FASTERR | \
                                  FLASH_SR_RDERR | FLASH_SR_OPTVERR)
#endif

void _unlock(void)
{
    if (CNTRL_REG & CNTRL_REG_LOCK) {
        DEBUG("[flash-common] unlocking the flash module\n");
        KEY_REG = FLASH_KEY1;
        KEY_REG = FLASH_KEY2;
    }
}

void _lock(void)
{
    if (!(CNTRL_REG & CNTRL_REG_LOCK)) {
        DEBUG("[flash-common] locking the flash module\n");
        CNTRL_REG |= CNTRL_REG_LOCK;
    }
}

/* On the STM32H5 the flash banks cannot be read while one of them is being
 * erased, so the wait-for-busy loop must not be fetched from flash or the core
 * would stall forever when a slot erases itself (see riotboot rollback). Place
 * it in RAM. Other families keep it in flash (no cost). */
#if defined(CPU_FAM_STM32H5)
#define FLASH_RAMFUNC __attribute__((section(".ramfunc"), noinline))
#else
#define FLASH_RAMFUNC
#endif

FLASH_RAMFUNC
void _wait_for_pending_operations(void)
{
    if (FLASH_SR_REG & FLASH_SR_BSY) {
        DEBUG("[flash-common] waiting for any pending operation to finish\n");
        while (FLASH_SR_REG & FLASH_SR_BSY) {}
    }

#if defined(CPU_FAM_STM32G4)
    /* Check FLASH operation error flags */
    uint32_t error = (FLASH_SR_REG & FLASH_SR_ERRORS);
    /* Clear error programming flags */
    if (error != 0) {
        DEBUG("[flash-common] flash error flags set: 0x%" PRIx32 "\n", error);
        FLASH_SR_REG = error;
    }
#endif

    /* Clear 'end of operation' bit in status register, for other STM32 boards
       this bit is set only if EOPIE is set, which is currently not done */
#if defined(CPU_FAM_STM32F0) || defined(CPU_FAM_STM32F1) || \
    defined(CPU_FAM_STM32F3) || defined(CPU_FAM_STM32L0) || \
    defined(CPU_FAM_STM32L1)
    FLASH_SR_REG |= FLASH_SR_EOP;
#endif
}
