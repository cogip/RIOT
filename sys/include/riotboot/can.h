/*
 * Copyright (C) 2026 Mathis Lécrivain <lecrivain.mathis@gmail.com>
 *                    COGIP Robotics association
 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 */

/**
 * @defgroup    sys_riotboot_can CAN Bootloader Protocol
 * @ingroup     sys
 * @{
 *
 * @file
 * @brief       CAN Bootloader
 *
 * Streaming protocol backed by riotboot_flashwrite (RAW mode).
 * The host streams firmware bytes via START / DATA / FINISH commands
 * and lets the device manage flash erasure and page boundaries internally.
 *
 * Frame format:
 *
 *     +--------+--------+-----------------+--------+
 *     | type   | len    | data            | crc8   |
 *     | 1 byte | 1 byte | (len bytes)     | 1 byte |
 *     +--------+--------+-----------------+--------+
 *
 * Usable payload per DATA frame:
 *   - Classic CAN (8 bytes MTU): up to 5 bytes
 *   - CAN-FD     (64 bytes MTU): up to 61 bytes
 *
 * @author      Mathis Lécrivain <lecrivain.mathis@gmail.com>
 *
 * @}
 */

#pragma once

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief riotboot CAN commands
 *        Commands typically have the format [type|length|value]
 *        where type and length are one byte and value is $length bytes.
 *
 *        Commands are signed with a CRC-8 checksum that is calculated
 *        over the entire record. The Checksum is not part of length.
 * @{
 */
/** start application ('b' + slot char, no CRC) */
#define RIOTBOOT_CAN_CMD_BOOT        'b'
/** start firmware update ([s, 0x00, crc8], slot auto-selected) */
#define RIOTBOOT_CAN_CMD_START       's'
/** firmware data chunk ([d, len, data(N), crc8]) */
#define RIOTBOOT_CAN_CMD_DATA        'd'
/** finish firmware update ([f, 0x00, crc8]) */
#define RIOTBOOT_CAN_CMD_FINISH      'f'
/** @} */

/**
 * @brief riotboot CAN response codes
 * @{
 */
#define RIOTBOOT_CAN_STAT_OK         '.' /**< operation successful */
#define RIOTBOOT_CAN_STAT_BAD_CRC    '?' /**< CRC error */
#define RIOTBOOT_CAN_STAT_ILLEGAL    '!' /**< illegal parameter */
#define RIOTBOOT_CAN_STAT_ERROR      'E' /**< flash write/erase error */
/** @} */

/**
 * @brief riotboot CAN synchronisation
 * @{
 */
#define RIOTBOOT_CAN_ENTER_LOADER    'B' /**< sent to stop auto-boot */
#define RIOTBOOT_CAN_PROBE           '?' /**< probe bootloader */
#define RIOTBOOT_CAN_STAT_WAITING    'b' /**< sent during boot delay */
#define RIOTBOOT_CAN_STAT_READY      '>' /**< ready for commands */
                                         /** @} */

/**
 * @brief response size for START: status(1 byte) + slot(1 byte) + slot_size(4 bytes)
 */
#define RIOTBOOT_CAN_START_RESP_SIZE (sizeof(uint8_t) + sizeof(uint8_t) + sizeof(size_t))

/**
 * @name    CAN ID configuration
 *
 * Each node has a unique NODE_ID (0–127).  CAN IDs are derived as:
 *   CMD  = BASE + 2*NODE_ID      (host -> node)
 *   RESP = BASE + 2*NODE_ID + 1  (node -> host)
 *
 * Set RIOTBOOT_CAN_NODE_ID via CFLAGS in the application Makefile:
 *   CFLAGS += -DRIOTBOOT_CAN_NODE_ID=3
 *
 * @{
 */
#ifndef RIOTBOOT_CAN_BASE_ID
#  define RIOTBOOT_CAN_BASE_ID (0x100) /**< base CAN ID */
#endif

#ifndef RIOTBOOT_CAN_NODE_ID
#  define RIOTBOOT_CAN_NODE_ID (0) /**< node identifier (0–127) */
#endif

#define RIOTBOOT_CAN_ID_CMD  (RIOTBOOT_CAN_BASE_ID + 2 * RIOTBOOT_CAN_NODE_ID)
#define RIOTBOOT_CAN_ID_RESP (RIOTBOOT_CAN_BASE_ID + 2 * RIOTBOOT_CAN_NODE_ID + 1)
/** @} */

/**
 * @name Bootloader configuration
 * @{
 */
#ifndef RIOTBOOT_CAN_DELAY_MS
#  define RIOTBOOT_CAN_DELAY_MS (250) /**< auto-boot delay in ms */
#endif

#ifndef RIOTBOOT_CAN_CRC8_POLY
#  define RIOTBOOT_CAN_CRC8_POLY (0x31) /**< CRC-8 polynomial */
#endif

#ifndef RIOTBOOT_CAN_POLL_TIMEOUT_US
#  define RIOTBOOT_CAN_POLL_TIMEOUT_US (50000) /**< recv poll timeout in us */
#endif
/** @} */

/**
 * @brief Initialize CAN bootloader peripherals
 *
 * @return 0 on success, -1 on can initialization failure
 */
int riotboot_can_init(void);

/**
 * @brief Start CAN bootloader
 *
 * @return slot to boot, -1 if default slot should be started
 */
int riotboot_can_loader(void);

#ifdef __cplusplus
}
#endif
