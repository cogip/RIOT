/*
 * Copyright (C) 2026 Mathis Lécrivain <lecrivain.mathis@gmail.com>
 *                    COGIP Robotics association
 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 */

/**
 * @ingroup     sys_riotboot_can
 * @{
 *
 * @file
 * @brief       CAN Bootloader
 *
 * @author      Mathis Lécrivain <lecrivain.mathis@gmail.com>
 *
 * @}
 */

#include <string.h>

#include "checksum/crc8.h"

#include "can/conn/raw.h"
#include "can/device.h"
#include "can/dll.h"
#include "can_params.h"

#include "periph_conf.h"
#include "periph/can.h"
#include "periph/gpio.h"

#include "riotboot/bootloader_selection.h"
#include "riotboot/can.h"
#include "riotboot/flashwrite.h"
#include "riotboot/magic.h"
#include "riotboot/slot.h"

#include "ztimer.h"

/*
 * The bootloader only initializes the required peripherals 
 * and minimal system components to keep a small binary.
 * All auto-init services are disabled, so the bootloader is
 * responsible for setting up the CAN interface manually.
 */
#define CANDEV_NUMOF (ARRAY_SIZE(candev_params))

#ifndef CANDEV_STACKSIZE
#  define CANDEV_STACKSIZE (THREAD_STACKSIZE_DEFAULT)
#endif

#ifndef CANDEV_BASE_PRIORITY
#  define CANDEV_BASE_PRIORITY (THREAD_PRIORITY_MAIN - CANDEV_NUMOF - 2)
#endif

/* CAN device descriptors, thread stacks and device handles */
static candev_dev_t _candev_dev[CANDEV_NUMOF];
static char _can_stacks[CANDEV_NUMOF][CANDEV_STACKSIZE];
static can_t _candev[CANDEV_NUMOF];

/* CAN connection and filter handles */
static conn_can_raw_t _conn;
static struct can_filter _filter;

/* Flash writer handle */
static riotboot_flashwrite_t _writer;

/* Flag that indicates firmware streaming in progress */
static bool _flash_write_pending;

static void _init_can(void)
{
    can_dll_init();

    for (size_t i = 0; i < CANDEV_NUMOF; i++) {
        can_init(&_candev[i], &candev_conf[i]);
        _candev_dev[i].dev = (candev_t *)&_candev[i];
        _candev_dev[i].name = candev_params[i].name;
#ifdef MODULE_CAN_TRX
        _candev_dev[i].trx = candev_params[i].trx;
#endif
#ifdef MODULE_CAN_PM
        _candev_dev[i].rx_inactivity_timeout = candev_params[i].rx_inactivity_timeout;
        _candev_dev[i].tx_wakeup_timeout = candev_params[i].tx_wakeup_timeout;
#endif
#ifdef MODULE_FDCAN
        _candev_dev[i].loop_delay = candev_params[i].loop_delay;
#endif

        can_device_init(_can_stacks[i], CANDEV_STACKSIZE, CANDEV_BASE_PRIORITY + i,
                        candev_params[i].name, &_candev_dev[i]);
    }
}

static void _send(const void *data, uint8_t len)
{
    can_frame_t frame;

    memset(&frame, 0, sizeof(frame));
    frame.can_id = RIOTBOOT_CAN_ID_RESP;
    frame.len = len;
#ifdef MODULE_FDCAN
    frame.flags = CANFD_FDF;
#endif
    memcpy(frame.data, data, len);

    conn_can_raw_send(&_conn, &frame, 0);
}

static void _send_byte(uint8_t b)
{
    _send(&b, sizeof(b));
}

static int _recv(can_frame_t *frame, uint32_t timeout_us)
{
    return conn_can_raw_recv(&_conn, frame, timeout_us);
}

/**
  * @brief Wait for bootloader entry command or timeout.
  * 
  * @note Sends WAITING periodically, listens for ENTER_LOADER or boot button.
  * 
  * @return true if should boot (timeout)
  * @return false if entering bootloader
  */
static bool _bootdelay(void)
{
    uint32_t *magic = (void *)(uintptr_t)RIOTBOOT_MAGIC_ADDR;

    if (*magic == RIOTBOOT_MAGIC) {
        *magic = 0;
        return false;
    }

    _send_byte(RIOTBOOT_CAN_STAT_WAITING);

    uint32_t elapsed_ms = 0;
    const uint32_t poll_ms = RIOTBOOT_CAN_POLL_TIMEOUT_US / 1000;

    while (elapsed_ms < RIOTBOOT_CAN_DELAY_MS) {
        can_frame_t rx;

        if (_recv(&rx, RIOTBOOT_CAN_POLL_TIMEOUT_US) == sizeof(can_frame_t) && rx.len >= 1) {
            if (rx.data[0] == RIOTBOOT_CAN_PROBE) {
                _send_byte(RIOTBOOT_CAN_STAT_WAITING);
            }
            else if (rx.data[0] == RIOTBOOT_CAN_ENTER_LOADER) {
                _send_byte(RIOTBOOT_CAN_STAT_READY);
                return false;
            }
        }

#ifdef LED_BOOTLOADER_PIN
        LED_BOOTLOADER_TOGGLE;
#endif

        elapsed_ms += poll_ms;
    }

    return true;
}

/**
 * @brief  Find the best slot for a firmware update.
 *
 * @note   Selection priority:
 *         1. First slot with an invalid firmware (corrupt or absent).
 *         2. First slot whose header start address does not match the
 *            expected image start address (inconsistent header).
 *         3. Among valid slots, the one with the lowest firmware version
 *            (oldest firmware gets overwritten).
 *
 * @return slot index (>= 0), or -1 if none available
 */
static int _get_available_slot(void)
{
    int slot = -1;
    uint32_t version = UINT32_MAX;

    for (unsigned i = 0; i < riotboot_slot_numof; i++) {
        const riotboot_hdr_t *hdr = riotboot_slot_get_hdr(i);

        if (riotboot_slot_validate(i)) {
            return i;
        }

        if (hdr->start_addr != riotboot_slot_get_image_startaddr(i)) {
            return i;
        }

        if (slot == -1 || hdr->version < version) {
            version = hdr->version;
            slot = i;
        }
    }

    return slot;
}

static void _cmd_start(void)
{
    int slot = _get_available_slot();

    if (slot < 0) {
        _send_byte(RIOTBOOT_CAN_STAT_ILLEGAL);
        return;
    }

    if (riotboot_flashwrite_init(&_writer, slot) < 0) {
        _send_byte(RIOTBOOT_CAN_STAT_ERROR);
        return;
    }

    _flash_write_pending = true;

    size_t slot_size = riotboot_flashwrite_slotsize(&_writer);
    uint8_t resp[RIOTBOOT_CAN_START_RESP_SIZE];
    resp[0] = RIOTBOOT_CAN_STAT_OK;
    resp[1] = (uint8_t)(slot & 0xFF);
    memcpy(&resp[2], &slot_size, sizeof(size_t));
    _send(resp, sizeof(resp));
}

static void _cmd_data(uint8_t len, const uint8_t *data)
{
    if (!_flash_write_pending || len == 0) {
        _send_byte(RIOTBOOT_CAN_STAT_ILLEGAL);
        return;
    }

    if (riotboot_flashwrite_putbytes(&_writer, data, len, true) < 0) {
        _flash_write_pending = false;
        _send_byte(RIOTBOOT_CAN_STAT_ERROR);
        return;
    }

    _send_byte(RIOTBOOT_CAN_STAT_OK);
}

static void _cmd_finish(void)
{
    if (!_flash_write_pending) {
        _send_byte(RIOTBOOT_CAN_STAT_ILLEGAL);
        return;
    }

    _flash_write_pending = false;

    if (riotboot_flashwrite_flush(&_writer) < 0) {
        _send_byte(RIOTBOOT_CAN_STAT_ERROR);
        return;
    }

    if (riotboot_flashwrite_finish(&_writer) < 0) {
        _send_byte(RIOTBOOT_CAN_STAT_ERROR);
        return;
    }

    _send_byte(RIOTBOOT_CAN_STAT_OK);
}

static int _process_cmd(const can_frame_t *rx)
{
    if (rx->len < 1) {
        return -2;
    }

    uint8_t type = rx->data[0];

    switch (type) {
    /* re-create initial sync handshake if already in bootloader */
    case RIOTBOOT_CAN_PROBE:
        _send_byte(RIOTBOOT_CAN_STAT_WAITING);
        return -2;
    case RIOTBOOT_CAN_ENTER_LOADER:
        _send_byte(RIOTBOOT_CAN_STAT_READY);
        return -2;
    /* boot command needs no checksum */
    case RIOTBOOT_CAN_CMD_BOOT:
        if (rx->len < 2) {
            _send_byte(RIOTBOOT_CAN_STAT_ILLEGAL);
            return -2;
        }
        switch (rx->data[1]) {
        case '\n':
            _send_byte(RIOTBOOT_CAN_STAT_OK);
            return -1;
#ifdef MODULE_RIOTBOOT_SLOT
        case '0':
            _send_byte(RIOTBOOT_CAN_STAT_OK);
            return 0;
        case '1':
            _send_byte(RIOTBOOT_CAN_STAT_OK);
            return 1;
#endif
        default:
            _send_byte(RIOTBOOT_CAN_STAT_ILLEGAL);
            return -2;
        }
    case RIOTBOOT_CAN_CMD_START:
    case RIOTBOOT_CAN_CMD_DATA:
    case RIOTBOOT_CAN_CMD_FINISH:
        break;
    default:
        _send_byte(RIOTBOOT_CAN_STAT_ILLEGAL);
        return -2;
    }

    /* CRC-protected commands: [type(1) | len(1) | data(len) | crc8(1)] */
    uint8_t len = rx->data[1];

    if (rx->len < 3 || rx->len < 2u + len + 1u) {
        _send_byte(RIOTBOOT_CAN_STAT_ILLEGAL);
        return -2;
    }

    uint8_t crc = crc8(rx->data, 2 + len, RIOTBOOT_CAN_CRC8_POLY, 0xFF);
    if (crc != rx->data[2 + len]) {
        _send_byte(RIOTBOOT_CAN_STAT_BAD_CRC);
        return -2;
    }

    switch (type) {
    case RIOTBOOT_CAN_CMD_START:
        _cmd_start();
        break;
    case RIOTBOOT_CAN_CMD_DATA:
        _cmd_data(len, &rx->data[2]);
        break;
    case RIOTBOOT_CAN_CMD_FINISH:
        _cmd_finish();
        break;
    }

    return -2;
}

int riotboot_can_init(void)
{
#ifdef BTN_BOOTLOADER_PIN
    gpio_init(BTN_BOOTLOADER_PIN, BTN_BOOTLOADER_MODE);
#endif
#ifdef LED_BOOTLOADER_PIN
    gpio_init(LED_BOOTLOADER_PIN, GPIO_OUT);
    LED_BOOTLOADER_OFF;
#endif

    /* Initialize ztimer (needed for conn_can timeouts) */
    ztimer_init();

    /* Initialize CAN stack */
    _init_can();

    /* Create CAN connection with filter on command ID */
    _filter.can_id = RIOTBOOT_CAN_ID_CMD;
    _filter.can_mask = CAN_SFF_MASK;
    if (conn_can_raw_create(&_conn, &_filter, 1, 0, 0) < 0) {
        return -1;
    }

    return 0;
}

int riotboot_can_loader(void)
{
    if (_bootdelay()) {
        conn_can_raw_close(&_conn);
        return -1;
    }

#ifdef LED_BOOTLOADER_ON
    LED_BOOTLOADER_ON;
#endif

    while (1) {
        can_frame_t rx;

        if (_recv(&rx, 0) != sizeof(can_frame_t)) {
            continue;
        }

        int slot = _process_cmd(&rx);
        if (slot != -2) {
            conn_can_raw_close(&_conn);
            return slot;
        }
    }

    return -1;
}
