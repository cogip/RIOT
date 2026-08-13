/*
 * Copyright (C) 2026 Gilles DOFFE
 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 */

#pragma once

/**
 * @defgroup    sys_stdio_syslog stdio_syslog: syslog stdio backend
 * @ingroup     sys
 * @brief       Additional stdio backend that forwards every console line as an
 *              RFC 5424 syslog message over UDP (lwIP).
 *
 * Registered alongside the regular console backend (e.g. stdio_uart) via the
 * stdio dispatch layer, so all output (printf/puts and LOG_*) is both printed
 * locally and shipped to a syslog collector. Lines are buffered and sent from
 * a dedicated thread, so writing from any context (including ISRs) is safe.
 *
 * With the @ref sys_log_syslog module also enabled, LOG_* lines carry their
 * real severity; everything else defaults to the "info" severity.
 * @{
 *
 * @file
 * @brief       Public API of the syslog stdio backend
 *
 * @author      Gilles DOFFE <g.doffe@gmail.com>
 */

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief   Start the syslog forwarder.
 *
 * Creates the UDP socket and the sender thread, then flushes any lines that
 * were buffered during boot. Must be called once the network stack is up and
 * the interface has an address. Until it runs, output is still printed on the
 * console and buffered (bounded) for later forwarding.
 *
 * @return  0 on success, <0 on error
 */
int stdio_syslog_init(void);

/**
 * @brief   Set the syslog severity for the next console line.
 *
 * Used by the log_syslog LOG backend to tag LOG_* lines. Automatically reset
 * to the default (info) after each completed line.
 *
 * @param[in] level  RIOT LOG level (LOG_ERROR..LOG_DEBUG)
 */
void stdio_syslog_set_level(unsigned level);

#ifdef __cplusplus
}
#endif
/**@}*/
