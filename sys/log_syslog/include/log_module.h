/*
 * Copyright (C) 2026 Gilles DOFFE
 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 */

#pragma once

/**
 * @defgroup    sys_log_syslog log_syslog: network syslog log backend
 * @ingroup     sys
 * @brief       LOG backend that echoes to the console and forwards each line
 *              as an RFC 5424 message over UDP (lwIP).
 * @{
 *
 * @file
 * @brief       log_module override header
 *
 * @author      Gilles DOFFE <g.doffe@gmail.com>
 */

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief   log_write override: console echo + UDP syslog forwarding
 *
 * @param[in] level  Logging level
 * @param[in] format printf-style format string
 */
void log_write(unsigned level, const char *format, ...);

#ifdef __cplusplus
}
#endif
/**@}*/
