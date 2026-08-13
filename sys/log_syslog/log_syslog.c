/*
 * Copyright (C) 2026 Gilles DOFFE
 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 */

/**
 * @ingroup     sys_log_syslog
 * @{
 *
 * @file
 * @brief       LOG backend that tags console lines with a syslog severity
 *
 * This backend only prints to the console (like the default LOG backend) but
 * first records the message severity so the @ref sys_stdio_syslog stdio tee
 * can label the corresponding UDP line. The actual network forwarding is done
 * by stdio_syslog, which also captures plain printf/puts output.
 *
 * @author      Gilles DOFFE <g.doffe@gmail.com>
 */

#include <stdarg.h>
#include <stdio.h>

#include "stdio_syslog.h"

void log_write(unsigned level, const char *format, ...)
{
    /* tag the line the syslog stdio backend is about to assemble */
    stdio_syslog_set_level(level);

    va_list args;
    va_start(args, format);
#ifdef __clang__
#pragma clang diagnostic push
#pragma clang diagnostic ignored "-Wformat-nonliteral"
#endif
    vprintf(format, args);
#ifdef __clang__
#pragma clang diagnostic pop
#endif
    va_end(args);

#if !defined(__MSP430__)
    fflush(stdout);
#endif
}
/**@}*/
