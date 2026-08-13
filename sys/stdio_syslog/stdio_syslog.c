/*
 * Copyright (C) 2026 Gilles DOFFE
 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 */

/**
 * @ingroup     sys_stdio_syslog
 * @{
 *
 * @file
 * @brief       syslog stdio backend: console tee to RFC 5424 over UDP
 *
 * The stdio write hook runs in the caller's context (any thread, possibly an
 * ISR), so it only assembles lines and hands complete ones to a ring buffer
 * under a short interrupt-masked section. A dedicated thread wraps each line
 * in an RFC 5424 header and does the lwIP UDP send, where taking the TCP/IP
 * core lock is safe.
 *
 * @author      Gilles DOFFE <g.doffe@gmail.com>
 */

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "irq.h"
#include "mutex.h"
#include "stdio_base.h"
#include "thread.h"
#include "ztimer.h"

#include "lwip/etharp.h"
#include "lwip/ip_addr.h"
#include "lwip/netif.h"
#include "lwip/pbuf.h"
#include "lwip/tcpip.h"
#include "lwip/udp.h"

#include "log.h"
#include "stdio_syslog.h"

/* --- configuration (override via CFLAGS) --- */
#ifndef CONFIG_SYSLOG_SERVER
#define CONFIG_SYSLOG_SERVER   "192.168.0.1"
#endif
#ifndef CONFIG_SYSLOG_PORT
#define CONFIG_SYSLOG_PORT     514
#endif
#ifndef CONFIG_SYSLOG_FACILITY
#define CONFIG_SYSLOG_FACILITY 1    /* user-level messages */
#endif
#ifndef CONFIG_SYSLOG_HOSTNAME
#define CONFIG_SYSLOG_HOSTNAME "-"
#endif
#ifndef CONFIG_SYSLOG_APPNAME
#define CONFIG_SYSLOG_APPNAME  "riot"
#endif
#ifndef CONFIG_SYSLOG_LINE_LEN
#define CONFIG_SYSLOG_LINE_LEN 192
#endif
#ifndef CONFIG_SYSLOG_LINE_NUM
#define CONFIG_SYSLOG_LINE_NUM 64    /* deep enough to hold the whole boot */
#endif
#ifndef CONFIG_SYSLOG_NETIF
#define CONFIG_SYSLOG_NETIF    "ET0" /* first lwIP Ethernet interface */
#endif
/* how long to wait for the collector to become reachable before giving up on
 * the buffered boot lines (a permanently unplugged robot must not buffer for
 * ever). Covers cold PHY autonegotiation + ARP resolution. */
#ifndef CONFIG_SYSLOG_READY_TIMEOUT_MS
#define CONFIG_SYSLOG_READY_TIMEOUT_MS 8000
#endif

/* RIOT LOG level default when no LOG severity is set for a line */
#define SYSLOG_DEFAULT_LEVEL   LOG_INFO

typedef struct {
    uint8_t level;                      /* RIOT LOG level of the line   */
    uint16_t len;                       /* number of chars in buf       */
    char buf[CONFIG_SYSLOG_LINE_LEN];   /* raw line, no newline         */
} _line_t;

/* completed-line ring: producer = stdio write hook, consumer = sender thread */
static _line_t _ring[CONFIG_SYSLOG_LINE_NUM];
static volatile unsigned _head;
static volatile unsigned _tail;

/* line currently being assembled by the write hook */
static char _cur[CONFIG_SYSLOG_LINE_LEN];
static unsigned _cur_len;
static volatile unsigned _level = SYSLOG_DEFAULT_LEVEL;

/* ANSI escape stripping state (colour codes look like raw bytes in journald).
 * A sequence may straddle two _write() calls, so keep the state static. */
enum { _ESC_NONE, _ESC_SEEN, _ESC_CSI };
static unsigned _esc;

static mutex_t _doorbell = MUTEX_INIT_LOCKED;
static char _stack[THREAD_STACKSIZE_DEFAULT];
static struct udp_pcb *_pcb;
static ip_addr_t _server;
static bool _ready;

/* RIOT LOG level (1..4) -> RFC 5424 severity code */
static unsigned _severity(unsigned level)
{
    switch (level) {
    case LOG_ERROR:
        return 3; /* error   */
    case LOG_WARNING:
        return 4; /* warning */
    case LOG_INFO:
        return 6; /* info    */
    case LOG_DEBUG:
        return 7; /* debug   */
    default:
        return 6;
    }
}

/* commit the assembled line into the ring; called with interrupts masked */
static void _commit(void)
{
    if (_cur_len == 0) {
        _level = SYSLOG_DEFAULT_LEVEL;
        return;
    }
    unsigned next = (_head + 1) % CONFIG_SYSLOG_LINE_NUM;
    if (next != _tail) { /* drop the line if the ring is full */
        _line_t *l = &_ring[_head];
        l->level = _level;
        l->len = _cur_len;
        memcpy(l->buf, _cur, _cur_len);
        _head = next;
    }
    _cur_len = 0;
    _level = SYSLOG_DEFAULT_LEVEL;
}

void stdio_syslog_set_level(unsigned level)
{
    _level = level;
}

static ssize_t _write(const void *src, size_t len)
{
    const char *p = src;
    bool produced = false;

    unsigned irq_state = irq_disable();
    for (size_t i = 0; i < len; i++) {
        char c = p[i];

        /* drop ANSI escape sequences (ESC, ESC[..final) */
        if (_esc == _ESC_CSI) {
            if ((unsigned char)c >= 0x40 && (unsigned char)c <= 0x7e) {
                _esc = _ESC_NONE; /* final byte ends the sequence */
            }
            continue;
        }
        if (_esc == _ESC_SEEN) {
            _esc = (c == '[') ? _ESC_CSI : _ESC_NONE;
            continue;
        }
        if (c == '\033') {
            _esc = _ESC_SEEN;
            continue;
        }

        if (c == '\r') {
            continue;
        }
        if (c == '\n') {
            _commit();
            produced = true;
            continue;
        }
        if (_cur_len >= CONFIG_SYSLOG_LINE_LEN) {
            _commit(); /* overlong line: flush what we have and keep going */
            produced = true;
        }
        _cur[_cur_len++] = c;
    }
    irq_restore(irq_state);

    if (produced && _ready) {
        mutex_unlock(&_doorbell);
    }
    return len;
}

static void _send(const _line_t *l)
{
    char dgram[CONFIG_SYSLOG_LINE_LEN + 64];
    unsigned pri = CONFIG_SYSLOG_FACILITY * 8 + _severity(l->level);

    /* RFC 5424: <PRI>VERSION TIMESTAMP HOSTNAME APP-NAME PROCID MSGID SD MSG.
     * No wall clock on the target, so TIMESTAMP is the NILVALUE "-". */
    int hlen = snprintf(dgram, sizeof(dgram), "<%u>1 - %s %s - - - ", pri,
                        CONFIG_SYSLOG_HOSTNAME, CONFIG_SYSLOG_APPNAME);
    if (hlen < 0) {
        return;
    }
    unsigned n = l->len;
    if ((unsigned)hlen + n > sizeof(dgram)) {
        n = sizeof(dgram) - hlen;
    }
    memcpy(dgram + hlen, l->buf, n);
    unsigned total = (unsigned)hlen + n;

    sys_lock_tcpip_core();
    struct pbuf *p = pbuf_alloc(PBUF_TRANSPORT, total, PBUF_RAM);
    if (p != NULL) {
        pbuf_take(p, dgram, total);
        udp_sendto(_pcb, p, &_server, CONFIG_SYSLOG_PORT);
        pbuf_free(p);
    }
    sys_unlock_tcpip_core();
}

/* Block until the collector's MAC is resolved, then flush.
 *
 * The buffered boot lines must not be flushed before the link genuinely
 * carries traffic, otherwise the leading frames are lost. The STM32 MAC v5
 * driver reports the link up unconditionally at init, so its link flag is not
 * a reliable "ready" signal; ARP resolution is: it only completes once the
 * PHY has finished autonegotiation and packets actually flow. Poll for the
 * server's ARP entry, prodding a request each round, until it resolves (or
 * the bounded timeout, so an unplugged robot still drains and moves on). */
static void _wait_ready(void)
{
    sys_lock_tcpip_core();
    struct netif *iface = netif_find(CONFIG_SYSLOG_NETIF);
    sys_unlock_tcpip_core();
    if (iface == NULL) {
        return;
    }
    const ip4_addr_t *ip4 = ip_2_ip4(&_server);
    uint32_t waited = 0;
    for (;;) {
        struct eth_addr *eth_ret;
        const ip4_addr_t *ip_ret;
        sys_lock_tcpip_core();
        bool resolved = etharp_find_addr(iface, ip4, &eth_ret, &ip_ret) >= 0;
        if (!resolved && netif_is_link_up(iface)) {
            etharp_request(iface, ip4);
        }
        sys_unlock_tcpip_core();
        if (resolved || waited >= CONFIG_SYSLOG_READY_TIMEOUT_MS) {
            break;
        }
        ztimer_sleep(ZTIMER_MSEC, 100);
        waited += 100;
    }
}

static void *_drain_thread(void *arg)
{
    (void)arg;
    _wait_ready();
    while (1) {
        mutex_lock(&_doorbell);
        while (_tail != _head) {
            _send(&_ring[_tail]);
            _tail = (_tail + 1) % CONFIG_SYSLOG_LINE_NUM;
        }
    }
    return NULL;
}

int stdio_syslog_init(void)
{
    sys_lock_tcpip_core();
    _pcb = udp_new_ip_type(IPADDR_TYPE_V4);
    sys_unlock_tcpip_core();
    if (_pcb == NULL) {
        return -1;
    }
    if (!ipaddr_aton(CONFIG_SYSLOG_SERVER, &_server)) {
        return -1;
    }

    kernel_pid_t pid = thread_create(_stack, sizeof(_stack), THREAD_PRIORITY_MAIN - 1, 0,
                                     _drain_thread, NULL, "syslog");
    if (pid <= KERNEL_PID_UNDEF) {
        return -1;
    }

    _ready = true;
    mutex_unlock(&_doorbell); /* flush lines buffered during boot */
    LOG_INFO("stdio_syslog: forwarding to %s:%u\n", CONFIG_SYSLOG_SERVER,
             (unsigned)CONFIG_SYSLOG_PORT);
    return 0;
}

STDIO_PROVIDER(STDIO_SYSLOG, NULL, NULL, _write)
/**@}*/
