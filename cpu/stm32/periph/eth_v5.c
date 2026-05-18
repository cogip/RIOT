/*
 * SPDX-FileCopyrightText: 2026 COGIP Robotics association
 * SPDX-License-Identifier: LGPL-2.1-only
 */

/**
 * @ingroup     cpu_stm32
 * @{
 *
 * @file
 * @brief       Low-level ETH driver implementation for the STM32H7
 *              Synopsys DesignWare MAC v5 IP block.
 *
 * Structure mirrors cpu/stm32/periph/eth.c. The v5 descriptor
 * format and register set come from the H7 reference manual; ST's
 * HAL was used as a sanity check on bit positions only.
 *
 * The DMA descriptors and RX buffers live in SRAM3 via the .eth_ram
 * linker section. SRAM3 is currently used as plain SRAM because RIOT
 * does not enable the M7 D-cache on stm32h7, so DMA coherency does
 * not require an MPU non-cacheable region. When the D-cache is later
 * enabled, configure an MPU region over SRAM3 with TEX=1/C=0/B=1/S=1
 * to keep this driver coherent without per-frame cache management.
 *
 * @author      Gilles DOFFE <g.doffe@gmail.com>
 *
 * @}
 */

#include <assert.h>
#include <errno.h>
#include <string.h>

#include "board.h"
#include "iolist.h"
#include "macros/utils.h"
#include "mii.h"
#include "net/ethernet.h"
#include "net/eui_provider.h"
#include "net/netdev/eth.h"
#include "periph/gpio.h"
#include "periph_conf.h"

#define ENABLE_DEBUG    0
#include "debug.h"

#ifndef ETH_RX_DESCRIPTOR_COUNT
#define ETH_RX_DESCRIPTOR_COUNT     (8U)
#endif
#ifndef ETH_TX_DESCRIPTOR_COUNT
#define ETH_TX_DESCRIPTOR_COUNT     (4U)
#endif
#ifndef ETH_RX_BUFFER_SIZE
#define ETH_RX_BUFFER_SIZE          (1536U)
#endif

/* v5 normal descriptor: four 32-bit words. Field positions are
 * spelled out as the H7 CMSIS header does not define them. */
typedef struct {
    volatile uint32_t des0;
    volatile uint32_t des1;
    volatile uint32_t des2;
    volatile uint32_t des3;
} eth_v5_dma_desc_t;

/* RX descriptor (read format) DES3 */
#define RX_DESC_RD_BUF1V    (1U << 24)
#define RX_DESC_RD_IOC      (1U << 30)
#define RX_DESC_RD_OWN      (1U << 31)

/* RX descriptor (write-back format) DES3 */
#define RX_DESC_WB_PL       (0x7FFFU)       /* bits 14:0 packet length */
#define RX_DESC_WB_ES       (1U << 15)
#define RX_DESC_WB_OWN      (1U << 31)

/* TX descriptor (read format) DES2 */
#define TX_DESC_RD_B1L      (0x3FFFU)       /* bits 13:0 buffer length */
#define TX_DESC_RD_IOC      (1U << 31)
/* TX descriptor (read format) DES3 */
#define TX_DESC_RD_FL       (0x7FFFU)       /* bits 14:0 frame length */
#define TX_DESC_RD_FD       (1U << 29)
#define TX_DESC_RD_LD       (1U << 28)
#define TX_DESC_RD_OWN      (1U << 31)

/* TX descriptor (write-back format) DES3 */
#define TX_DESC_WB_OWN      (1U << 31)
#define TX_DESC_WB_ES       (1U << 15)

/* Descriptors and RX buffers must be DMA-coherent. The .eth_ram
 * section is mapped to SRAM3 by cpu/stm32/Makefile.include, and the
 * MPU below marks the region non-cacheable. */
__attribute__((section(".eth_ram"), aligned(32)))
static eth_v5_dma_desc_t rx_desc[ETH_RX_DESCRIPTOR_COUNT];

__attribute__((section(".eth_ram"), aligned(32)))
static eth_v5_dma_desc_t tx_desc[ETH_TX_DESCRIPTOR_COUNT];

__attribute__((section(".eth_ram"), aligned(32)))
static uint8_t rx_buffer[ETH_RX_DESCRIPTOR_COUNT][ETH_RX_BUFFER_SIZE];

/* TX coalescing buffer: upper layers hand the MAC multi-segment
 * iolists (one entry per protocol header plus the payload). The MAC
 * v5 read descriptor only references a single buffer in this first
 * pass, so each iolist is flattened into one TX_BUFFER_SIZE slot
 * before being handed to the DMA. */
#define ETH_TX_BUFFER_SIZE      (1536U)

__attribute__((section(".eth_ram"), aligned(32)))
static uint8_t tx_buffer[ETH_TX_DESCRIPTOR_COUNT][ETH_TX_BUFFER_SIZE];

static unsigned rx_idx;
static unsigned tx_idx;

netdev_t *stm32_eth_netdev;

/* MDIO clock divisor selection from HCLK frequency. The CR field
 * values come from the CMSIS header (encoded as constants, not
 * straight numerics). */
static uint32_t _mdio_clock_range(void)
{
#if CLOCK_AHB <= 35000000U
    return ETH_MACMDIOAR_CR_DIV16;
#elif CLOCK_AHB <= 60000000U
    return ETH_MACMDIOAR_CR_DIV26;
#elif CLOCK_AHB <= 100000000U
    return ETH_MACMDIOAR_CR_DIV42;
#elif CLOCK_AHB <= 150000000U
    return ETH_MACMDIOAR_CR_DIV62;
#elif CLOCK_AHB <= 250000000U
    return ETH_MACMDIOAR_CR_DIV102;
#else
    return ETH_MACMDIOAR_CR_DIV124;
#endif
}

static uint16_t _mii_reg_transfer(unsigned reg, uint16_t value, bool write)
{
    while (ETH->MACMDIOAR & ETH_MACMDIOAR_MB) {}

    if (write) {
        ETH->MACMDIODR = value;
    }

    uint32_t tmp = ETH->MACMDIOAR
                 & ~(ETH_MACMDIOAR_PA | ETH_MACMDIOAR_RDA | ETH_MACMDIOAR_MOC);
    tmp |= ((uint32_t)eth_config.phy_addr << ETH_MACMDIOAR_PA_Pos)
         & ETH_MACMDIOAR_PA;
    tmp |= ((uint32_t)reg << ETH_MACMDIOAR_RDA_Pos) & ETH_MACMDIOAR_RDA;
    tmp |= write ? ETH_MACMDIOAR_MOC_WR : ETH_MACMDIOAR_MOC_RD;
    tmp |= ETH_MACMDIOAR_MB;
    ETH->MACMDIOAR = tmp;

    while (ETH->MACMDIOAR & ETH_MACMDIOAR_MB) {}
    return (uint16_t)ETH->MACMDIODR;
}

static inline int16_t _mii_reg_read(uint8_t reg)
{
    return _mii_reg_transfer(reg, 0, false);
}

static inline void _mii_reg_write(uint8_t reg, uint16_t value)
{
    (void)_mii_reg_transfer(reg, value, true);
}

static inline bool _get_link_status(void)
{
    return (_mii_reg_read(MII_BMSR) & MII_BMSR_LINK);
}

static void _rx_desc_rearm(unsigned i)
{
    rx_desc[i].des0 = (uint32_t)(uintptr_t)&rx_buffer[i][0];
    rx_desc[i].des1 = 0;
    rx_desc[i].des2 = 0;
    __DMB();
    rx_desc[i].des3 = RX_DESC_RD_OWN | RX_DESC_RD_BUF1V | RX_DESC_RD_IOC;
    __DSB();
}

static void _init_dma_descriptors(void)
{
    memset(tx_desc, 0, sizeof(tx_desc));
    memset(rx_desc, 0, sizeof(rx_desc));

    for (unsigned i = 0; i < ETH_RX_DESCRIPTOR_COUNT; i++) {
        _rx_desc_rearm(i);
    }

    ETH->DMACTDRLR = ETH_TX_DESCRIPTOR_COUNT - 1U;
    ETH->DMACTDLAR = (uint32_t)(uintptr_t)tx_desc;
    ETH->DMACTDTPR = (uint32_t)(uintptr_t)tx_desc;
    tx_idx = 0;

    ETH->DMACRDRLR = ETH_RX_DESCRIPTOR_COUNT - 1U;
    ETH->DMACRDLAR = (uint32_t)(uintptr_t)rx_desc;
    ETH->DMACRDTPR =
        (uint32_t)(uintptr_t)&rx_desc[ETH_RX_DESCRIPTOR_COUNT - 1U];
    rx_idx = 0;
}

static void stm32_eth_get_addr(char *out)
{
    uint32_t t = ETH->MACA0HR;
    out[5] = (uint8_t)(t >> 8);
    out[4] = (uint8_t)t;
    t = ETH->MACA0LR;
    out[3] = (uint8_t)(t >> 24);
    out[2] = (uint8_t)(t >> 16);
    out[1] = (uint8_t)(t >> 8);
    out[0] = (uint8_t)t;
}

static void stm32_eth_set_addr(const uint8_t *addr)
{
    ETH->MACA0HR = ((uint32_t)addr[5] << 8) | (uint32_t)addr[4];
    ETH->MACA0LR = ((uint32_t)addr[3] << 24)
                 | ((uint32_t)addr[2] << 16)
                 | ((uint32_t)addr[1] << 8)
                 | (uint32_t)addr[0];
}

static void _setup_phy(void)
{
    DEBUG("[stm32_eth_v5] Reset PHY\n");
    _mii_reg_write(MII_BMCR, MII_BMCR_RESET);
    while (MII_BMCR_RESET & _mii_reg_read(MII_BMCR)) {}

    _mii_reg_write(MII_BMCR, MII_BMCR_AN_ENABLE | MII_BMCR_AN_RESTART);
}

static int stm32_eth_init(netdev_t *netdev)
{
    /* Pins (RMII alternate function 11 on H7, same as F-series). */
    for (int i = 0; i < (int)eth_config.mode; i++) {
        gpio_init(eth_config.pins[i], GPIO_IN);
        gpio_init_af(eth_config.pins[i], GPIO_AF11);
    }

    /* SYSCFG access for the MII/RMII selector. */
    RCC->APB4ENR |= RCC_APB4ENR_SYSCFGEN;
    SYSCFG->PMCR &= ~SYSCFG_PMCR_EPIS_SEL;
    if (eth_config.mode == RMII) {
        SYSCFG->PMCR |= SYSCFG_PMCR_EPIS_SEL_2;     /* 0b100 = RMII */
    }
    (void)SYSCFG->PMCR;

    /* MAC, MAC-TX, MAC-RX clocks plus a reset pulse. */
    RCC->AHB1ENR |= RCC_AHB1ENR_ETH1MACEN
                  | RCC_AHB1ENR_ETH1TXEN
                  | RCC_AHB1ENR_ETH1RXEN;
    RCC->AHB1RSTR |= RCC_AHB1RSTR_ETH1MACRST;
    RCC->AHB1RSTR &= ~RCC_AHB1RSTR_ETH1MACRST;

    /* DMA software reset. */
    ETH->DMAMR |= ETH_DMAMR_SWR;
    while (ETH->DMAMR & ETH_DMAMR_SWR) {}

    /* MDIO clock divisor: program before any PHY access. */
    uint32_t mdio = ETH->MACMDIOAR & ~ETH_MACMDIOAR_CR;
    ETH->MACMDIOAR = mdio | _mdio_clock_range();

    /* MAC config: 100 Mbps full duplex, RX checksum offload, automatic
     * pad/CRC strip. TE/RE stay cleared until the DMA is set up. */
    ETH->MACCR = ETH_MACCR_IPC | ETH_MACCR_ACS
               | ETH_MACCR_FES | ETH_MACCR_DM;
    /* Pass all multicast. */
    ETH->MACPFR = ETH_MACPFR_PM;
    /* No MAC flow control. */
    ETH->MACTFCR = 0;
    ETH->MACRFCR = 0;
    /* MTL: store-and-forward both directions. Also enable the
     * TX queue: TXQEN field (bits 3:2 of MTLTQOMR) reset value is 0b00
     * which means the TX queue is disabled, so the MAC accepts a
     * frame in the FIFO but never emits it on the wire. Set the
     * field to 0b10 = "enabled when MAC TX is enabled". The H7
     * CMSIS does not define the constant; encode the bit pattern
     * directly. */
    ETH->MTLTQOMR = ETH_MTLTQOMR_TSF | (0x2UL << 2);
    ETH->MTLRQOMR = ETH_MTLRQOMR_RSF;

    /* DMA bus mode: address-aligned, fixed burst. */
    ETH->DMASBMR = ETH_DMASBMR_AAL | ETH_DMASBMR_FB;
    ETH->DMAMR   = 0;

    /* DMA channel control: descriptors stored contiguously. */
    ETH->DMACCR  = 0;
    /* TX channel: PBL 32 beats. */
    ETH->DMACTCR = ETH_DMACTCR_TPBL_32PBL;
    /* RX channel: PBL 32 beats, RBSZ = RX buffer size (bits 14:1). */
    ETH->DMACRCR = (32U << 16) | ((ETH_RX_BUFFER_SIZE << 1)
                                  & ETH_DMACRCR_RBSZ);

    /* Disable MMC statistics interrupts (otherwise the ETH NVIC line
     * fires on every counter wrap-around). */
    ETH->MMCRIMR = ETH_MMCRIMR_RXLPITRCIM | ETH_MMCRIMR_RXLPIUSCIM
                 | ETH_MMCRIMR_RXUCGPIM   | ETH_MMCRIMR_RXALGNERPIM
                 | ETH_MMCRIMR_RXCRCERPIM;
    ETH->MMCTIMR = ETH_MMCTIMR_TXLPITRCIM | ETH_MMCTIMR_TXLPIUSCIM
                 | ETH_MMCTIMR_TXGPKTIM   | ETH_MMCTIMR_TXMCOLGPIM
                 | ETH_MMCTIMR_TXSCOLGPIM;

    _init_dma_descriptors();

    eui48_t hwaddr;
    netdev_eui48_get(netdev, &hwaddr);
    stm32_eth_set_addr(hwaddr.uint8);

    /* Start DMA, MAC, then enable interrupts. */
    ETH->DMACTCR |= ETH_DMACTCR_ST;
    ETH->DMACRCR |= ETH_DMACRCR_SR;
    ETH->DMACSR   = ETH_DMACSR_TPS | ETH_DMACSR_RPS;  /* clear */
    ETH->MTLTQOMR |= ETH_MTLTQOMR_FTQ;
    ETH->MACCR   |= ETH_MACCR_TE | ETH_MACCR_RE;
    ETH->DMACIER  = ETH_DMACIER_NIE | ETH_DMACIER_RIE | ETH_DMACIER_TIE
                  | ETH_DMACIER_AIE | ETH_DMACIER_RBUE | ETH_DMACIER_FBEE;

    NVIC_EnableIRQ(ETH_IRQn);

    _setup_phy();

    /* The H7 v5 driver does not implement stm32_eth_link_up yet; just
     * signal LINK_UP unconditionally so the upper layer probes the
     * link via the periodic netdev get / set instead. */
    netdev->event_callback(netdev, NETDEV_EVENT_LINK_UP);
    return 0;
}

static int stm32_eth_send(netdev_t *netdev, const struct iolist *iolist)
{
    (void)netdev;
    netdev->event_callback(netdev, NETDEV_EVENT_TX_STARTED);

    size_t total = iolist_size(iolist);
    if (total > ETH_TX_BUFFER_SIZE) {
        return -ENOBUFS;
    }

    eth_v5_dma_desc_t *d = &tx_desc[tx_idx];
    if (d->des3 & TX_DESC_RD_OWN) {
        return -EBUSY;
    }

    /* Coalesce the iolist into a contiguous TX slot. The MAC v5 read
     * descriptor can chain multiple buffers across descriptors with
     * FD / LD bits, but flattening the iolist into one buffer keeps
     * the descriptor handling simple for this first cut and is the
     * pattern other RIOT MAC drivers use. */
    size_t copied = iolist_to_buffer(iolist, &tx_buffer[tx_idx][0],
                                     ETH_TX_BUFFER_SIZE);
    if (copied != total) {
        return -ENOBUFS;
    }

    d->des0 = (uint32_t)(uintptr_t)&tx_buffer[tx_idx][0];
    d->des1 = 0;
    d->des2 = ((uint32_t)total & TX_DESC_RD_B1L) | TX_DESC_RD_IOC;
    __DMB();
    d->des3 = ((uint32_t)total & TX_DESC_RD_FL)
            | TX_DESC_RD_FD | TX_DESC_RD_LD | TX_DESC_RD_OWN;
    __DSB();

    tx_idx = (tx_idx + 1U) % ETH_TX_DESCRIPTOR_COUNT;
    ETH->DMACTDTPR = (uint32_t)(uintptr_t)&tx_desc[tx_idx];
    return 0;
}

static int stm32_eth_confirm_send(netdev_t *netdev, void *info)
{
    (void)netdev;
    (void)info;
    unsigned prev = (tx_idx + ETH_TX_DESCRIPTOR_COUNT - 1U)
                  % ETH_TX_DESCRIPTOR_COUNT;
    eth_v5_dma_desc_t *d = &tx_desc[prev];

    if (d->des3 & TX_DESC_WB_OWN) {
        return -EAGAIN;
    }
    if (d->des3 & TX_DESC_WB_ES) {
        return -EIO;
    }
    return (int)(d->des3 & RX_DESC_WB_PL);  /* same bit field as RX_PL */
}

static int stm32_eth_recv(netdev_t *netdev, void *buf, size_t max_len,
                          void *info)
{
    (void)netdev;
    (void)info;
    __DMB();
    eth_v5_dma_desc_t *d = &rx_desc[rx_idx];

    uint32_t des3 = d->des3;
    if (des3 & RX_DESC_WB_OWN) {
        return -EAGAIN;
    }

    /* DES3 in v5 write-back format can carry a context descriptor
     * (bit 30) instead of a data descriptor. Treat any context entry
     * or any out-of-range length the same way: drop the slot and
     * return EIO so the caller can keep draining. */
    bool ctxt = des3 & (1U << 30);
    uint32_t size = des3 & RX_DESC_WB_PL;

    if (ctxt || (des3 & RX_DESC_WB_ES) || size > ETH_RX_BUFFER_SIZE) {
        DEBUG("[stm32_eth_v5] RX drop des3=%lx size=%lu\n",
              (unsigned long)des3, (unsigned long)size);
        _rx_desc_rearm(rx_idx);
        ETH->DMACRDTPR = (uint32_t)(uintptr_t)&rx_desc[rx_idx];
        rx_idx = (rx_idx + 1U) % ETH_RX_DESCRIPTOR_COUNT;
        return -EIO;
    }

    if (!buf) {
        if (max_len) {
            _rx_desc_rearm(rx_idx);
            ETH->DMACRDTPR = (uint32_t)(uintptr_t)&rx_desc[rx_idx];
            rx_idx = (rx_idx + 1U) % ETH_RX_DESCRIPTOR_COUNT;
        }
        return (int)size;
    }

    if (max_len < size) {
        _rx_desc_rearm(rx_idx);
        ETH->DMACRDTPR = (uint32_t)(uintptr_t)&rx_desc[rx_idx];
        rx_idx = (rx_idx + 1U) % ETH_RX_DESCRIPTOR_COUNT;
        return -ENOBUFS;
    }

    memcpy(buf, &rx_buffer[rx_idx][0], size);
    _rx_desc_rearm(rx_idx);
    ETH->DMACRDTPR = (uint32_t)(uintptr_t)&rx_desc[rx_idx];
    rx_idx = (rx_idx + 1U) % ETH_RX_DESCRIPTOR_COUNT;
    return (int)size;
}

static int stm32_eth_get(netdev_t *netdev, netopt_t opt,
                         void *value, size_t max_len)
{
    switch (opt) {
    case NETOPT_ADDRESS:
        assert(max_len >= ETHERNET_ADDR_LEN);
        stm32_eth_get_addr(value);
        return ETHERNET_ADDR_LEN;
    case NETOPT_LINK: {
        netopt_enable_t tmp = _get_link_status();
        memcpy(value, &tmp, sizeof(tmp));
        return sizeof(netopt_enable_t);
    }
    default:
        return netdev_eth_get(netdev, opt, value, max_len);
    }
}

static int stm32_eth_set(netdev_t *netdev, netopt_t opt,
                         const void *value, size_t max_len)
{
    switch (opt) {
    case NETOPT_ADDRESS:
        assert(max_len >= ETHERNET_ADDR_LEN);
        stm32_eth_set_addr(value);
        return ETHERNET_ADDR_LEN;
    default:
        return netdev_eth_set(netdev, opt, value, max_len);
    }
}

static void stm32_eth_isr(netdev_t *netdev)
{
    netdev->event_callback(netdev, NETDEV_EVENT_RX_COMPLETE);
}

void isr_eth(void)
{
    uint32_t dmacsr = ETH->DMACSR;

    if (dmacsr & ETH_DMACSR_RI) {
        ETH->DMACSR = ETH_DMACSR_RI | ETH_DMACSR_NIS;
        if (stm32_eth_netdev) {
            netdev_trigger_event_isr(stm32_eth_netdev);
        }
    }
    if (dmacsr & ETH_DMACSR_TI) {
        ETH->DMACSR = ETH_DMACSR_TI | ETH_DMACSR_NIS;
        if (stm32_eth_netdev) {
            stm32_eth_netdev->event_callback(stm32_eth_netdev,
                                             NETDEV_EVENT_TX_COMPLETE);
        }
    }
    /* Always W1C RBU even when not in AIS: a burst that fills every
     * RX descriptor before the application drains them latches RBU
     * separately and would otherwise stick. */
    if (dmacsr & ETH_DMACSR_RBU) {
        ETH->DMACSR = ETH_DMACSR_RBU;
    }
    if (dmacsr & ETH_DMACSR_AIS) {
        ETH->DMACSR = ETH_DMACSR_AIS | ETH_DMACSR_FBE | ETH_DMACSR_RWT;
    }

    cortexm_isr_end();
}

static const netdev_driver_t netdev_driver_stm32h7eth = {
    .send         = stm32_eth_send,
    .confirm_send = stm32_eth_confirm_send,
    .recv         = stm32_eth_recv,
    .init         = stm32_eth_init,
    .isr          = stm32_eth_isr,
    .get          = stm32_eth_get,
    .set          = stm32_eth_set,
};

void stm32_eth_netdev_setup(netdev_t *netdev)
{
    stm32_eth_netdev = netdev;
    netdev->driver = &netdev_driver_stm32h7eth;
    netdev_register(netdev, NETDEV_STM32_ETH, 0);
}
