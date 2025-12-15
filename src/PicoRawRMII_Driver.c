#include "PicoRawRMII_Driver.h"

#include <string.h>
#include <stdio.h>

#include "pico/stdlib.h"
#include "pico/unique_id.h"

#include "hardware/pio.h"
#include "hardware/dma.h"
#include "hardware/clocks.h"
#include "hardware/irq.h"
#include "hardware/vreg.h"

#include "rmii_ethernet_phy_rx.pio.h"
#include "rmii_ethernet_phy_tx.pio.h"

#ifdef ARDUINO
#define RMII_ETH_HAS_STDIO 0
#else
#define RMII_ETH_HAS_STDIO 1
#endif

// ===== Driver config/state =====
static rmii_ethernet_config_t g_cfg;

static inline uint rmii_get_pio_irq_index(void) {
    uint idx = pio_get_index(g_cfg.pio);
    switch (idx) {
    case 0: return PIO0_IRQ_0;
    case 1: return PIO1_IRQ_0;
#if defined(PIO2_IRQ_0)
    case 2: return PIO2_IRQ_0;
#endif
    default: return PIO0_IRQ_0;
    }
}

// ===== RX ring (DMA writes) =====
#define RX_BUF_SIZE_POW 12
#define RX_BUF_SIZE (1u << RX_BUF_SIZE_POW)
#define RX_BUF_MASK (RX_BUF_SIZE - 1u)
static volatile uint8_t rx_ring[RX_BUF_SIZE] __attribute__((aligned(RX_BUF_SIZE)));

#define RX_NUM_PTR_POW 5
#define RX_NUM_PTR (1u << RX_NUM_PTR_POW)
#define RX_NUM_MASK (RX_NUM_PTR - 1u)

typedef struct {
    uint16_t pkt_addr;
    uint16_t pkt_len;
} pkt_ptr_t;

static volatile pkt_ptr_t rx_pkt_ptr[RX_NUM_PTR];
static volatile uint32_t rx_curr_pkt_ptr = 0; // ISR writes, poll reads
static uint32_t rx_prev_pkt_ptr = 0;          // poll only
static uint32_t rx_addr = 0;                  // ISR only

// ===== TX ring (DMA reads) =====
#define TX_BUF_SIZE_POW 12
#define TX_BUF_SIZE (1u << TX_BUF_SIZE_POW)
#define TX_BUF_MASK (TX_BUF_SIZE - 1u)
static volatile uint8_t tx_ring[TX_BUF_SIZE] __attribute__((aligned(TX_BUF_SIZE)));

#define TX_NUM_PTR_POW (4 + 2)                 // 64 lengths (4 bytes each) -> 256B
#define TX_NUM_PTR_BYTES (1u << TX_NUM_PTR_POW)
#define TX_NUM_PTR (1u << (TX_NUM_PTR_POW - 2))
#define TX_NUM_MASK (TX_NUM_PTR - 1u)

static volatile uint32_t tx_pkt_ptr[TX_NUM_PTR] __attribute__((aligned(TX_NUM_PTR_BYTES)));
static volatile uint32_t tx_addr = 0;
static volatile uint32_t tx_curr_pkt_ptr = 0;
static volatile uint32_t ipg_tx_rd_addr;

// ===== DMA =====
static int rx_dma_chan;
static int rx_chain_chan;
static int tx_dma_chan;
static int ipg_dma_chan;
static int tx_chain_chan;

static dma_channel_config rx_dma_cfg;
static dma_channel_config rx_chain_cfg;
static dma_channel_config tx_dma_cfg;
static dma_channel_config ipg_dma_cfg;
static dma_channel_config tx_chain_cfg;

// CRC/copy DMA sniffer channel
static int crc_chan;
static dma_channel_config crc_rx_cfg;
static dma_channel_config crc_tx_cfg;
static dma_channel_config crc_tx_noinc_cfg;

// Reload RX DMA after this many bytes
static uint32_t rx_packet_size = RX_BUF_SIZE;

// Ethernet CRC check value (reversed CRC32, expected final)
static const uint32_t crc_check_value = 0xdebb20e3;

// ===== PIO offsets =====
static uint rx_sm_offset;
static uint tx_sm_offset;

// ===== Driver state =====
static rmii_rx_cb_t g_rx_cb = NULL;
static rmii_stats_t g_stats;
static uint8_t g_mac[6];

void rmii_get_default_config(rmii_ethernet_config_t *cfg) {
    if (!cfg) return;
    cfg->pio = pio0;
    cfg->sm_rx = PICO_RMII_ETHERNET_SM_RX;
    cfg->sm_tx = PICO_RMII_ETHERNET_SM_TX;
    cfg->pin_rx_base = PICO_RMII_ETHERNET_RX_PIN;
    cfg->pin_tx_base = PICO_RMII_ETHERNET_TX_PIN;
    cfg->pin_refclk = PICO_RMII_ETHERNET_RETCLK_PIN;
    cfg->pin_reset = PICO_RMII_ETHERNET_RST_PIN;
    cfg->sys_clock_khz = 300000;
    cfg->vreg_voltage = VREG_VOLTAGE_1_20;
    cfg->init_sys_clock = true;
    cfg->init_stdio = RMII_ETH_HAS_STDIO ? true : false;
}

static void rmii_apply_config(const rmii_ethernet_config_t *cfg) {
    if (cfg) g_cfg = *cfg;
    else rmii_get_default_config(&g_cfg);
}

// ========= helpers =========

static void make_mac(uint8_t mac[6]) {
    pico_unique_board_id_t id;
    pico_get_unique_board_id(&id);
    mac[0] = 0xb8;
    mac[1] = 0x27;
    mac[2] = 0xeb;
    mac[3] = id.id[5];
    mac[4] = id.id[6];
    mac[5] = id.id[7];
}

static void ring_copy_rx(uint8_t *dst, uint32_t addr, uint32_t len) {
    for (uint32_t i = 0; i < len; i++) {
        dst[i] = rx_ring[(addr + i) & RX_BUF_MASK];
    }
}

static bool rx_crc_ok(uint32_t addr, uint32_t len_bytes) {
    static volatile uint8_t sink;

    dma_channel_wait_for_finish_blocking(crc_chan);
    dma_hw->sniff_data = 0xffffffff;

    dma_channel_hw_addr(crc_chan)->read_addr  = (uint32_t)&rx_ring[addr & RX_BUF_MASK];
    dma_channel_hw_addr(crc_chan)->write_addr = (uint32_t)&sink;
    dma_channel_hw_addr(crc_chan)->transfer_count = len_bytes;

    dma_channel_set_config(crc_chan, &crc_rx_cfg, true);
    dma_channel_wait_for_finish_blocking(crc_chan);

    return (dma_hw->sniff_data == crc_check_value);
}

static uint32_t tx_copy_add_crc(const uint8_t *frame_no_fcs, uint32_t len_no_fcs, uint32_t wr_addr) {
    dma_channel_wait_for_finish_blocking(crc_chan);
    dma_hw->sniff_data = 0xffffffff;

    // Copy payload into TX ring (wrap)
    dma_channel_wait_for_finish_blocking(crc_chan);
    dma_channel_hw_addr(crc_chan)->read_addr  = (uint32_t)frame_no_fcs;
    dma_channel_hw_addr(crc_chan)->write_addr = (uint32_t)&tx_ring[wr_addr];
    dma_channel_hw_addr(crc_chan)->transfer_count = len_no_fcs;
    dma_channel_set_config(crc_chan, &crc_tx_cfg, true);

    wr_addr = (wr_addr + len_no_fcs) & TX_BUF_MASK;
    uint32_t tot_len = len_no_fcs;

    // Pad to minimum Ethernet frame size (60 bytes excluding FCS)
    if (tot_len < 60) {
        uint32_t zero = 0;
        uint32_t pad = 60 - tot_len;

        dma_channel_wait_for_finish_blocking(crc_chan);
        dma_channel_hw_addr(crc_chan)->read_addr  = (uint32_t)&zero;
        dma_channel_hw_addr(crc_chan)->write_addr = (uint32_t)&tx_ring[wr_addr];
        dma_channel_hw_addr(crc_chan)->transfer_count = pad;
        dma_channel_set_config(crc_chan, &crc_tx_noinc_cfg, true);

        wr_addr = (wr_addr + pad) & TX_BUF_MASK;
        tot_len += pad;
    }

    dma_channel_wait_for_finish_blocking(crc_chan);
    uint32_t crc = dma_hw->sniff_data;
    uint32_t fcs = ~crc;

    // Append FCS little-endian
    for (int i = 0; i < 4; i++) {
        tx_ring[wr_addr] = (uint8_t)(fcs >> (i * 8));
        wr_addr = (wr_addr + 1) & TX_BUF_MASK;
    }
    tot_len += 4;

    return tot_len;
}

// ===== RX EOF ISR (SRAM) =====
static void __not_in_flash_func(rmii_eof_isr)(void) {
    uint32_t prev_rx_addr = rx_addr;

    pio_interrupt_clear(g_cfg.pio, 0);

    rx_addr = dma_hw->ch[rx_dma_chan].write_addr - (uint32_t)(&rx_ring[0]);

    uint32_t pkt_len;
    if (rx_addr < prev_rx_addr) pkt_len = (RX_BUF_SIZE + rx_addr) - prev_rx_addr;
    else                        pkt_len = rx_addr - prev_rx_addr;

    rx_pkt_ptr[rx_curr_pkt_ptr].pkt_addr = (uint16_t)prev_rx_addr;
    rx_pkt_ptr[rx_curr_pkt_ptr].pkt_len  = (uint16_t)pkt_len;
    rx_curr_pkt_ptr = (rx_curr_pkt_ptr + 1) & RX_NUM_MASK;
}

// ===== essential PHY clock + reset =====
// Keep this close to your previously working arch_pico_init()
static void phy_clock_and_reset_init(void) {
    if (g_cfg.init_sys_clock) {
        vreg_set_voltage(g_cfg.vreg_voltage);
        set_sys_clock_khz(g_cfg.sys_clock_khz, true);
    }

    gpio_init(g_cfg.pin_reset);
    gpio_put(g_cfg.pin_reset, 0);
    gpio_set_dir(g_cfg.pin_reset, GPIO_OUT);

    uint32_t sys_hz = clock_get_hz(clk_sys);
    uint gpio = g_cfg.pin_refclk;
    uint gpclk = clk_gpout0;

    if      (gpio == 21) gpclk = clk_gpout0;
    else if (gpio == 23) gpclk = clk_gpout1;
    else if (gpio == 24) gpclk = clk_gpout2;
    else if (gpio == 25) gpclk = clk_gpout3;

    float div = (float)sys_hz / 50000000.0f;
    clock_gpio_init(g_cfg.pin_refclk,
                    CLOCKS_CLK_GPOUT0_CTRL_AUXSRC_VALUE_CLK_SYS,
                    div);
    clocks_hw->clk[gpclk].ctrl |= CLOCKS_CLK_GPOUT0_CTRL_DC50_BITS;

    sleep_ms(1);

#if RMII_ETH_HAS_STDIO
    if (g_cfg.init_stdio) {
        stdio_init_all();
        sleep_ms(10);
    }
#endif

    sleep_ms(25);
    gpio_put(g_cfg.pin_reset, 1);

    sleep_ms(10);
}

// ========= public API =========

void rmii_set_rx_callback(rmii_rx_cb_t cb) { g_rx_cb = cb; }

void rmii_get_mac(uint8_t mac[6]) { memcpy(mac, g_mac, 6); }

void rmii_get_stats(rmii_stats_t *out) { *out = g_stats; }

void rmii_init_with_config(const rmii_ethernet_config_t *cfg) {
    rmii_apply_config(cfg);

    memset(&g_stats, 0, sizeof(g_stats));
    make_mac(g_mac);

    phy_clock_and_reset_init();

    // PIO programs
    rx_sm_offset = pio_add_program(g_cfg.pio, &rmii_ethernet_phy_rx_data_program);
    tx_sm_offset = pio_add_program(g_cfg.pio, &rmii_ethernet_phy_tx_data_program);

    // Run both SM at 100MHz (2x RMII)
    float div_100 = (float)clock_get_hz(clk_sys) / 100e6f;

    rmii_ethernet_phy_tx_init(g_cfg.pio,
                              g_cfg.sm_tx,
                              tx_sm_offset,
                              g_cfg.pin_tx_base,
                              g_cfg.pin_refclk,
                              div_100);

    rmii_ethernet_phy_rx_init(g_cfg.pio,
                              g_cfg.sm_rx,
                              rx_sm_offset,
                              g_cfg.pin_rx_base,
                              div_100);

    // DMA channels
    rx_dma_chan   = dma_claim_unused_channel(true);
    rx_chain_chan = dma_claim_unused_channel(true);
    tx_dma_chan   = dma_claim_unused_channel(true);
    ipg_dma_chan  = dma_claim_unused_channel(true);
    tx_chain_chan = dma_claim_unused_channel(true);

    // ---- RX DMA: PIO RX FIFO -> rx_ring (byte, ring wrap), chain to reload ----
    rx_dma_cfg = dma_channel_get_default_config(rx_dma_chan);
    channel_config_set_read_increment(&rx_dma_cfg, false);
    channel_config_set_write_increment(&rx_dma_cfg, true);
    channel_config_set_ring(&rx_dma_cfg, true, RX_BUF_SIZE_POW);
    channel_config_set_dreq(&rx_dma_cfg,
        pio_get_dreq(g_cfg.pio, g_cfg.sm_rx, false));
    channel_config_set_transfer_data_size(&rx_dma_cfg, DMA_SIZE_8);
    channel_config_set_chain_to(&rx_dma_cfg, rx_chain_chan);

    dma_channel_configure(rx_dma_chan, &rx_dma_cfg,
        &rx_ring[0],
        ((uint8_t*)&g_cfg.pio->rxf[g_cfg.sm_rx]) + 3,
        1500, false);

    // ---- RX chain reload DMA (CRITICAL: NO INCREMENT) ----
    rx_chain_cfg = dma_channel_get_default_config(rx_chain_chan);
    channel_config_set_read_increment(&rx_chain_cfg, false);
    channel_config_set_write_increment(&rx_chain_cfg, false);

    dma_channel_configure(rx_chain_chan, &rx_chain_cfg,
        &dma_hw->ch[rx_dma_chan].al1_transfer_count_trig,
        &rx_packet_size,
        1, false);

    // ---- TX DMA: tx_ring -> PIO TX FIFO ----
    tx_dma_cfg = dma_channel_get_default_config(tx_dma_chan);
    channel_config_set_read_increment(&tx_dma_cfg, true);
    channel_config_set_write_increment(&tx_dma_cfg, false);
    channel_config_set_ring(&tx_dma_cfg, false, TX_BUF_SIZE_POW);
    channel_config_set_dreq(&tx_dma_cfg,
        pio_get_dreq(g_cfg.pio, g_cfg.sm_tx, true));
    channel_config_set_transfer_data_size(&tx_dma_cfg, DMA_SIZE_8);
    channel_config_set_chain_to(&tx_dma_cfg, ipg_dma_chan);

    dma_channel_configure(tx_dma_chan, &tx_dma_cfg,
        ((uint8_t*)&g_cfg.pio->txf[g_cfg.sm_tx]) + 3,
        &tx_ring[0],
        1518, false);

    // ---- IPG DMA paced timer (kept) ----
    ipg_dma_cfg = dma_channel_get_default_config(ipg_dma_chan);
    channel_config_set_read_increment(&ipg_dma_cfg, false);
    channel_config_set_write_increment(&ipg_dma_cfg, false);
    channel_config_set_chain_to(&ipg_dma_cfg, tx_chain_chan);

    uint32_t dma_timer = dma_claim_unused_timer(true);
    uint16_t div = (uint16_t)(clock_get_hz(clk_sys) / 12500000u);
    dma_timer_set_fraction(dma_timer, 1u, div);
    channel_config_set_dreq(&ipg_dma_cfg, dma_get_timer_dreq(dma_timer));

    dma_channel_configure(ipg_dma_chan, &ipg_dma_cfg,
        &ipg_tx_rd_addr,
        (uint32_t *)&(dma_hw->ch[tx_dma_chan].read_addr),
        10, true);

    // ---- TX chain DMA: tx_pkt_ptr -> tx_dma transfer_count_trig ----
    tx_chain_cfg = dma_channel_get_default_config(tx_chain_chan);
    channel_config_set_read_increment(&tx_chain_cfg, true);
    channel_config_set_ring(&tx_chain_cfg, false, TX_NUM_PTR_POW);
    channel_config_set_write_increment(&tx_chain_cfg, false);

    dma_channel_configure(tx_chain_chan, &tx_chain_cfg,
        &dma_hw->ch[tx_dma_chan].al1_transfer_count_trig,
        &tx_pkt_ptr[0],
        1, false);

    // ---- CRC sniffer DMA channel ----
    crc_chan = dma_claim_unused_channel(true);

    // RX sniff: ring read wrap
    crc_rx_cfg = dma_channel_get_default_config(crc_chan);
    channel_config_set_read_increment(&crc_rx_cfg, true);
    channel_config_set_ring(&crc_rx_cfg, false, RX_BUF_SIZE_POW);
    channel_config_set_write_increment(&crc_rx_cfg, false);
    channel_config_set_transfer_data_size(&crc_rx_cfg, DMA_SIZE_8);
    channel_config_set_sniff_enable(&crc_rx_cfg, true);

    // TX sniff: linear read -> ring write wrap
    crc_tx_cfg = dma_channel_get_default_config(crc_chan);
    channel_config_set_read_increment(&crc_tx_cfg, true);
    channel_config_set_write_increment(&crc_tx_cfg, true);
    channel_config_set_ring(&crc_tx_cfg, true, TX_BUF_SIZE_POW);
    channel_config_set_transfer_data_size(&crc_tx_cfg, DMA_SIZE_8);
    channel_config_set_sniff_enable(&crc_tx_cfg, true);

    crc_tx_noinc_cfg = crc_tx_cfg;
    channel_config_set_read_increment(&crc_tx_noinc_cfg, false);

    dma_sniffer_enable(crc_chan, DMA_SNIFF_CTRL_CALC_VALUE_CRC32R, true);
    dma_sniffer_set_output_reverse_enabled(true);

    // ---- PIO IRQ0 -> ISR ----
    uint irq_index = rmii_get_pio_irq_index();
    irq_set_exclusive_handler(irq_index, rmii_eof_isr);
    pio_set_irq0_source_enabled(g_cfg.pio, pis_interrupt0, true);
    irq_set_enabled(irq_index, true);

    // Start RX chain (this arms RX DMA)
    dma_channel_start(rx_chain_chan);
}

void rmii_init(void) { rmii_init_with_config(NULL); }

bool rmii_send_frame(const uint8_t *frame_no_fcs, uint16_t len_no_fcs) {
    if (!frame_no_fcs || len_no_fcs < 14 || len_no_fcs > RMII_MAX_FRAME_NOFCS) {
        g_stats.tx_dropped++;
        return false;
    }

    // target bytes in ring including FCS and padding
    uint32_t want = (uint32_t)len_no_fcs + 4;
    if (want < 64) want = 64;

    // Approx free space check (kept in spirit of your original)
    uint32_t curr_rd = (dma_hw->ch[tx_dma_chan].read_addr) & TX_BUF_MASK;
    uint32_t curr_wr = tx_addr;

    uint32_t wrap = (curr_rd ^ curr_wr) & TX_BUF_MASK;
    uint32_t used = wrap ? ((curr_wr - curr_rd) & TX_BUF_MASK) : (curr_rd - curr_wr);
    uint32_t free = TX_BUF_SIZE - used;

    if (want > free) {
        // wait for drain (no drop)
        dma_channel_wait_for_finish_blocking(tx_dma_chan);
    }

    uint32_t len_with_fcs = tx_copy_add_crc(frame_no_fcs, len_no_fcs, tx_addr);
    tx_addr = (tx_addr + len_with_fcs) & TX_BUF_MASK;

    // enqueue TX length into command ring (same logic as known-good)
    uint32_t curr_cmd;
    uint32_t tx_next_pkt_ptr;

    if (!dma_channel_is_busy(tx_dma_chan) && !dma_channel_is_busy(ipg_dma_chan)) {
        curr_cmd = dma_channel_hw_addr(tx_chain_chan)->read_addr;
        tx_curr_pkt_ptr = (curr_cmd >> 2) & TX_NUM_MASK;

        tx_next_pkt_ptr = (tx_curr_pkt_ptr + 1) & TX_NUM_MASK;

        tx_pkt_ptr[tx_next_pkt_ptr] = 0;              // EOC
        tx_pkt_ptr[tx_curr_pkt_ptr] = len_with_fcs;   // command

        tx_curr_pkt_ptr = tx_next_pkt_ptr;
        dma_channel_hw_addr(tx_chain_chan)->al1_transfer_count_trig = 1;
    } else {
        tx_next_pkt_ptr = (tx_curr_pkt_ptr + 1) & TX_NUM_MASK;

        tx_pkt_ptr[tx_next_pkt_ptr] = 0;
        tx_pkt_ptr[tx_curr_pkt_ptr] = len_with_fcs;

        if (!dma_channel_is_busy(tx_dma_chan) && !dma_channel_is_busy(ipg_dma_chan)) {
            curr_cmd = dma_channel_hw_addr(tx_chain_chan)->read_addr;
            curr_cmd = (curr_cmd >> 2) & TX_NUM_MASK;
            if (curr_cmd == tx_next_pkt_ptr) {
                dma_channel_hw_addr(tx_chain_chan)->al3_read_addr_trig =
                    (uint32_t)&tx_pkt_ptr[tx_curr_pkt_ptr];
            }
        }

        tx_curr_pkt_ptr = tx_next_pkt_ptr;
    }

    g_stats.tx_total++;
    return true;
}

void rmii_poll(void) {
    uint32_t safe_rx_curr = rx_curr_pkt_ptr;

    uint32_t rx_count;
    if (safe_rx_curr < rx_prev_pkt_ptr) rx_count = (RX_NUM_PTR + safe_rx_curr) - rx_prev_pkt_ptr;
    else                                rx_count = safe_rx_curr - rx_prev_pkt_ptr;

    static uint8_t frame_buf[RMII_MAX_FRAME_NOFCS];

    while (rx_count--) {
        uint32_t len = rx_pkt_ptr[rx_prev_pkt_ptr].pkt_len;
        uint32_t addr = rx_pkt_ptr[rx_prev_pkt_ptr].pkt_addr;
        rx_prev_pkt_ptr = (rx_prev_pkt_ptr + 1) & RX_NUM_MASK;

        g_stats.rx_total++;

        if (len < 64) { g_stats.rx_short++; continue; }

        if (!rx_crc_ok(addr, len)) {
            g_stats.rx_crc_bad++;
            continue;
        }
        g_stats.rx_crc_ok++;

        // Remove FCS before delivering
        uint32_t nofcs = len - 4;
        if (nofcs > RMII_MAX_FRAME_NOFCS) nofcs = RMII_MAX_FRAME_NOFCS;

        ring_copy_rx(frame_buf, addr, nofcs);

        if (g_rx_cb) g_rx_cb(frame_buf, (uint16_t)nofcs);
    }
}
