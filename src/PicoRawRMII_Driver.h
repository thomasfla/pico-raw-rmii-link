#pragma once
#include <stdint.h>
#include <stdbool.h>

#include "hardware/pio.h"
#include "hardware/vreg.h"

#ifdef __cplusplus
extern "C" {
#endif

// Max Ethernet frame bytes excluding FCS (dst..payload)
#define RMII_MAX_FRAME_NOFCS 1518u

typedef void (*rmii_rx_cb_t)(const uint8_t *frame_no_fcs, uint16_t len_no_fcs);

typedef struct {
    uint32_t rx_total;
    uint32_t rx_crc_ok;
    uint32_t rx_crc_bad;
    uint32_t rx_short;
    uint32_t tx_total;
    uint32_t tx_dropped;
} rmii_stats_t;

typedef struct {
    PIO pio;
    uint32_t sm_rx;
    uint32_t sm_tx;
    uint8_t pin_rx_base;
    uint8_t pin_tx_base;
    uint8_t pin_refclk;
    uint8_t pin_reset;
    uint32_t sys_clock_khz;
    enum vreg_voltage vreg_voltage;
    bool init_sys_clock;
    bool init_stdio;
} rmii_ethernet_config_t;

void rmii_get_default_config(rmii_ethernet_config_t *cfg);
void rmii_init_with_config(const rmii_ethernet_config_t *cfg);
void rmii_init(void);
void rmii_poll(void);

void rmii_set_rx_callback(rmii_rx_cb_t cb);
void rmii_get_mac(uint8_t mac[6]);
void rmii_get_stats(rmii_stats_t *out);

/**
 * Send an Ethernet frame WITHOUT FCS (dst..payload).
 * Driver pads to 60B (no FCS) and appends FCS (CRC32).
 * Returns true if queued, false if invalid/dropped.
 */
bool rmii_send_frame(const uint8_t *frame_no_fcs, uint16_t len_no_fcs);

#ifdef __cplusplus
}
#endif
