#include "PicoRawRMII.h"

bool PicoRawRMIIClass::begin(const rmii_ethernet_config_t *cfg) {
    if (cfg) {
        rmii_init_with_config(cfg);
    } else {
        rmii_ethernet_config_t local_cfg;
        rmii_get_default_config(&local_cfg);
#ifdef ARDUINO
        local_cfg.init_stdio = false;
#endif
        rmii_init_with_config(&local_cfg);
    }
    return true;
}

void PicoRawRMIIClass::onReceive(rmii_rx_cb_t cb) { rmii_set_rx_callback(cb); }

bool PicoRawRMIIClass::sendFrame(const uint8_t *frame_no_fcs, uint16_t len_no_fcs) {
    return rmii_send_frame(frame_no_fcs, len_no_fcs);
}

void PicoRawRMIIClass::poll() { rmii_poll(); }

void PicoRawRMIIClass::getMAC(uint8_t mac[6]) { rmii_get_mac(mac); }

void PicoRawRMIIClass::getStats(rmii_stats_t &stats) { rmii_get_stats(&stats); }

PicoRawRMIIClass PicoRawRMII;
