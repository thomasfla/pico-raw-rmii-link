#pragma once

#ifdef ARDUINO
#include <Arduino.h>
#endif

#include "PicoRawRMII_Driver.h"

class PicoRawRMIIClass {
public:
    bool begin(const rmii_ethernet_config_t *cfg = nullptr);
    void onReceive(rmii_rx_cb_t cb);
    bool sendFrame(const uint8_t *frame_no_fcs, uint16_t len_no_fcs);
    void poll();
    void getMAC(uint8_t mac[6]);
    void getStats(rmii_stats_t &stats);
};

extern PicoRawRMIIClass PicoRawRMII;
