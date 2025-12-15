#include <PicoRawRMII.h>
#include <cstring>

static uint8_t mac_addr[6];
static uint8_t tx_frame[64];
static unsigned long next_send_ms = 0;

static void dumpFrame(const uint8_t *frame, uint16_t len) {
    Serial.printf("RX %u bytes dst=%02X:%02X:%02X:%02X:%02X:%02X src=%02X:%02X:%02X:%02X:%02X:%02X type=%02X%02X\r\n",
                  len,
                  frame[0], frame[1], frame[2], frame[3], frame[4], frame[5],
                  frame[6], frame[7], frame[8], frame[9], frame[10], frame[11],
                  frame[12], frame[13]);
}

void setup() {
    Serial.begin(115200);
    while (!Serial) {
        delay(10);
    }

    rmii_ethernet_config_t cfg;
    rmii_get_default_config(&cfg);
    cfg.init_stdio = false;  // Arduino already sets up serial IO

    PicoRawRMII.onReceive(dumpFrame);
    PicoRawRMII.begin(&cfg);

    PicoRawRMII.getMAC(mac_addr);
    Serial.printf("LAN8720 RMII driver ready MAC %02X:%02X:%02X:%02X:%02X:%02X\r\n",
                  mac_addr[0], mac_addr[1], mac_addr[2],
                  mac_addr[3], mac_addr[4], mac_addr[5]);

    memset(tx_frame, 0, sizeof(tx_frame));
    memset(&tx_frame[0], 0xFF, 6);    // broadcast
    memcpy(&tx_frame[6], mac_addr, 6);
    tx_frame[12] = 0x88;
    tx_frame[13] = 0xB5;
    memcpy(&tx_frame[14], "HELLO", 5);

    next_send_ms = millis();
}

void loop() {
    PicoRawRMII.poll();

    if (millis() - next_send_ms >= 1000) {
        PicoRawRMII.sendFrame(tx_frame, sizeof(tx_frame));

        rmii_stats_t stats;
        PicoRawRMII.getStats(stats);
        Serial.printf("tick rx=%lu crc_ok=%lu crc_bad=%lu tx=%lu\r\n",
                      stats.rx_total, stats.rx_crc_ok, stats.rx_crc_bad, stats.tx_total);
        next_send_ms = millis();
    }
}
