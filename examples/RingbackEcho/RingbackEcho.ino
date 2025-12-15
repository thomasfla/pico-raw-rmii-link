#include <PicoRawRMII.h>

static uint8_t tx_buf[RMII_MAX_FRAME_NOFCS];

static void echoFrame(const uint8_t *frame, uint16_t len) {
    if (len > sizeof(tx_buf)) return;
    memcpy(tx_buf, frame, len);
    PicoRawRMII.sendFrame(tx_buf, len);
}

void setup() {
    rmii_ethernet_config_t cfg;
    rmii_get_default_config(&cfg);
    cfg.init_stdio = false;

    PicoRawRMII.onReceive(echoFrame);
    PicoRawRMII.begin(&cfg);
}

void loop() {
    PicoRawRMII.poll();
}
