#include <stdio.h>
#include <string.h>
#include "pico/stdlib.h"
#include "PicoRawRMII_Driver.h"

static void rx_cb(const uint8_t *f, uint16_t len) {
    //rmii_send_frame("YO00000000000000000000000000000000000000000000000000!!!", 20);
    printf("RX %u bytes  dst=%02X:%02X:%02X:%02X:%02X:%02X  src=%02X:%02X:%02X:%02X:%02X:%02X  type=%02X%02X\r\n",
           len,
           f[0],f[1],f[2],f[3],f[4],f[5],
           f[6],f[7],f[8],f[9],f[10],f[11],
           f[12],f[13]);
}

int main() {
    rmii_init();
    rmii_set_rx_callback(rx_cb);

    uint8_t mac[6];
    rmii_get_mac(mac);

    printf("RMII driver (no lwIP\r\n");
    printf("MAC %02X:%02X:%02X:%02X:%02X:%02X\r\n",
           mac[0],mac[1],mac[2],mac[3],mac[4],mac[5]);

    // broadcast frame
    uint8_t tx[64];
    memset(tx, 0, sizeof(tx));
    memset(&tx[0], 0xFF, 6);
    memcpy(&tx[6], mac, 6);
    tx[12] = 0x88; tx[13] = 0xB5;
    memcpy(&tx[14], "HELLO", 5);

    absolute_time_t next = make_timeout_time_ms(1000);

    while (1) {
        rmii_poll();

        if (absolute_time_diff_us(get_absolute_time(), next) <= 0) {
            rmii_send_frame(tx, sizeof(tx));
            rmii_stats_t st;
            rmii_get_stats(&st);
            printf("tick: rx=%lu ok=%lu bad=%lu tx=%lu\r\n",
                   st.rx_total, st.rx_crc_ok, st.rx_crc_bad, st.tx_total);
            next = make_timeout_time_ms(1000);
        }

        tight_loop_contents();
    }
}
