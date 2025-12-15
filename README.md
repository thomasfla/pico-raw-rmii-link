# PicoRawRMII – RMII Ethernet Driver for Arduino Pico Core

PicoRawRMII is a raw RMII/LAN8720 driver for the Raspberry Pi Pico that targets the Earle Philhower Arduino core. It configures the LAN8720 entirely through its power-up strapping pins (no MDIO bus), bypasses lwIP so sketches talk to the PHY directly, and keeps packet handling zero-copy through PIO + DMA.

PicoRawRMII stands on the shoulders of [rscott2049/pico-rmii-ethernet_nce](https://github.com/rscott2049/pico-rmii-ethernet_nce) and [sandeepmistry/pico-rmii-ethernet](https://github.com/sandeepmistry/pico-rmii-ethernet). The PIO programs used by this library are directly extracted from rscott2049’s repository, which itself incorporates Sandeep Mistry’s original implementation.

If you need a matching PCB, check out the companion [pico-lan8720-minimal](https://github.com/thomasfla/pico-lan8720-minimal) carrier. All code in this repo has been validated on that Pico 2 + LAN8720 board.

This project exists to compensate for the lack of high-speed USB on the Pico/Pico 2 by providing a time-critical, high-bandwidth RMII link between a host computer and the microcontroller. Working with raw Ethernet has trade-offs though: there is no lwIP stack, so packet framing, retransmission, and higher-level protocols are entirely up to you. PicoRawRMII is therefore intended for point-to-point connections where you control both ends, typically with a simple Linux userland program that crafts and parses the frames you care about.

## Features
- LAN8720 bring-up without MDIO – the PHY is configured solely via hardware straps so only the RMII signals are required.
- Zero-copy RMII RX/TX paths built with PIO + chained DMA so frames stay in contiguous buffers you control.
- No lwIP dependency – sketches send and receive raw Ethernet frames through a minimal C/C++ API.
- Automatic MAC address derived from the RP2040 unique ID plus configurable pin/state-machine assignment.
- Arduino-friendly class with example sketches demonstrating periodic broadcasts and frame echoing.

## Default Pinout
| Signal | Pico pin |
| ------ | -------- |
| REF_CLK (output to LAN8720) | GP21 |
| PHY reset | GP20 |
| RX0/RX1/CRS_DV | GP12 / GP13 / GP14 |
| TX0/TX1/TXEN | GP26 / GP27 / GP28 |

The driver outputs a 50 MHz RMII reference clock on `REF_CLK`. Change the pin mapping by editing `rmii_ethernet_phy_rx.pio` (and regenerating the `.pio.h` headers) or by supplying a custom `rmii_ethernet_config_t` at runtime.

## Using the Library in Arduino
1. Install the [Arduino-Pico core](https://github.com/earlephilhower/arduino-pico) and select your Pico board (a Pico 2 is recommended for the 300 MHz system clock).
2. Copy/clone this repository into your Arduino `libraries/` folder. The folder should contain `library.properties`, `src/`, and `examples/`.
3. Wire the Pico to the LAN8720 according to the table above. Keep the RMII traces short and make sure the PHY is powered from 3V3.
4. Open `File → Examples → PicoRawRMII → RawFrameSender`, select your serial port, and upload.

The example registers an RX callback, prints the generated MAC address, and blasts a custom EtherType frame once a second so you can verify packet timing with Wireshark.

## Arduino API Overview
```c++
#include <PicoRawRMII.h>

static void onFrame(const uint8_t *frame, uint16_t len) {
  Serial.printf("RX %u bytes\n", len);
}

void setup() {
  Serial.begin(115200);
  while (!Serial) delay(10);

  rmii_ethernet_config_t cfg;
  rmii_get_default_config(&cfg);
  cfg.init_stdio = false;           // Arduino handles Serial on its own

  PicoRawRMII.onReceive(onFrame);
  PicoRawRMII.begin(&cfg);
}

void loop() {
  PicoRawRMII.poll();              // MUST be called often to drain RX
  // ... call PicoRawRMII.sendFrame() whenever you need to transmit
}
```

- `PicoRawRMII.begin(cfg)` – configures the PHY/PIO/DMA. Pass `nullptr` to use the defaults shown above.  
- `PicoRawRMII.onReceive(cb)` – register a callback that receives complete Ethernet frames without the FCS.  
- `PicoRawRMII.sendFrame(buf, len)` – queue a frame for transmission; driver pads to 60 bytes and appends the CRC.  
- `PicoRawRMII.poll()` – processes pending RX packets. Call it from `loop()` as frequently as possible.  
- `PicoRawRMII.getMAC()` / `getStats()` – retrieve the auto-generated MAC address or current statistics.

### Examples
- `RawFrameSender` – Prints your MAC address and sprays a custom EtherType frame once a second so you can sniff it with Wireshark.
- `RingbackEcho` – Minimal latency echo that immediately re-transmits every received frame, useful for timing tests. On the pico-lan8720-minimal board, a logic analyser shows roughly 16 µs between packet arrival and retransmission.

### Low-Level C API
The original C functions are still available from `PicoRawRMII_Driver.h` if you prefer to integrate the driver in C or another runtime. Use `rmii_get_default_config()` and `rmii_init_with_config()` to override pins, automatic stdio init, or the boosted system clock.

## Customising the Build
- The `.pio.h` files in `src/` are pre-generated so the Arduino build does not need `pioasm`. If you change the `.pio` sources, regenerate the headers with the Pico SDK `pioasm` tool and replace the versions in `src/`.
- By default the driver raises the core voltage to 1.2 V and overclocks the RP2040 to 300 MHz for better RMII timing margins. Set `cfg.init_sys_clock = false` if you want to keep the board’s existing clock configuration.
- Set `cfg.init_stdio = false` when running under Arduino so `Serial` is left alone.

## License
BSD-3-Clause – see the original headers for additional copyright notices.
