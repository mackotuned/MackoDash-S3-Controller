# MackoDash Rotary Controller

**Current firmware: 1.0.1**

Standalone ESP-IDF 5.5 project for the Elecrow 1.28-inch ESP32-S3 rotary display.
It is the optional controller add-on for MackoDash P4 firmware 3.0.0.

Open `MackoDash-Rotary.code-workspace` in VS Code to use the ESP32-S3 target and
configure the controller's USB serial port without changing the ESP32-P4 dashboard
workspace. Use the ESP-IDF Build, Flash, and Monitor commands from that window.

## Controls

- Rotate on Home: select View, Air Ride, Climate, Brightness, REC, Trip, Peaks,
  Shift, or Logs.
- Press on Home: open or run the selected dashboard action.
- Rotate in an action: change the dashboard selection or value.
- Press in an action: confirm.
- Long press: request the page-specific secondary action or return Home.
- Touch: return to Home.

The Air Ride page shows live pressure for all four corners and controls Ruin Air
Management over ESP-NOW. The S3 remains the sole owner of Air Ride actuation;
the P4 mirrors the controller page over UART.

The status dot at the bottom is green while dashboard packets are arriving and
red while disconnected. The controller remains usable for UI testing without a
dashboard connection.

## Verified Pin Map

| Function | GPIO |
| --- | ---: |
| LCD SCLK / MOSI / DC / CS / RST | 10 / 11 / 3 / 9 / 14 |
| Touch SDA / SCL / INT / RST | 6 / 7 / 5 / 13 |
| Encoder A / B / press | 45 / 42 / 41 |
| Backlight | 46 |
| Dashboard UART TX / RX | 43 / 44 |
| Board power enables | 1 / 2 |

UART uses 115200 baud, 8-N-1, and CRC16-CCITT framed `MD` packets. Debug output
uses the native USB Serial/JTAG console so it cannot corrupt dashboard traffic.

## Build

1. Install ESP-IDF 5.5.
2. Open `MackoDash-Rotary.code-workspace`.
3. Select the `esp32s3` target and the controller USB port.
4. Run ESP-IDF Build, then Flash.

## Flash a Release

Extract the release firmware ZIP, replace `COM4` with the controller's USB port,
and run:

```powershell
esptool.py --chip esp32s3 -p COM4 -b 460800 write_flash `
    0x0 bootloader.bin `
    0x8000 partition-table.bin `
    0x10000 mackodash_rotary.bin
```