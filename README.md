# MackoDash Rotary Controller

**Current firmware: 1.2.1**

Standalone ESP-IDF 5.5 project for the Elecrow 1.28-inch ESP32-S3 rotary display.
It is the optional controller add-on for MackoDash P4 firmware 3.4.0 or newer.

## Wireless

From 1.2.0 the knob connects to the dash over Wi-Fi; only power needs wiring.

1. On the dash, open **Settings > System > Knob** and turn on **Wireless knob**.
2. Power the knob. It joins the nearest MackoDash by itself (the dot at the bottom
   turns green) and sticks with that dash from then on.

Later knob versions install from the dash: **Settings > System > Knob > Check for
knob update** (the dash needs home Wi-Fi for the download). Install 1.2.0 once
with the MackoDash USB updater; it adds a second firmware slot and goes back to
the previous version by itself if a new one cannot reach the dash.

The wired link (UART) still works when the cable is connected.

Open `MackoDash-Rotary.code-workspace` in VS Code to use the ESP32-S3 target and
configure the controller's USB serial port without changing the ESP32-P4 dashboard
workspace. Use the ESP-IDF Build, Flash, and Monitor commands from that window.

## Controls

Home menu (turn to choose, press to open):

| Item | What it does |
| --- | --- |
| GAUGE | Live readout on the knob: boost, coolant, oil pressure/temp, AFR, intake, battery, fuel, knock, timing, injector duty, throttle. Turn to change. Stays open until you leave it. |
| VIEW | Turn to switch the dash theme live; press to keep it. Leaving without a press puts the old theme back. |
| BRIGHT | Dash screen brightness. |
| NIGHT | One press dims the dash, shift lights and knob; press again for day. |
| REC | Start / stop data logging. |
| TRIP | Odometer, Trip A and Trip B. Hold to reset a trip. |
| PEAKS | Session peaks. Hold to reset all. |

- Press on a page: done / back to the menu.
- Touch the screen: back to the menu.
- Any page except GAUGE goes back to the menu after 30 seconds without input.
- Dash warnings appear on the knob automatically; press to dismiss.

The ring around the edge shows the menu position, list position or the live
value. The dot at the bottom is green while the dash link is up.

## Verified Pin Map

| Function | GPIO |
| --- | ---: |
| LCD SCLK / MOSI / DC / CS / RST | 10 / 11 / 3 / 9 / 14 |
| Touch SDA / SCL / INT / RST | 6 / 7 / 5 / 13 |
| Encoder A / B / press | 45 / 42 / 41 |
| Backlight | 46 |
| Dashboard UART TX / RX (optional) | 43 / 44 |
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