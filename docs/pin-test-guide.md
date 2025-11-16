# Waveshare LCD Pin Diagnostic Guide

This repository now ships with a boot-time diagnostic that cycles through several known pin maps for the Waveshare ESP32-S3 Touch LCD family. The goal is to help you confirm which GPIO assignment matches your specific hardware revision before hard-coding it into `waveshare_lcd_port.h`.

## Reference pin map

The mapping below is aggregated from the official Espressif `ESP32_Display_Panel` board presets (`components/ESP32_Display_Panel/src/board/supported/waveshare/BOARD_WAVESHARE_ESP32_S3_TOUCH_LCD_4_3*.h`) and the Waveshare wiki for the 4.3" kit (<https://www.waveshare.com/wiki/ESP32-S3-Touch-LCD-4.3>). Both sources agree on the base wiring for the RGB565 bus, control signals, and the CH422G IO expander that drives `LCD_RST`, `LCD_BL`, and `USB_SEL`.

| Lane | Expected bit | GPIO | Notes |
| ---- | ------------- | ---- | ----- |
| D0   | B0            | 14   | RGB data bus lowest bit |
| D1   | B1            | 38   |  |
| D2   | B2            | 18   |  |
| D3   | B3            | 17   |  |
| D4   | B4            | 10   |  |
| D5   | G0            | 39   |  |
| D6   | G1            | 0    |  |
| D7   | G2            | 45   |  |
| D8   | G3            | 48   |  |
| D9   | G4            | 47   |  |
| D10  | G5            | 21   |  |
| D11  | R0            | 1    | Also touch-reset line via expander |
| D12  | R1            | 2    | Shared with LCD backlight enable on CH422G |
| D13  | R2            | 42   |  |
| D14  | R3            | 41   |  |
| D15  | R4            | 40   |  |
| HSYNC | — | 46 | Horizontal sync |
| VSYNC | — | 3  | Vertical sync |
| DE | — | 5 | Some panel revisions float `DE`; the diagnostic covers both cases |
| PCLK | — | 7 | Datasheet recommends sampling on the falling edge |

> **Touch panel:** The GT911 controller is connected over I2C0 on GPIO8/9 with interrupt on GPIO4 and reset routed through the CH422G expander (pin 1). Keep that in mind if you add touch diagnostics later.

## Running the test

1. Build/flash the firmware as usual (`idf.py flash monitor` or via PlatformIO).
2. On boot the firmware logs each candidate, configures the RGB bus with that mapping, and renders 16 horizontal stripes.
3. Each stripe sets exactly one RGB data bit high (`D0…D15`). The color you observe tells you which physical channel (R/G/B) that lane currently drives.
4. Watch the serial monitor: once a candidate succeeds the log prints the GPIO-to-lane table so you can copy those values straight into `waveshare_lcd_port.h`.
5. By default the sweep stops after the first visible pattern. Set `EXAMPLE_LCD_PIN_TEST_STOP_ON_SUCCESS` to `0` in `waveshare_lcd_port.h` if you want to keep cycling through all presets.

## Interpreting the pattern

- Bright blueish stripes correspond to bits that currently feed the blue channel; reddish or greenish stripes indicate swapped wiring.
- If no pattern appears, that candidate’s control pins (HSYNC/VSYNC/DE/PCLK) are probably incorrect for your board revision.
- When a candidate finally renders stripes without tearing, update the `EXAMPLE_LCD_RGB_IO_*` macros (and PCLK polarity) to lock it in.

## Tweaking the sweep

- The candidate list lives in `src/waveshare_lcd_port.cpp` (`PIN_PROBE_CANDIDATES`). Feel free to duplicate an entry and tweak the GPIO numbers, `DE` usage, or `pclk_active_neg` flag for additional experiments.
- Hold time per candidate is governed by `EXAMPLE_LCD_PIN_TEST_HOLD_TIME_MS` (default 3.5 seconds).
- The diagnostic uses the same ST7262 driver/timings defined in `waveshare_lcd_port.h`, so make sure those match your panel (800×480 @ 16 MHz PCLK per the Waveshare datasheet).

Once you have a stable image, rebuild with the confirmed mapping and disable the pin tester (or re-enable the normal demo by skipping `waveshare_lcd_pin_test()` in `app_main`).
