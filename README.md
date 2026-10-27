# Portable Altitude, Navigation, and In-situ Conditions

ESP32-based portable environmental and navigation instrument for the ELEGOO E32R28T.

## Current status

- TFT display and resistive touchscreen are configured; a valid touch calibration is saved in NVS and reused at startup. Tap `CAL` beside the touch coordinates to recalibrate. Captured raw points are shown during calibration, with a 15-second result screen afterward.
- The main status page remains the startup page. Tap `SYS` at the bottom right for ESP32 chip, clock, flash, PSRAM, heap, uptime, reset, and SDK information; tap `APP` to return.
- BME280 SPI and PA1616D GPS UART acquisition and diagnostics are implemented, but sensor hardware operation has not yet been verified.
- Power displays the GPIO34 voltage and a rough voltage-based battery percentage. It does not identify USB versus battery power.
- SYS combines system statistics with radio state, screen/backlight status, and the power estimate. Wi-Fi and Bluetooth are not started; current draw is not measured.
- Below an estimated 90%, the backlight turns off after 60 seconds without touch. The dark screen refreshes every 5 seconds; sensors and Serial continue normally. Any touch wakes it without activating controls. Missing or >=90% estimates keep it lit. No PWM dimming or CPU sleep is used.
- Firmware builds with PlatformIO: `pio run`.

## Project goals

Bring up and verify the BME280 and GPS independently, then together. Collect real GPS and barometric altitude readings before considering altitude fusion. Keep sensor acquisition separate from display code and preserve raw diagnostics.

The microSD card and altitude fusion are not implemented. Preserve GPIO18/19/23 for future SD use (CS GPIO5); the BME280 uses those SPI signals with CS GPIO27.

Keep the known-working TFT_eSPI pins, build flags, SPI frequency, and upload configuration unchanged unless a demonstrated issue requires a change.
