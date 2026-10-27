#include <Arduino.h>
#include <TFT_eSPI.h>

#define TFT_BACKLIGHT 21

TFT_eSPI tft = TFT_eSPI();

void setup()
{
    Serial.begin(115200);
    delay(500);

    pinMode(TFT_BACKLIGHT, OUTPUT);
    digitalWrite(TFT_BACKLIGHT, HIGH);

    tft.init();
    tft.setRotation(1);

    tft.fillScreen(TFT_BLACK);

    tft.setTextDatum(MC_DATUM);

    tft.setTextColor(TFT_GREEN, TFT_BLACK);
    tft.drawString(
        "ESP32CYD-PANIC",
        tft.width() / 2,
        90,
        4
    );

    tft.setTextColor(TFT_WHITE, TFT_BLACK);
    tft.drawString(
        "Display test successful!",
        tft.width() / 2,
        140,
        2
    );

    Serial.println("Display test complete.");
}

void loop()
{
}