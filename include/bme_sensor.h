#pragma once

#include <Arduino.h>
#include <SPI.h>
#include <Adafruit_BME280.h>

struct BmeReadings
{
    float temperatureC;
    float humidityPercent;
    float pressureHpa;
    float altitudeMeters;
};

class BmeSensor
{
public:
    BmeSensor();
    bool begin();
    bool read(BmeReadings &readings);

private:
    SPIClass spi_;
    Adafruit_BME280 sensor_;
    bool initialized_ = false;
};
