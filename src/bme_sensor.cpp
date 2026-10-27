#include "bme_sensor.h"

#include <math.h>

#include "board_pins.h"

namespace
{
constexpr float SeaLevelPressureHpa = 1013.25F;
}

// Keep the sensor on HSPI; TFT_eSPI owns VSPI.
BmeSensor::BmeSensor()
    : spi_(HSPI), sensor_(BoardPins::BmeCs, &spi_)
{
}

bool BmeSensor::begin()
{
    pinMode(BoardPins::BmeCs, OUTPUT);
    digitalWrite(BoardPins::BmeCs, HIGH);
    spi_.begin(BoardPins::BmeSck, BoardPins::BmeMiso,
               BoardPins::BmeMosi, BoardPins::BmeCs);

    initialized_ = sensor_.begin();
    return initialized_;
}

bool BmeSensor::read(BmeReadings &readings)
{
    if (!initialized_)
    {
        return false;
    }

    readings.temperatureC = sensor_.readTemperature();
    readings.humidityPercent = sensor_.readHumidity();
    readings.pressureHpa = sensor_.readPressure() / 100.0F;
    readings.altitudeMeters = sensor_.readAltitude(SeaLevelPressureHpa);

    return isfinite(readings.temperatureC) &&
           isfinite(readings.humidityPercent) &&
           isfinite(readings.pressureHpa) &&
           isfinite(readings.altitudeMeters) &&
           readings.pressureHpa > 0.0F;
}
