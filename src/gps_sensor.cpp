#include "gps_sensor.h"

#include <stdio.h>
#include <string.h>

#include "board_pins.h"

namespace
{
constexpr uint32_t GpsBaudRate = 9600;
constexpr uint32_t NmeaDataTimeoutMs = 3000;
constexpr uint32_t FixQualityTimeoutMs = 5000;
constexpr size_t MaxNmeaSentenceLength = 126;
}

GpsSensor::GpsSensor()
    : uart_(1)
{
}

void GpsSensor::begin()
{
    uart_.begin(GpsBaudRate, SERIAL_8N1, BoardPins::GpsRx, -1);
}

void GpsSensor::update()
{
    while (uart_.available() > 0)
    {
        const char value = static_cast<char>(uart_.read());
        parser_.encode(value);
        processNmeaByte(value);
    }

    if (millis() - lastFixQualityAt_ > FixQualityTimeoutMs)
    {
        fixQualityValid_ = false;
    }
}

bool GpsSensor::read(GpsReadings &readings)
{
    const uint32_t now = millis();
    readings = {};
    readings.hasNmeaData = lastNmeaDataAt_ != 0 &&
                          now - lastNmeaDataAt_ <= NmeaDataTimeoutMs;
    readings.hasFix = parser_.location.isValid() &&
                      parser_.location.age() <= NmeaDataTimeoutMs &&
                      (!fixQualityValid_ || fixQuality_ > 0);
    readings.hasLatitudeLongitude = readings.hasFix;
    readings.hasAltitude = readings.hasFix &&
                           parser_.altitude.isValid() &&
                           parser_.altitude.age() <= NmeaDataTimeoutMs;
    readings.hasSpeed = readings.hasFix &&
                        parser_.speed.isValid() &&
                        parser_.speed.age() <= NmeaDataTimeoutMs;
    readings.hasCourse = readings.hasFix &&
                         parser_.course.isValid() &&
                         parser_.course.age() <= NmeaDataTimeoutMs;
    readings.hasSatellites = parser_.satellites.isValid() &&
                             parser_.satellites.age() <= FixQualityTimeoutMs;
    readings.hasFixQuality = fixQualityValid_;
    readings.hasTime = parser_.time.isValid() &&
                       parser_.time.age() <= NmeaDataTimeoutMs;

    if (readings.hasLatitudeLongitude)
    {
        readings.latitude = parser_.location.lat();
        readings.longitude = parser_.location.lng();
    }
    if (readings.hasAltitude)
    {
        readings.altitudeMeters = parser_.altitude.meters();
    }
    if (readings.hasSpeed)
    {
        readings.speedKmph = parser_.speed.kmph();
    }
    if (readings.hasCourse)
    {
        readings.courseDegrees = parser_.course.deg();
    }
    if (readings.hasSatellites)
    {
        readings.satellites = parser_.satellites.value();
    }
    if (readings.hasFixQuality)
    {
        readings.fixQuality = fixQuality_;
    }
    if (readings.hasTime)
    {
        readings.hour = parser_.time.hour();
        readings.minute = parser_.time.minute();
        readings.second = parser_.time.second();
    }
    return readings.hasNmeaData;
}

bool GpsSensor::takeNmeaSentence(char *destination, size_t capacity)
{
    if (!latestNmeaAvailable_ || destination == nullptr || capacity == 0)
    {
        return false;
    }

    snprintf(destination, capacity, "%s", latestNmea_);
    latestNmeaAvailable_ = false;
    return true;
}

void GpsSensor::processNmeaByte(char value)
{
    if (value == '$')
    {
        nmeaLength_ = 0;
        nmeaBuffer_[nmeaLength_++] = value;
        return;
    }

    if (nmeaLength_ == 0)
    {
        return;
    }

    if (value == '\r')
    {
        return;
    }

    if (value == '\n')
    {
        nmeaBuffer_[nmeaLength_] = '\0';
        memcpy(latestNmea_, nmeaBuffer_, nmeaLength_ + 1);
        latestNmeaAvailable_ = true;
        lastNmeaDataAt_ = millis();
        readFixQuality();
        nmeaLength_ = 0;
        return;
    }

    if (nmeaLength_ >= MaxNmeaSentenceLength)
    {
        nmeaLength_ = 0;
        return;
    }

    nmeaBuffer_[nmeaLength_++] = value;
}

void GpsSensor::readFixQuality()
{
    const size_t length = strlen(nmeaBuffer_);
    if (length < 6 || nmeaBuffer_[0] != '$' ||
        nmeaBuffer_[3] != 'G' ||
        nmeaBuffer_[4] != 'G' ||
        nmeaBuffer_[5] != 'A')
    {
        return;
    }

    const char *field = strchr(nmeaBuffer_, ',');
    for (uint8_t index = 0; field != nullptr && index < 6; ++index)
    {
        field = strchr(field + 1, ',');
    }
    if (field == nullptr || field[1] < '0' || field[1] > '8')
    {
        return;
    }

    fixQuality_ = static_cast<uint8_t>(field[1] - '0');
    fixQualityValid_ = true;
    lastFixQualityAt_ = millis();
}
