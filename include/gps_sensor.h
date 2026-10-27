#pragma once

#include <Arduino.h>
#include <TinyGPSPlus.h>

struct GpsReadings
{
    bool hasNmeaData;
    bool hasFix;
    bool hasLatitudeLongitude;
    bool hasAltitude;
    bool hasSpeed;
    bool hasCourse;
    bool hasSatellites;
    bool hasFixQuality;
    bool hasTime;
    double latitude;
    double longitude;
    double altitudeMeters;
    double speedKmph;
    double courseDegrees;
    uint32_t satellites;
    uint8_t fixQuality;
    uint8_t hour;
    uint8_t minute;
    uint8_t second;
};

class GpsSensor
{
public:
    GpsSensor();
    void begin();
    void update();
    bool read(GpsReadings &readings);
    bool takeNmeaSentence(char *destination, size_t capacity);

private:
    void processNmeaByte(char value);
    void readFixQuality();

    HardwareSerial uart_;
    TinyGPSPlus parser_;
    char nmeaBuffer_[128] = {};
    char latestNmea_[128] = {};
    size_t nmeaLength_ = 0;
    uint32_t lastNmeaDataAt_ = 0;
    uint32_t lastFixQualityAt_ = 0;
    uint8_t fixQuality_ = 0;
    bool fixQualityValid_ = false;
    bool latestNmeaAvailable_ = false;
};
