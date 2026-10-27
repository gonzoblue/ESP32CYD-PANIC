#pragma once

#include <Arduino.h>

struct PowerReadings
{
    bool batteryVoltageValid;
    float batteryVoltage;
    uint8_t estimatedChargePercent;
};

class PowerMonitor
{
public:
    void begin();
    PowerReadings read() const;
};
