#include "power_monitor.h"

#include "board_pins.h"

namespace
{
constexpr uint8_t BatterySampleCount = 16;
constexpr float BatteryDividerScale = 2.0F;
constexpr float BatteryAbsentThreshold = 2.5F;
constexpr uint8_t FullChargeDisplayThreshold = 95;

struct ChargeCurvePoint
{
    float voltage;
    uint8_t percent;
};

constexpr ChargeCurvePoint ChargeCurve[] = {
    {3.30F, 0},
    {3.50F, 5},
    {3.60F, 10},
    {3.70F, 20},
    {3.75F, 30},
    {3.80F, 40},
    {3.85F, 50},
    {3.90F, 60},
    {3.95F, 70},
    {4.00F, 80},
    {4.10F, 90},
    {4.20F, 100}};

uint8_t estimateChargePercent(float voltage)
{
    if (voltage <= ChargeCurve[0].voltage)
    {
        return ChargeCurve[0].percent;
    }

    constexpr size_t CurvePointCount = sizeof(ChargeCurve) / sizeof(ChargeCurve[0]);
    for (size_t index = 1; index < CurvePointCount; ++index)
    {
        const ChargeCurvePoint &lower = ChargeCurve[index - 1];
        const ChargeCurvePoint &upper = ChargeCurve[index];
        if (voltage <= upper.voltage)
        {
            const float fraction = (voltage - lower.voltage) /
                                   (upper.voltage - lower.voltage);
            const uint8_t percent = static_cast<uint8_t>(
                lower.percent + fraction * (upper.percent - lower.percent) + 0.5F);
            return percent >= FullChargeDisplayThreshold ? 100 : percent;
        }
    }

    return 100;
}
}

void PowerMonitor::begin()
{
    pinMode(BoardPins::BatteryAdc, INPUT);
    analogReadResolution(12);
    analogSetPinAttenuation(BoardPins::BatteryAdc, ADC_11db);
}

PowerReadings PowerMonitor::read() const
{
    uint32_t adcMillivolts = 0;
    for (uint8_t sample = 0; sample < BatterySampleCount; ++sample)
    {
        adcMillivolts += analogReadMilliVolts(BoardPins::BatteryAdc);
        delay(2);
    }

    const float batteryVoltage =
        (static_cast<float>(adcMillivolts) / BatterySampleCount / 1000.0F) *
        BatteryDividerScale;

    PowerReadings readings = {};
    readings.batteryVoltageValid = batteryVoltage >= BatteryAbsentThreshold;
    if (readings.batteryVoltageValid)
    {
        readings.batteryVoltage = batteryVoltage;
        readings.estimatedChargePercent = estimateChargePercent(batteryVoltage);
    }
    return readings;
}
