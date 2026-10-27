#pragma once

#include <Arduino.h>

namespace BoardPins
{
constexpr uint8_t TftBacklight = 21;
constexpr uint8_t TouchSck = 25;
constexpr uint8_t TouchMosi = 32;
constexpr uint8_t TouchMiso = 39;
constexpr uint8_t TouchCs = 33;
constexpr uint8_t TouchIrq = 36;

constexpr uint8_t BmeSck = 18;
constexpr uint8_t BmeMiso = 19;
constexpr uint8_t BmeMosi = 23;
constexpr uint8_t BmeCs = 27;
}
