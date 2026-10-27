#pragma once

#include <stdint.h>

namespace BacklightPolicy
{
constexpr uint32_t IdleTimeoutMs = 60000;
constexpr uint32_t DarkRefreshMs = 5000;

constexpr bool shouldBlank(bool valid, uint8_t percent, uint32_t idleMs)
{
    return valid && percent < 90 && idleMs >= IdleTimeoutMs;
}

constexpr bool refreshDue(uint32_t now, uint32_t previous)
{
    return static_cast<uint32_t>(now - previous) >= DarkRefreshMs;
}
}
