#pragma once

#include <Arduino.h>

struct RawTouchPoint
{
    uint16_t x;
    uint16_t y;
    uint16_t z1;
    uint16_t z2;
};

class ResistiveTouch
{
public:
    void begin();
    bool isPressed() const;
    bool read(RawTouchPoint &point) const;

private:
    uint16_t readChannel(uint8_t command) const;
    void writeByte(uint8_t value) const;
};
