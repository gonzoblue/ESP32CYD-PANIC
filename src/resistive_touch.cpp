#include "resistive_touch.h"

#include "board_pins.h"

void ResistiveTouch::begin()
{
    pinMode(BoardPins::TouchCs, OUTPUT);
    pinMode(BoardPins::TouchSck, OUTPUT);
    pinMode(BoardPins::TouchMosi, OUTPUT);
    pinMode(BoardPins::TouchMiso, INPUT);
    pinMode(BoardPins::TouchIrq, INPUT);

    digitalWrite(BoardPins::TouchCs, HIGH);
    digitalWrite(BoardPins::TouchSck, LOW);
    digitalWrite(BoardPins::TouchMosi, LOW);
}

bool ResistiveTouch::isPressed() const
{
    return digitalRead(BoardPins::TouchIrq) == LOW;
}

bool ResistiveTouch::read(RawTouchPoint &point) const
{
    if (!isPressed())
    {
        return false;
    }

    digitalWrite(BoardPins::TouchCs, LOW);
    point.x = readChannel(0xD0);
    point.y = readChannel(0x90);
    point.z1 = readChannel(0xB0);
    point.z2 = readChannel(0xC0);
    digitalWrite(BoardPins::TouchCs, HIGH);

    return point.z1 != 0 && point.z2 < 4095;
}

uint16_t ResistiveTouch::readChannel(uint8_t command) const
{
    writeByte(command);

    uint16_t value = 0;
    for (uint8_t bit = 0; bit < 12; ++bit)
    {
        digitalWrite(BoardPins::TouchSck, HIGH);
        value = static_cast<uint16_t>((value << 1) | digitalRead(BoardPins::TouchMiso));
        digitalWrite(BoardPins::TouchSck, LOW);
    }

    return value;
}

void ResistiveTouch::writeByte(uint8_t value) const
{
    for (int8_t bit = 7; bit >= 0; --bit)
    {
        digitalWrite(BoardPins::TouchMosi, (value >> bit) & 0x01);
        digitalWrite(BoardPins::TouchSck, HIGH);
        digitalWrite(BoardPins::TouchSck, LOW);
    }
}
