#include <Arduino.h>
#include <TFT_eSPI.h>

#include "board_pins.h"
#include "resistive_touch.h"

namespace
{
constexpr uint8_t CalibrationPointCount = 4;
constexpr uint8_t CalibrationSamples = 16;
constexpr uint16_t CalibrationInset = 20;

struct CalibrationPoint
{
    double rawX;
    double rawY;
    double screenX;
    double screenY;
};

struct ScreenPoint
{
    int16_t x;
    int16_t y;
};

TFT_eSPI tft;
ResistiveTouch touch;
CalibrationPoint calibrationPoints[CalibrationPointCount];
double xCoefficients[3];
double yCoefficients[3];
int16_t previousMarkerX = -1;
int16_t previousMarkerY = -1;

bool solveSystem(const double matrix[3][3], const double values[3], double result[3])
{
    double augmented[3][4];
    for (uint8_t row = 0; row < 3; ++row)
    {
        for (uint8_t column = 0; column < 3; ++column)
        {
            augmented[row][column] = matrix[row][column];
        }
        augmented[row][3] = values[row];
    }

    for (uint8_t column = 0; column < 3; ++column)
    {
        uint8_t pivotRow = column;
        for (uint8_t row = column + 1; row < 3; ++row)
        {
            if (fabs(augmented[row][column]) > fabs(augmented[pivotRow][column]))
            {
                pivotRow = row;
            }
        }

        if (fabs(augmented[pivotRow][column]) < 1e-9)
        {
            return false;
        }

        if (pivotRow != column)
        {
            for (uint8_t item = column; item < 4; ++item)
            {
                const double temporary = augmented[column][item];
                augmented[column][item] = augmented[pivotRow][item];
                augmented[pivotRow][item] = temporary;
            }
        }

        const double divisor = augmented[column][column];
        for (uint8_t item = column; item < 4; ++item)
        {
            augmented[column][item] /= divisor;
        }

        for (uint8_t row = 0; row < 3; ++row)
        {
            if (row == column)
            {
                continue;
            }

            const double factor = augmented[row][column];
            for (uint8_t item = column; item < 4; ++item)
            {
                augmented[row][item] -= factor * augmented[column][item];
            }
        }
    }

    for (uint8_t row = 0; row < 3; ++row)
    {
        result[row] = augmented[row][3];
    }
    return true;
}

bool calculateCalibration()
{
    double normalMatrix[3][3] = {};
    double xValues[3] = {};
    double yValues[3] = {};

    for (const CalibrationPoint &point : calibrationPoints)
    {
        const double features[3] = {point.rawX, point.rawY, 1.0};
        for (uint8_t row = 0; row < 3; ++row)
        {
            xValues[row] += features[row] * point.screenX;
            yValues[row] += features[row] * point.screenY;
            for (uint8_t column = 0; column < 3; ++column)
            {
                normalMatrix[row][column] += features[row] * features[column];
            }
        }
    }

    return solveSystem(normalMatrix, xValues, xCoefficients) &&
           solveSystem(normalMatrix, yValues, yCoefficients);
}

ScreenPoint mapTouch(const RawTouchPoint &raw)
{
    const double mappedX = xCoefficients[0] * raw.x +
                           xCoefficients[1] * raw.y +
                           xCoefficients[2];
    const double mappedY = yCoefficients[0] * raw.x +
                           yCoefficients[1] * raw.y +
                           yCoefficients[2];

    ScreenPoint mapped;
    mapped.x = static_cast<int16_t>(constrain(lround(mappedX), 0L, static_cast<long>(tft.width() - 1)));
    mapped.y = static_cast<int16_t>(constrain(lround(mappedY), 0L, static_cast<long>(tft.height() - 1)));
    return mapped;
}

void drawCross(int16_t x, int16_t y, uint16_t color)
{
    tft.drawFastHLine(x - 9, y, 19, color);
    tft.drawFastVLine(x, y - 9, 19, color);
    tft.drawCircle(x, y, 4, color);
}

void drawCalibrationScreen(uint8_t pointIndex, int16_t targetX, int16_t targetY)
{
    tft.fillScreen(TFT_BLACK);
    tft.setTextDatum(TL_DATUM);
    tft.setTextColor(TFT_WHITE, TFT_BLACK);
    tft.drawString("Touch calibration", 8, 8, 2);
    tft.drawString("Tap the green target", 8, 32, 2);
    tft.setTextColor(TFT_CYAN, TFT_BLACK);
    tft.drawString(String("Point ") + String(pointIndex + 1) + " of 4", 8, 56, 2);
    drawCross(targetX, targetY, TFT_GREEN);
}

void waitForRelease()
{
    while (touch.isPressed())
    {
        delay(10);
    }
    delay(80);
}

bool captureCalibrationPoint(uint8_t pointIndex, int16_t targetX, int16_t targetY,
                             CalibrationPoint &calibrationPoint)
{
    drawCalibrationScreen(pointIndex, targetX, targetY);
    Serial.printf("Calibration point %u: tap target at screen (%d, %d).\n",
                  pointIndex + 1, targetX, targetY);

    uint32_t lastRawPrint = 0;
    RawTouchPoint raw;
    while (true)
    {
        if (touch.read(raw))
        {
            const uint32_t now = millis();
            if (now - lastRawPrint >= 100)
            {
                Serial.printf("Raw touch -> X: %u  Y: %u  Z1: %u  Z2: %u\n",
                              raw.x, raw.y, raw.z1, raw.z2);
                lastRawPrint = now;
            }
            break;
        }
        delay(10);
    }

    uint32_t sumX = 0;
    uint32_t sumY = 0;
    uint8_t samples = 0;
    while (samples < CalibrationSamples)
    {
        if (!touch.isPressed())
        {
            break;
        }
        if (touch.read(raw))
        {
            sumX += raw.x;
            sumY += raw.y;
            ++samples;
        }
        delay(4);
    }

    waitForRelease();
    if (samples < CalibrationSamples / 2)
    {
        Serial.println("Not enough stable samples; repeat this calibration point.");
        return false;
    }

    calibrationPoint = {
        static_cast<double>(sumX) / samples,
        static_cast<double>(sumY) / samples,
        static_cast<double>(targetX),
        static_cast<double>(targetY)};
    Serial.printf("Calibration raw average -> X: %.1f  Y: %.1f\n",
                  calibrationPoint.rawX, calibrationPoint.rawY);
    return true;
}

bool runCalibration()
{
    const int16_t left = CalibrationInset;
    const int16_t top = CalibrationInset;
    const int16_t right = tft.width() - 1 - CalibrationInset;
    const int16_t bottom = tft.height() - 1 - CalibrationInset;
    const int16_t targetX[CalibrationPointCount] = {left, right, right, left};
    const int16_t targetY[CalibrationPointCount] = {top, top, bottom, bottom};

    tft.fillScreen(TFT_BLACK);
    tft.setTextDatum(MC_DATUM);
    tft.setTextColor(TFT_WHITE, TFT_BLACK);
    tft.drawString("Touch calibration", tft.width() / 2, tft.height() / 2 - 12, 2);
    tft.drawString("Tap each green target", tft.width() / 2, tft.height() / 2 + 14, 2);
    delay(1200);
    waitForRelease();

    for (uint8_t point = 0; point < CalibrationPointCount; ++point)
    {
        while (!captureCalibrationPoint(point, targetX[point], targetY[point],
                                        calibrationPoints[point]))
        {
        }
    }

    if (!calculateCalibration())
    {
        Serial.println("Touch calibration failed: points do not define a usable mapping.");
        tft.fillScreen(TFT_BLACK);
        tft.setTextColor(TFT_RED, TFT_BLACK);
        tft.drawString("Calibration failed", tft.width() / 2, tft.height() / 2, 2);
        return false;
    }

    Serial.println("Touch calibration complete.");
    tft.fillScreen(TFT_BLACK);
    tft.setTextColor(TFT_GREEN, TFT_BLACK);
    tft.setTextDatum(MC_DATUM);
    tft.drawString("Calibration complete", tft.width() / 2, tft.height() / 2, 2);
    delay(900);
    return true;
}

void drawRuntimeHeader(int16_t x, int16_t y, const RawTouchPoint &raw)
{
    tft.fillRect(0, 0, tft.width(), 54, TFT_BLACK);
    tft.setTextDatum(TL_DATUM);
    tft.setTextColor(TFT_GREEN, TFT_BLACK);
    tft.drawString("ESP32CYD-PANIC  Touch", 8, 4, 2);
    tft.setTextColor(TFT_WHITE, TFT_BLACK);
    tft.drawString(String("Screen X: ") + String(x) + "  Y: " + String(y), 8, 26, 2);
    tft.setTextColor(TFT_CYAN, TFT_BLACK);
    tft.drawString(String("Raw X: ") + String(raw.x) + "  Y: " + String(raw.y), 8, 44, 1);
}

void runTouchDisplay()
{
    RawTouchPoint raw;
    static uint32_t lastSerialPrint = 0;
    static uint32_t lastDraw = 0;

    if (!touch.read(raw))
    {
        return;
    }

    const ScreenPoint mapped = mapTouch(raw);
    const uint32_t now = millis();
    if (now - lastSerialPrint >= 100)
    {
        Serial.printf("Touch -> Raw X: %u  Y: %u  Pressure: %u; Screen X: %d  Y: %d\n",
                      raw.x, raw.y, raw.z1 + 4095 - raw.z2, mapped.x, mapped.y);
        lastSerialPrint = now;
    }

    if (now - lastDraw >= 35)
    {
        if (previousMarkerX >= 0)
        {
            tft.fillCircle(previousMarkerX, previousMarkerY, 5, TFT_BLACK);
        }
        drawRuntimeHeader(mapped.x, mapped.y, raw);
        tft.fillCircle(mapped.x, mapped.y, 5, TFT_YELLOW);
        tft.drawCircle(mapped.x, mapped.y, 8, TFT_RED);
        previousMarkerX = mapped.x;
        previousMarkerY = mapped.y;
        lastDraw = now;
    }
}
}

void setup()
{
    Serial.begin(115200);
    delay(500);
    Serial.println();
    Serial.println("ESP32CYD-PANIC");
    Serial.println("Starting E32R28T display and touchscreen test...");

    pinMode(BoardPins::TftBacklight, OUTPUT);
    digitalWrite(BoardPins::TftBacklight, HIGH);

    tft.init();
    tft.setRotation(1);
    tft.fillScreen(TFT_BLACK);

    touch.begin();
    Serial.println("Touch SPI initialized on GPIO25/32/39/33.");

    while (!runCalibration())
    {
        delay(1000);
    }

    tft.fillScreen(TFT_BLACK);
    drawRuntimeHeader(0, 0, {0, 0, 0, 0});
    Serial.println("Touch the screen to test calibrated coordinates.");
}

void loop()
{
    runTouchDisplay();
    delay(8);
}
