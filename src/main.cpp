#include <Arduino.h>
#include <TFT_eSPI.h>

#include "bme_sensor.h"
#include "board_pins.h"
#include "gps_sensor.h"
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
BmeSensor bme;
GpsSensor gps;
CalibrationPoint calibrationPoints[CalibrationPointCount];
double xCoefficients[3];
double yCoefficients[3];
int16_t previousMarkerX = -1;
int16_t previousMarkerY = -1;
BmeReadings latestBmeReadings = {};
bool bmeAvailable = false;
uint32_t lastBmeRead = 0;
uint32_t lastGpsDiagnostics = 0;
char latestGpsNmea[128] = {};
constexpr bool EchoGpsNmeaToSerial = true;

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
    struct TitleSegment
    {
        const char *text;
        uint16_t color;
    };
    const TitleSegment segments[] = {
        {"P", TFT_GREEN},
        {"ortable ", TFT_WHITE},
        {"A", TFT_GREEN},
        {"ltitude, ", TFT_WHITE},
        {"N", TFT_GREEN},
        {"avigation, and ", TFT_WHITE},
        {"I", TFT_GREEN},
        {"n-situ ", TFT_WHITE},
        {"C", TFT_GREEN},
        {"onditions", TFT_WHITE}};

    int16_t titleX = 1;
    for (const TitleSegment &segment : segments)
    {
        tft.setTextColor(segment.color, TFT_BLACK);
        tft.drawString(segment.text, titleX, 4, 1);
        titleX += tft.textWidth(segment.text, 1);
    }

    tft.setTextColor(TFT_CYAN, TFT_BLACK);
    const String touchCoordinates =
        String("X: ") + String(x) + " (" + String(raw.x) + ") Y: " +
        String(y) + " (" + String(raw.y) + ")";
    const uint8_t coordinateFont =
        tft.textWidth(touchCoordinates, 2) <= tft.width() - 16 ? 2 : 1;
    tft.drawString(touchCoordinates, 8, 20, coordinateFont);
}

void drawBmeDiagnostics()
{
    tft.fillRect(0, 64, tft.width(), 78, TFT_BLACK);
    tft.setTextDatum(TL_DATUM);
    tft.setTextColor(bmeAvailable ? TFT_GREEN : TFT_RED, TFT_BLACK);
    tft.drawString(bmeAvailable ? "BME280  OK" : "BME280  ERROR", 8, 66, 2);

    if (!bmeAvailable)
    {
        return;
    }

    const float temperatureF = latestBmeReadings.temperatureC * 1.8F + 32.0F;
    const float altitudeFeet = latestBmeReadings.altitudeMeters * 3.28084F;
    tft.setTextColor(TFT_WHITE, TFT_BLACK);
    tft.drawString(
        String("Temp ") + String(latestBmeReadings.temperatureC, 1) + " C / " +
            String(temperatureF, 1) + " F  Hum " +
            String(latestBmeReadings.humidityPercent, 1) + "%",
        8, 90, 2);
    tft.drawString(
        String("Pressure ") + String(latestBmeReadings.pressureHpa, 1) + " hPa",
        8, 114, 2);
    tft.drawString(
        String("Baro Alt ") + String(latestBmeReadings.altitudeMeters, 1) + " m / " +
            String(altitudeFeet, 0) + " ft",
        8, 136, 2);
}

void updateBmeDiagnostics()
{
    const uint32_t now = millis();
    if (now - lastBmeRead < 1000)
    {
        return;
    }
    lastBmeRead = now;

    BmeReadings readings;
    bmeAvailable = bme.read(readings);
    if (bmeAvailable)
    {
        latestBmeReadings = readings;
        Serial.printf(
            "BME280 -> Temperature: %.2f C  Humidity: %.2f %%  Pressure: %.2f hPa  Barometric altitude: %.2f m\n",
            readings.temperatureC, readings.humidityPercent,
            readings.pressureHpa, readings.altitudeMeters);
    }
    else
    {
        Serial.println("BME280 read failed; no replacement values are being reported.");
    }

    drawBmeDiagnostics();
}

void drawGpsDiagnostics(const GpsReadings &readings)
{
    tft.fillRect(0, 156, tft.width(), 84, TFT_BLACK);
    tft.setTextDatum(TL_DATUM);
    tft.setTextColor(readings.hasFix ? TFT_GREEN : TFT_YELLOW, TFT_BLACK);
    tft.drawString(readings.hasFix ? "GPS  FIX" :
                   (readings.hasNmeaData ? "GPS  NO FIX" : "GPS  NO DATA"),
                   8, 158, 2);

    tft.setTextColor(TFT_WHITE, TFT_BLACK);
    if (readings.hasFix && readings.hasLatitudeLongitude)
    {
        tft.drawString(
            String("Lat ") + String(readings.latitude, 6) +
                "  Lon " + String(readings.longitude, 6),
            8, 181, 1);
        String motion = "Alt ";
        motion += readings.hasAltitude
                      ? String(readings.altitudeMeters, 1) + " m"
                      : String("--");
        motion += "  Speed ";
        motion += readings.hasSpeed
                      ? String(readings.speedKmph * 0.621371F, 1) + " mph"
                      : String("--");
        tft.drawString(motion, 8, 195, 1);
    }
    else
    {
        tft.drawString("Position unavailable", 8, 181, 1);
    }

    String status = "Satellites ";
    status += readings.hasSatellites
                  ? String(readings.satellites)
                  : String("--");
    status += "  Fix quality ";
    status += readings.hasFixQuality
                  ? String(readings.fixQuality)
                  : String("--");
    tft.drawString(status, 8, 209, 1);

    tft.setTextColor(TFT_CYAN, TFT_BLACK);
    tft.drawString(String(latestGpsNmea).substring(0, 52), 8, 223, 1);
}

void printGpsDiagnostics(const GpsReadings &readings)
{
    if (!readings.hasNmeaData)
    {
        Serial.println("GPS -> NO DATA");
        return;
    }
    if (!readings.hasFix)
    {
        Serial.print("GPS -> NO FIX");
    }
    else
    {
        Serial.print("GPS -> FIX");
    }

    if (readings.hasLatitudeLongitude)
    {
        Serial.printf("  Latitude: %.6f  Longitude: %.6f",
                      readings.latitude, readings.longitude);
    }
    if (readings.hasAltitude)
    {
        Serial.printf("  Altitude: %.1f m", readings.altitudeMeters);
    }
    if (readings.hasSpeed)
    {
        Serial.printf("  Speed: %.2f km/h", readings.speedKmph);
    }
    if (readings.hasCourse)
    {
        Serial.printf("  Course: %.1f deg", readings.courseDegrees);
    }
    if (readings.hasSatellites)
    {
        Serial.printf("  Satellites: %lu",
                      static_cast<unsigned long>(readings.satellites));
    }
    if (readings.hasFixQuality)
    {
        Serial.printf("  Fix quality: %u", readings.fixQuality);
    }
    if (readings.hasTime)
    {
        Serial.printf("  UTC: %02u:%02u:%02u",
                      static_cast<unsigned>(readings.hour),
                      static_cast<unsigned>(readings.minute),
                      static_cast<unsigned>(readings.second));
    }
    Serial.println();
}

void updateGpsDiagnostics()
{
    gps.update();

    char sentence[sizeof(latestGpsNmea)];
    if (gps.takeNmeaSentence(sentence, sizeof(sentence)))
    {
        snprintf(latestGpsNmea, sizeof(latestGpsNmea), "%s", sentence);
        if (EchoGpsNmeaToSerial)
        {
            Serial.printf("GPS NMEA -> %s\n", sentence);
        }
    }

    const uint32_t now = millis();
    if (now - lastGpsDiagnostics < 1000)
    {
        return;
    }
    lastGpsDiagnostics = now;

    GpsReadings readings;
    gps.read(readings);
    if (!readings.hasNmeaData)
    {
        latestGpsNmea[0] = '\0';
    }
    printGpsDiagnostics(readings);
    drawGpsDiagnostics(readings);
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

    if (now - lastDraw >= 250)
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
    Serial.println("Portable Altitude, Navigation, and In-situ Conditions");
    Serial.println("Starting E32R28T display and touchscreen test...");

    pinMode(BoardPins::TftBacklight, OUTPUT);
    digitalWrite(BoardPins::TftBacklight, HIGH);

    tft.init();
    tft.setRotation(1);
    tft.fillScreen(TFT_BLACK);

    touch.begin();
    Serial.println("Touch SPI initialized on GPIO25/32/39/33.");

    bmeAvailable = bme.begin();
    if (bmeAvailable)
    {
        Serial.println("BME280 initialized over SPI on GPIO18/19/23 with CS GPIO27.");
    }
    else
    {
        Serial.println("BME280 initialization failed; check SPI wiring, power, and sensor identity.");
    }

    gps.begin();
    Serial.println("GPS UART1 receive-only on exposed GPIO3/RX0 at 9600 baud; leave GPS RX disconnected.");
    Serial.println("GPIO3 is shared with the USB serial adapter; disconnect GPS TX if flashing or diagnostics are affected.");

    while (!runCalibration())
    {
        delay(1000);
    }

    tft.fillScreen(TFT_BLACK);
    drawRuntimeHeader(0, 0, {0, 0, 0, 0});
    drawBmeDiagnostics();
    GpsReadings gpsReadings = {};
    drawGpsDiagnostics(gpsReadings);
    Serial.println("Touch the screen to test calibrated coordinates.");
}

void loop()
{
    updateBmeDiagnostics();
    updateGpsDiagnostics();
    runTouchDisplay();
    delay(8);
}
