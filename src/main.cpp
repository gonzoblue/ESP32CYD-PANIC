#include <Arduino.h>
#include <Preferences.h>
#include <TFT_eSPI.h>
#include <esp_system.h>
#include <esp_wifi.h>
#include <esp_bt.h>
#include <math.h>

#include "bme_sensor.h"
#include "backlight_policy.h"
#include "board_pins.h"
#include "gps_sensor.h"
#include "power_monitor.h"
#include "resistive_touch.h"

namespace
{
constexpr uint8_t CalibrationPointCount = 4;
constexpr uint8_t CalibrationSamples = 16;
constexpr uint16_t CalibrationInset = 20;
constexpr int16_t CalibrationButtonWidth = 38;
constexpr int16_t CalibrationButtonHeight = 24;
constexpr int16_t CalibrationButtonTop = 18;
constexpr int16_t CalibrationButtonRightInset = 8;
constexpr int16_t PageButtonWidth = 50;
constexpr int16_t PageButtonHeight = 22;
constexpr int16_t PageButtonRightInset = 8;
constexpr int16_t PageButtonBottomInset = 8;
constexpr uint32_t CalibrationStorageMagic = 0x5443414CUL;
constexpr uint32_t CalibrationStorageVersion = 1;
constexpr uint32_t CalibrationResultDurationMs = 15000;
constexpr uint32_t SystemStatsRefreshMs = 1000;

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

struct StoredTouchCalibration
{
    uint32_t magic;
    uint32_t version;
    double xCoefficients[3];
    double yCoefficients[3];
};

TFT_eSPI tft;
ResistiveTouch touch;
BmeSensor bme;
GpsSensor gps;
PowerMonitor powerMonitor;
CalibrationPoint calibrationPoints[CalibrationPointCount];
double xCoefficients[3];
double yCoefficients[3];
int16_t previousMarkerX = -1;
int16_t previousMarkerY = -1;
BmeReadings latestBmeReadings = {};
bool bmeAvailable = false;
uint32_t lastBmeRead = 0;
uint32_t lastGpsDiagnostics = 0;
uint32_t lastPowerDiagnostics = 0;
uint32_t lastSystemStatsDraw = 0;
char latestGpsNmea[128] = {};
bool systemPageActive = false;
bool backlightOn = true;
bool consumingWakeTouch = false;
uint32_t lastTouchTime = 0;
uint32_t lastDarkRefresh = 0;
PowerReadings latestPowerReadings = {};
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

bool isMappingValid(const double mappingX[3], const double mappingY[3])
{
    for (uint8_t index = 0; index < 3; ++index)
    {
        if (!isfinite(mappingX[index]) || !isfinite(mappingY[index]))
        {
            return false;
        }
    }

    const double determinant =
        mappingX[0] * mappingY[1] - mappingX[1] * mappingY[0];
    if (!isfinite(determinant) || fabs(determinant) < 1e-8)
    {
        return false;
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

    double calculatedX[3];
    double calculatedY[3];
    if (!solveSystem(normalMatrix, xValues, calculatedX) ||
        !solveSystem(normalMatrix, yValues, calculatedY))
    {
        return false;
    }

    if (!isMappingValid(calculatedX, calculatedY))
    {
        return false;
    }

    for (uint8_t index = 0; index < 3; ++index)
    {
        xCoefficients[index] = calculatedX[index];
        yCoefficients[index] = calculatedY[index];
    }
    return true;
}

bool isCalibrationValid(const StoredTouchCalibration &stored)
{
    if (stored.magic != CalibrationStorageMagic ||
        stored.version != CalibrationStorageVersion)
    {
        return false;
    }

    return isMappingValid(stored.xCoefficients, stored.yCoefficients);
}

bool loadCalibration()
{
    Preferences preferences;
    if (!preferences.begin("touchcal", true))
    {
        Serial.println("Touch calibration NVS open failed; running calibration.");
        return false;
    }

    StoredTouchCalibration stored = {};
    const size_t bytesRead =
        preferences.getBytes("mapping", &stored, sizeof(stored));
    preferences.end();

    if (bytesRead != sizeof(stored) || !isCalibrationValid(stored))
    {
        Serial.println("No valid saved touch calibration; running calibration.");
        return false;
    }

    for (uint8_t index = 0; index < 3; ++index)
    {
        xCoefficients[index] = stored.xCoefficients[index];
        yCoefficients[index] = stored.yCoefficients[index];
    }
    Serial.println("Loaded touch calibration from NVS.");
    return true;
}

bool saveCalibration()
{
    StoredTouchCalibration stored = {};
    stored.magic = CalibrationStorageMagic;
    stored.version = CalibrationStorageVersion;
    for (uint8_t index = 0; index < 3; ++index)
    {
        stored.xCoefficients[index] = xCoefficients[index];
        stored.yCoefficients[index] = yCoefficients[index];
    }

    Preferences preferences;
    if (!preferences.begin("touchcal", false))
    {
        Serial.println("Touch calibration NVS open failed; calibration is active but not saved.");
        return false;
    }

    const size_t bytesWritten =
        preferences.putBytes("mapping", &stored, sizeof(stored));
    preferences.end();
    if (bytesWritten != sizeof(stored))
    {
        Serial.println("Touch calibration NVS write failed; calibration is active but not saved.");
        return false;
    }

    Serial.println("Saved touch calibration to NVS.");
    return true;
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
    tft.setTextDatum(MC_DATUM);
    tft.setTextColor(TFT_WHITE, TFT_BLACK);
    tft.drawString("Touch calibration", tft.width() / 2, 76, 2);
    tft.drawString("Tap the green target", tft.width() / 2, 96, 2);
    tft.setTextColor(TFT_CYAN, TFT_BLACK);
    tft.drawString(String("Point ") + String(pointIndex + 1) + " of 4",
                   tft.width() / 2, 112, 2);

    tft.setTextDatum(TL_DATUM);
    tft.setTextColor(TFT_WHITE, TFT_BLACK);
    for (uint8_t index = 0; index < pointIndex; ++index)
    {
        const CalibrationPoint &point = calibrationPoints[index];
        const String reading =
            String("P") + String(index + 1) + " X:" +
            String(point.rawX, 0) + " Y:" + String(point.rawY, 0) +
            " -> " + String(point.screenX, 0) + "," +
            String(point.screenY, 0);
        tft.drawString(reading, 8, 132 + index * 16, 1);
    }

    drawCross(targetX, targetY, TFT_GREEN);
}

void showCalibrationResult(const char *heading, const char *details,
                           uint16_t headingColor)
{
    tft.fillScreen(TFT_BLACK);
    tft.setTextDatum(MC_DATUM);
    tft.setTextColor(headingColor, TFT_BLACK);
    tft.drawString(heading, tft.width() / 2, 34, 2);
    tft.setTextColor(TFT_WHITE, TFT_BLACK);
    tft.drawString(details, tft.width() / 2, 58, 1);

    tft.setTextDatum(TL_DATUM);
    for (uint8_t index = 0; index < CalibrationPointCount; ++index)
    {
        const CalibrationPoint &point = calibrationPoints[index];
        const String reading =
            String("P") + String(index + 1) + " X:" +
            String(point.rawX, 0) + " Y:" + String(point.rawY, 0) +
            " -> " + String(point.screenX, 0) + "," +
            String(point.screenY, 0);
        tft.drawString(reading, 8, 84 + index * 20, 1);
    }

    delay(CalibrationResultDurationMs);
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

    for (uint8_t point = 0; point < CalibrationPointCount; ++point)
    {
        calibrationPoints[point] = {};
    }

    tft.fillScreen(TFT_BLACK);
    tft.setTextDatum(MC_DATUM);
    tft.setTextColor(TFT_WHITE, TFT_BLACK);
    tft.drawString("Touch calibration", tft.width() / 2, 82, 2);
    tft.drawString("Tap each green target", tft.width() / 2, 108, 2);
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
        showCalibrationResult("CALIBRATION FAILED", "Invalid touch mapping",
                              TFT_RED);
        return false;
    }

    Serial.println("Touch calibration complete.");
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
    const int16_t buttonX =
        tft.width() - CalibrationButtonRightInset - CalibrationButtonWidth;
    const uint8_t coordinateFont =
        tft.textWidth(touchCoordinates, 2) <= buttonX - 16 ? 2 : 1;
    tft.drawString(touchCoordinates, 8, 20, coordinateFont);

    tft.fillRoundRect(buttonX, CalibrationButtonTop, CalibrationButtonWidth,
                      CalibrationButtonHeight, 4, TFT_DARKGREY);
    tft.setTextColor(TFT_WHITE, TFT_DARKGREY);
    tft.drawCentreString("CAL", buttonX + CalibrationButtonWidth / 2,
                         CalibrationButtonTop + 6, 1);
}

int16_t pageButtonX()
{
    return tft.width() - PageButtonRightInset - PageButtonWidth;
}

int16_t pageButtonY()
{
    return tft.height() - PageButtonBottomInset - PageButtonHeight;
}

void drawPageButton(const char *label)
{
    const int16_t buttonX = pageButtonX();
    const int16_t buttonY = pageButtonY();
    tft.fillRoundRect(buttonX, buttonY, PageButtonWidth, PageButtonHeight,
                      4, TFT_DARKGREY);
    tft.setTextDatum(MC_DATUM);
    tft.setTextColor(TFT_WHITE, TFT_DARKGREY);
    tft.drawString(label, buttonX + PageButtonWidth / 2,
                   buttonY + PageButtonHeight / 2, 1);
}

bool isPageButtonTouch(const ScreenPoint &point)
{
    return point.x >= pageButtonX() &&
           point.x < pageButtonX() + PageButtonWidth &&
           point.y >= pageButtonY() &&
           point.y < pageButtonY() + PageButtonHeight;
}

const char *resetReasonName(esp_reset_reason_t reason)
{
    switch (reason)
    {
    case ESP_RST_POWERON:
        return "Power on";
    case ESP_RST_EXT:
        return "External reset";
    case ESP_RST_SW:
        return "Software reset";
    case ESP_RST_PANIC:
        return "Exception/panic";
    case ESP_RST_INT_WDT:
        return "Interrupt watchdog";
    case ESP_RST_TASK_WDT:
        return "Task watchdog";
    case ESP_RST_WDT:
        return "Watchdog";
    case ESP_RST_DEEPSLEEP:
        return "Deep sleep wake";
    case ESP_RST_BROWNOUT:
        return "Brownout";
    case ESP_RST_SDIO:
        return "SDIO reset";
    case ESP_RST_USB:
        return "USB reset";
    case ESP_RST_JTAG:
        return "JTAG reset";
    case ESP_RST_EFUSE:
        return "eFuse error";
    case ESP_RST_PWR_GLITCH:
        return "Power glitch";
    case ESP_RST_CPU_LOCKUP:
        return "CPU lockup";
    case ESP_RST_UNKNOWN:
    default:
        return "Unknown";
    }
}

void drawSystemDynamicStats()
{
    tft.fillRect(0, 88, tft.width(), 104, TFT_BLACK);
    tft.setTextDatum(TL_DATUM);
    tft.setTextColor(TFT_WHITE, TFT_BLACK);
    tft.drawString(String("Heap free/min: ") +
                       String(static_cast<unsigned long>(ESP.getFreeHeap() / 1024)) +
                       "/" +
                       String(static_cast<unsigned long>(ESP.getMinFreeHeap() / 1024)) +
                       " KB",
                   8, 90, 1);

    const uint32_t uptimeSeconds = millis() / 1000;
    const uint32_t days = uptimeSeconds / 86400;
    const uint32_t hours = (uptimeSeconds / 3600) % 24;
    const uint32_t minutes = (uptimeSeconds / 60) % 60;
    const uint32_t seconds = uptimeSeconds % 60;
    char uptime[32];
    snprintf(uptime, sizeof(uptime), "Uptime: %lu d %02lu:%02lu:%02lu",
             static_cast<unsigned long>(days),
             static_cast<unsigned long>(hours),
             static_cast<unsigned long>(minutes),
             static_cast<unsigned long>(seconds));
    tft.drawString(uptime, 8, 106, 1);

    wifi_mode_t wifiMode = WIFI_MODE_NULL;
    const esp_err_t wifiResult = esp_wifi_get_mode(&wifiMode);
    const char *wifiStatus = "ERROR";
    if (wifiResult == ESP_ERR_WIFI_NOT_INIT)
    {
        wifiStatus = "OFF (not initialized)";
    }
    else if (wifiResult == ESP_OK)
    {
        wifiStatus = wifiMode == WIFI_MODE_NULL ? "OFF" : "configured";
    }
    else
    {
        Serial.printf("Wi-Fi state query failed: %s\n", esp_err_to_name(wifiResult));
    }
    tft.drawString(String("Wi-Fi: ") + wifiStatus, 8, 122, 1);
    const esp_bt_controller_status_t bluetoothStatus = esp_bt_controller_get_status();
    tft.drawString(String("BT: ") +
                       (bluetoothStatus == ESP_BT_CONTROLLER_STATUS_ENABLED
                            ? "ON"
                            : bluetoothStatus == ESP_BT_CONTROLLER_STATUS_IDLE
                                  ? "OFF (not initialized)"
                                  : "OFF (initialized)"),
                   8, 138, 1);
    tft.drawString(String("ILI9341 320x240 BL: ") + (backlightOn ? "ON" : "OFF"),
                   8, 154, 1);
    tft.drawString(String("Saver: 60s idle <90%; dark refresh 5s"),
                   8, 170, 1);
    if (latestPowerReadings.batteryVoltageValid)
    {
        tft.drawString(String("Power: ") + String(latestPowerReadings.batteryVoltage, 2) +
                           " V (" + String(latestPowerReadings.estimatedChargePercent) +
                           "% est.)",
                       8, 184, 1);
    }
}

void drawSystemDiagnosticsScreen()
{
    tft.fillScreen(TFT_BLACK);
    tft.setTextDatum(TL_DATUM);
    tft.setTextColor(TFT_CYAN, TFT_BLACK);
    tft.drawString("ESP32 SYSTEM", 8, 8, 2);

    tft.setTextColor(TFT_WHITE, TFT_BLACK);
    tft.drawString(String(ESP.getChipModel()) + " rev " +
                       String(static_cast<unsigned>(ESP.getChipRevision())) +
                       " / " + String(static_cast<unsigned>(ESP.getChipCores())) +
                       " cores",
                   8, 36, 1);
    tft.drawString(String("CPU: ") +
                       String(static_cast<unsigned long>(ESP.getCpuFreqMHz())) +
                       " MHz / Flash: " +
                       String(static_cast<unsigned long>(ESP.getFlashChipSize() / (1024UL * 1024UL))) +
                       " MB",
                   8, 54, 1);

    const uint32_t psramSize = ESP.getPsramSize();
    String psram = "PSRAM: ";
    if (psramSize == 0)
    {
        psram += "not detected";
    }
    else
    {
        psram += String(static_cast<unsigned long>(psramSize / 1024)) +
                 " KB total, " +
                 String(static_cast<unsigned long>(ESP.getFreePsram() / 1024)) +
                 " KB free";
    }
    tft.drawString(psram, 8, 72, 1);

    drawSystemDynamicStats();
    tft.drawString(String("Reset: ") + resetReasonName(esp_reset_reason()),
                   8, 198, 1);
    tft.drawString(String("ESP-IDF: ") + ESP.getSdkVersion(),
                   8, 214, 1);
    drawPageButton("APP");
    lastSystemStatsDraw = millis();
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

    if (!systemPageActive && backlightOn)
    {
        drawBmeDiagnostics();
    }
}

void drawGpsDiagnostics(const GpsReadings &readings)
{
    tft.fillRect(0, 156, tft.width(), 76, TFT_BLACK);
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
    tft.drawString(String(latestGpsNmea).substring(0, 40), 8, 223, 1);
    drawPageButton("SYS");
}

void drawPowerDiagnostics(const PowerReadings &readings)
{
    tft.fillRect(0, 232, tft.width(), 8, TFT_BLACK);
    if (!readings.batteryVoltageValid)
    {
        return;
    }

    tft.setTextDatum(TL_DATUM);
    tft.setTextColor(TFT_YELLOW, TFT_BLACK);
    tft.drawString(
        String("Power ") + String(readings.batteryVoltage, 2) + " V (" +
            String(readings.estimatedChargePercent) + "%)",
        8, 232, 1);
}

void updatePowerDiagnostics()
{
    const uint32_t now = millis();
    if (now - lastPowerDiagnostics < 2000)
    {
        return;
    }
    lastPowerDiagnostics = now;

    const PowerReadings readings = powerMonitor.read();
    latestPowerReadings = readings;
    if (!systemPageActive && backlightOn)
    {
        drawPowerDiagnostics(readings);
    }
    if (readings.batteryVoltageValid)
    {
        Serial.printf("Power -> %.2f V (estimated charge %u%%)\n",
                      readings.batteryVoltage,
                      static_cast<unsigned>(readings.estimatedChargePercent));
    }
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
    if (!systemPageActive && backlightOn)
    {
        drawGpsDiagnostics(readings);
    }
}

void drawDiagnosticsScreen()
{
    systemPageActive = false;
    tft.fillScreen(TFT_BLACK);
    drawRuntimeHeader(0, 0, {0, 0, 0, 0});
    drawBmeDiagnostics();
    GpsReadings gpsReadings = {};
    gps.read(gpsReadings);
    drawGpsDiagnostics(gpsReadings);
    drawPowerDiagnostics(latestPowerReadings);
}

void updateSystemDiagnostics()
{
    if (!systemPageActive || !backlightOn)
    {
        return;
    }

    const uint32_t now = millis();
    if (now - lastSystemStatsDraw < SystemStatsRefreshMs)
    {
        return;
    }

    drawSystemDynamicStats();
    lastSystemStatsDraw = now;
}

void redrawActivePage()
{
    if (systemPageActive)
    {
        drawSystemDiagnosticsScreen();
    }
    else
    {
        drawDiagnosticsScreen();
    }
    previousMarkerX = -1;
    previousMarkerY = -1;
}

void updateBacklight()
{
    const uint32_t now = millis();
    const bool blank = BacklightPolicy::shouldBlank(
        latestPowerReadings.batteryVoltageValid,
        latestPowerReadings.estimatedChargePercent,
        now - lastTouchTime);
    if (backlightOn && blank)
    {
        backlightOn = false;
        digitalWrite(BoardPins::TftBacklight, LOW);
        lastDarkRefresh = now;
    }
    else if (!backlightOn && !blank)
    {
        backlightOn = true;
        redrawActivePage();
        digitalWrite(BoardPins::TftBacklight, HIGH);
    }

    if (!backlightOn && BacklightPolicy::refreshDue(now, lastDarkRefresh))
    {
        redrawActivePage();
        lastDarkRefresh = now;
    }
}

bool isCalibrationButtonTouch(const ScreenPoint &point)
{
    const int16_t buttonX =
        tft.width() - CalibrationButtonRightInset - CalibrationButtonWidth;
    return point.x >= buttonX &&
           point.x < buttonX + CalibrationButtonWidth &&
           point.y >= CalibrationButtonTop &&
           point.y < CalibrationButtonTop + CalibrationButtonHeight;
}

void runTouchDisplay()
{
    RawTouchPoint raw;
    static uint32_t lastSerialPrint = 0;
    static uint32_t lastDraw = 0;

    if (consumingWakeTouch)
    {
        lastTouchTime = millis();
        if (!touch.isPressed())
        {
            consumingWakeTouch = false;
        }
        return;
    }

    if (!backlightOn && touch.isPressed())
    {
        lastTouchTime = millis();
        consumingWakeTouch = true;
        backlightOn = true;
        redrawActivePage();
        digitalWrite(BoardPins::TftBacklight, HIGH);
        return;
    }

    if (touch.isPressed())
    {
        lastTouchTime = millis();
    }

    if (!touch.read(raw))
    {
        return;
    }

    lastTouchTime = millis();
    const ScreenPoint mapped = mapTouch(raw);
    if (systemPageActive)
    {
        if (isPageButtonTouch(mapped))
        {
            waitForRelease();
            lastTouchTime = millis();
            drawDiagnosticsScreen();
            previousMarkerX = -1;
            previousMarkerY = -1;
        }
        return;
    }

    if (isPageButtonTouch(mapped))
    {
        waitForRelease();
        lastTouchTime = millis();
        systemPageActive = true;
        drawSystemDiagnosticsScreen();
        previousMarkerX = -1;
        previousMarkerY = -1;
        return;
    }

    if (isCalibrationButtonTouch(mapped))
    {
        Serial.println("Touch calibration requested from CAL button.");
        waitForRelease();
        if (runCalibration())
        {
            if (saveCalibration())
            {
                showCalibrationResult("CALIBRATION SAVED", "Saved to NVS",
                                      TFT_GREEN);
            }
            else
            {
                showCalibrationResult("CALIBRATION NOT SAVED",
                                      "Active, but NVS write failed",
                                      TFT_YELLOW);
            }
        }
        else
        {
            Serial.println("Recalibration failed; retaining the previous mapping.");
        }
        drawDiagnosticsScreen();
        lastTouchTime = millis();
        previousMarkerX = -1;
        previousMarkerY = -1;
        return;
    }

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

    powerMonitor.begin();
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

    if (!loadCalibration())
    {
        while (!runCalibration())
        {
            delay(1000);
        }
        if (saveCalibration())
        {
            showCalibrationResult("CALIBRATION SAVED", "Saved to NVS",
                                  TFT_GREEN);
        }
        else
        {
            showCalibrationResult("CALIBRATION NOT SAVED",
                                  "Active, but NVS write failed",
                                  TFT_YELLOW);
        }
    }

    latestPowerReadings = powerMonitor.read();
    lastTouchTime = millis();
    drawDiagnosticsScreen();
    Serial.println("Touch the screen to test calibrated coordinates.");
}

void loop()
{
    updateBmeDiagnostics();
    updateGpsDiagnostics();
    updatePowerDiagnostics();
    runTouchDisplay();
    updateBacklight();
    updateSystemDiagnostics();
    delay(8);
}
