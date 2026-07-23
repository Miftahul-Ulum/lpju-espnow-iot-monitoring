/*
=========================================================
PROJECT     : ESP32 PJU Gateway Controller
PLATFORM    : ESP32
DESCRIPTION :
- Handle RTC scheduling using DS3231
- Synchronize RTC using NTP
- Monitor slave device status
- Display monitoring information on LCD
- Receive slave ACK data from ESP01 Master
- Read GPS coordinates from UART GPS module
- Read Electric Current With PZEM module
- Monitoring integration with HTTP API
- Scheduling State ON/OFF on slave
=========================================================
*/

#include <WiFi.h>
#include <WiFiManager.h>

#include <Wire.h>
#include <RTClib.h>

#include <TinyGPS++.h>

#include <LiquidCrystal_I2C.h>

#include <time.h>

#include <HTTPClient.h>
#include <ArduinoJson.h>

#include <Preferences.h>

#include <PZEM004Tv30.h>

// =========================================================
// SERIAL COMMUNICATION
// =========================================================

HardwareSerial masterSerial(2);
HardwareSerial sensorSerial(1);

// =========================================================
// DEVICE INSTANCES
// =========================================================

RTC_DS3231 rtc;
TinyGPSPlus gps;
LiquidCrystal_I2C lcd(0x27, 20, 2);

// =========================================================
// WIFI MANAGER CONFIGURATION
// =========================================================

static const char* WIFI_AP_SSID     = "PJU-GATEWAY";
static const char* WIFI_AP_PASSWORD = "11223344";

// =========================================================
// SYSTEM CONFIGURATION
// =========================================================

static const char* NODE_ID = "LPJU01"; // Silakan dirubah berdasarkan nama gateway

static const uint8_t MAX_SLAVE_COUNT = 10;

static const uint32_t LCD_UPDATE_INTERVAL_MS     = 1000;
static const uint32_t LCD_SLIDE_INTERVAL_MS      = 6000;
static const uint32_t SLAVE_TIMEOUT_MS           = 10000;
static const uint32_t STATE_REFRESH_INTERVAL_MS  = 5000;

// =========================================================
// SERVER CONFIGURATION
// =========================================================

static const char* SERVER_BASE_URL = "https://pju.biz.id/api/device/";

static const char* API_KEY = "LPJU_IOT_2026";

static const char* FIRMWARE_VERSION = "1.0.0";

// =========================================================
// TELEMETRY CONFIG
// =========================================================

static const uint32_t TELEMETRY_INTERVAL_MS = 5000;

unsigned long lastTelemetryMillis = 0;

static const uint32_t PZEM_INTERVAL_MS = 5000;

// =========================================================
// PIN CONFIGURATION
// =========================================================

// MASTER UART
static const uint8_t MASTER_RX_PIN = 19;
static const uint8_t MASTER_TX_PIN = 18;

// GPS UART
static const uint8_t GPS_RX_PIN = 32;
static const uint8_t GPS_TX_PIN = 33;

// I2C
static const uint8_t I2C_SDA_PIN = 21;
static const uint8_t I2C_SCL_PIN = 22;

// PZEM UART
static const uint8_t PZEM_RX_PIN = 16;
static const uint8_t PZEM_TX_PIN = 17;

PZEM004Tv30 pzem(sensorSerial,PZEM_RX_PIN,PZEM_TX_PIN);

// =========================================================
// MODE CONFIGURATION
// =========================================================

enum ControlMode
{
    MODE_SCHEDULE,
    MODE_MANUAL
};

enum SensorMode
{
    MODE_NONE,
    MODE_GPS,
    MODE_PZEM
};

ControlMode currentControlMode = MODE_SCHEDULE;
SensorMode currentSensorMode = MODE_NONE;

// =========================================================
// SCHEDULE CONFIGURATION
// =========================================================

// ON Schedule
int onHour   = 17;
int onMinute = 00;
int onSecond = 0;

// OFF Schedule
int offHour   = 6;
int offMinute = 00;
int offSecond = 0;

// =========================================================
// DATA STRUCTURES
// =========================================================

struct SlaveStatus
{
    int slaveId;

    bool relayState;
    bool lampStatusOk;
    bool active;

    unsigned long lastUpdateMillis;
};

// =========================================================
// GLOBAL VARIABLES
// =========================================================

Preferences preferences;

SlaveStatus slaveStatusList[MAX_SLAVE_COUNT];

bool isFirstScheduleSync = true;
bool currentRelayState   = false;
bool isGpsValid          = false;

String currentTimeString = "--:--:--";

String latitudeString    = "0.000000";
String longitudeString   = "0.000000";
int gpsSatelliteCount = 0;
float gpsHdopValue = 0;

String latestAckData     = "NO DATA";

unsigned long lastLcdUpdateMillis    = 0;
unsigned long lastLcdSlideMillis     = 0;
unsigned long lastStateRefreshMillis = 0;

int lcdPageIndex       = 0;
int previousSlaveCount = -1;

float voltageValue = 0;
float currentValue = 0;
float powerValue   = 0;
float energyValue  = 0;

unsigned long lastPzemMillis = 0;

// =========================================================
// FUNCTION DECLARATIONS
// =========================================================

void initializeWiFi();
void initializeRTC();
void initializeLCD();

void synchronizeRTCWithNTP();

void updateCurrentTime();
void updateGPSData();

void updateRelaySchedule();
void resendCurrentRelayState();

void processIncomingSlaveData(const String& incomingData);
void checkSlaveTimeout();

void updateLCD(bool forceUpdate);

int countActiveSlaves();
int convertToSeconds(int hour, int minute, int second);

// =========================================================
// DISPLAY FUNCTIONS
// =========================================================

/**
 * Display boot information
 */
void initializeLCD()
{
    lcd.init();
    lcd.backlight();
    lcd.clear();

    lcd.setCursor(3, 0);
    lcd.print("PJU SYSTEM");

    lcd.setCursor(0, 1);
    lcd.print("[");
    lcd.setCursor(15, 1);
    lcd.print("]");

    for (int i = 1; i < 15; i++) {
        lcd.setCursor(i, 1);
        lcd.print((char)255);
        delay(150);
    }

    lcd.clear();
}

/**
 * Update LCD monitoring pages.
 *
 * Page 0 : System state
 * Page N : Slave monitoring
 */
void updateLCD(bool forceUpdate)
{
    if (!forceUpdate)
    {
        if (millis() - lastLcdUpdateMillis < LCD_UPDATE_INTERVAL_MS)
        {
            return;
        }
    }

    lastLcdUpdateMillis = millis();

    int activeSlaveCount = countActiveSlaves();

    if (activeSlaveCount != previousSlaveCount)
    {
        lcdPageIndex = 0;
        previousSlaveCount = activeSlaveCount;
    }

    int totalPages = 2;

    if (currentControlMode == MODE_SCHEDULE)
    {
        totalPages += 2;
    }

    // PZEM PAGE
    totalPages += 4;

    // GPS PAGE
    totalPages += 2;
    
    totalPages += activeSlaveCount;

    if (millis() - lastLcdSlideMillis >= LCD_SLIDE_INTERVAL_MS)
    {
        lastLcdSlideMillis = millis();

        lcdPageIndex++;

        if (lcdPageIndex >= totalPages)
        {
            lcdPageIndex = 0;
        }
    }

    lcd.clear();

    // =====================================================
    // LINE 1 : TIME
    // =====================================================

    lcd.setCursor(0, 0);
    lcd.print("TIME :");
    lcd.print(currentTimeString);

    // =====================================================
    // PAGE 0 : SYSTEM STATE
    // =====================================================

    lcd.setCursor(0, 1);

    if (lcdPageIndex == 0)
    {
        lcd.print("STATE:");
        lcd.print(currentRelayState ? "ON" : "OFF");

        return;
    }

    // =====================================================
    // PAGE 1 : CONTROL MODE
    // =====================================================
    
    if (lcdPageIndex == 1)
    {
        lcd.print("MODE :");
    
        lcd.print(
            currentControlMode == MODE_MANUAL
            ? "MANUAL"
            : "SCHEDULE"
        );
    
        return;
    }

    // =====================================================
    // PAGE 2 : ON SCHEDULE
    // =====================================================
    
    if (
        currentControlMode == MODE_SCHEDULE &&
        lcdPageIndex == 2
    )
    {
        lcd.print("ON   :");
    
        lcd.print(
            formatScheduleTime(
                onHour,
                onMinute,
                onSecond
            )
        );
    
        return;
    }
    
    // =====================================================
    // PAGE 3 : OFF SCHEDULE
    // =====================================================
    
    if (
        currentControlMode == MODE_SCHEDULE &&
        lcdPageIndex == 3
    )
    {
        lcd.print("OFF  :");
    
        lcd.print(
            formatScheduleTime(
                offHour,
                offMinute,
                offSecond
            )
        );
    
        return;
    }

    // =====================================================
    // PAGE PZEM OFFSET
    // =====================================================
    
    int pzemPageOffset =
        (currentControlMode == MODE_SCHEDULE)
        ? 4
        : 2;
    
    // =====================================================
    // PAGE : VOLTAGE
    // =====================================================
    
    if (lcdPageIndex == pzemPageOffset)
    {
        lcd.print("VOLT :");
        lcd.print(voltageValue, 1);
        lcd.print("V");
    
        return;
    }
    
    // =====================================================
    // PAGE : CURRENT
    // =====================================================
    
    if (lcdPageIndex == pzemPageOffset + 1)
    {
        lcd.print("CURR :");
        lcd.print(currentValue, 2);
        lcd.print("A");
    
        return;
    }
    
    // =====================================================
    // PAGE : POWER
    // =====================================================
    
    if (lcdPageIndex == pzemPageOffset + 2)
    {
        lcd.print("PWR  :");
        lcd.print(powerValue, 1);
        lcd.print("W");
    
        return;
    }
    
    // =====================================================
    // PAGE : ENERGY
    // =====================================================
    
    if (lcdPageIndex == pzemPageOffset + 3)
    {
        lcd.print("ENG  :");
        lcd.print(energyValue, 2);
        lcd.print("kWh");
        return;
    }

    // =====================================================
    // PAGE GPS OFFSET
    // =====================================================
    
    int gpsPageOffset =
        pzemPageOffset + 4;
    
    // =====================================================
    // PAGE : GPS SATELLITE
    // =====================================================
    
    if (lcdPageIndex == gpsPageOffset)
    {
        lcd.print("G.SAT:");
    
        lcd.print(gpsSatelliteCount);
    
        return;
    }
    
    // =====================================================
    // PAGE : GPS HDOP
    // =====================================================
    
    if (lcdPageIndex == gpsPageOffset + 1)
    {
        lcd.print("G.HDO:");
    
        if (gpsHdopValue <= 1)
        {
            lcd.print("EXCELLENT");
        }
        else if (gpsHdopValue <= 2)
        {
            lcd.print("GOOD");
        }
        else if (gpsHdopValue <= 5)
        {
            lcd.print("NORMAL");
        }
        else
        {
            lcd.print("WEAK");
        }
    
        return;
    }

    // =====================================================
    // NO SLAVE AVAILABLE
    // =====================================================

    if (activeSlaveCount == 0)
    {
        lcd.print("LOADING SLAVE...");
        return;
    }

    // =====================================================
    // SHOW ACTIVE SLAVE DATA
    // =====================================================

    int activePageCounter = 0;

    for (int i = 0; i < MAX_SLAVE_COUNT; i++)
    {
        if (!slaveStatusList[i].active)
        {
            continue;
        }

        activePageCounter++;

        int slavePageOffset =
            (currentControlMode == MODE_SCHEDULE)
            ? 10
            : 8;
        
        if (
            activePageCounter ==
            (lcdPageIndex - slavePageOffset + 1)
        )
        {
            lcd.print("SLV");
            lcd.print(slaveStatusList[i].slaveId);

            lcd.print(" :R.");
            lcd.print(slaveStatusList[i].relayState ? "ON" : "OFF");

            lcd.print("-L.");
            lcd.print(slaveStatusList[i].lampStatusOk ? "OK" : "ER");

            break;
        }
    }
}

// =========================================================
// WIFI FUNCTIONS
// =========================================================

/**
 * Initialize WiFi connection using WiFiManager.
 */
void initializeWiFi()
{
    WiFiManager wifiManager;

    wifiManager.setConfigPortalTimeout(180);

    // =====================================
    // SHOW WAITING WIFI INFO
    // =====================================

    lcd.clear();

    lcd.setCursor(0, 0);
    lcd.print("PLEASE CONNECT :");

    lcd.setCursor(0, 1);
    lcd.print(WIFI_AP_SSID);

    Serial.println();
    Serial.println("WAITING WIFI CONFIGURATION");

    // =====================================
    // START AUTO CONNECT
    // =====================================

    bool isConnected = wifiManager.autoConnect(
        WIFI_AP_SSID,
        WIFI_AP_PASSWORD
    );

    // =====================================
    // FAILED
    // =====================================

    if (!isConnected)
    {
        Serial.println("WIFI CONNECTION FAILED");

        lcd.clear();

        lcd.setCursor(0, 0);
        lcd.print("WIFI FAILED");

        delay(2000);

        ESP.restart();
    }

    // =====================================
    // SUCCESS
    // =====================================

    Serial.println();
    Serial.println("WIFI CONNECTED");

    Serial.print("IP ADDRESS : ");
    Serial.println(WiFi.localIP());

    lcd.clear();

    lcd.setCursor(0, 0);
    lcd.print("WIFI CONNECTED");

    lcd.setCursor(0, 1);
    lcd.print(WiFi.localIP());

    delay(2000);
}

// =========================================================
// RTC FUNCTIONS
// =========================================================

/**
 * Initialize DS3231 RTC module.
 */
void initializeRTC()
{
    Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);

    if (!rtc.begin())
    {
        Serial.println("RTC MODULE NOT FOUND");

        lcd.clear();
        lcd.setCursor(0, 0);
        lcd.print("RTC NOT FOUND");

        while (true)
        {
        }
    }

    Serial.println("RTC READY");
}

/**
 * Synchronize RTC with NTP server.
 */
void synchronizeRTCWithNTP()
{
    // WIB UTC+7
    configTime(
        7 * 3600,
        0,
        "pool.ntp.org",
        "time.nist.gov"
    );

    Serial.println();
    Serial.println("SYNC RTC VIA NTP");

    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("SYNC RTC...");

    struct tm timeInfo;

    if (!getLocalTime(&timeInfo))
    {
        Serial.println("NTP SYNC FAILED");

        lcd.setCursor(0, 1);
        lcd.print("NTP FAILED");

        return;
    }

    rtc.adjust(DateTime(
        timeInfo.tm_year + 1900,
        timeInfo.tm_mon + 1,
        timeInfo.tm_mday,
        timeInfo.tm_hour,
        timeInfo.tm_min,
        timeInfo.tm_sec
    ));

    updateCurrentTime();

    Serial.println("RTC SYNC SUCCESS");
    Serial.print("TIME WIB : ");
    Serial.println(currentTimeString);

    lcd.setCursor(0, 1);
    lcd.print(currentTimeString);

    delay(2000);
}

/**
 * Update current RTC time string.
 */
void updateCurrentTime()
{
    DateTime now = rtc.now();

    char timeBuffer[20];

    sprintf(
        timeBuffer,
        "%02d:%02d:%02d",
        now.hour(),
        now.minute(),
        now.second()
    );

    currentTimeString = String(timeBuffer);
}

// =========================================================
// PZEM FUNCTIONS
// =========================================================

void switchToPZEM()
{
    if (currentSensorMode == MODE_PZEM)
    {
        return;
    }

    sensorSerial.end();

    sensorSerial.begin(
        9600,
        SERIAL_8N1,
        PZEM_RX_PIN,
        PZEM_TX_PIN
    );

    currentSensorMode = MODE_PZEM;

    delay(100);
}

void updatePZEMData()
{
    switchToPZEM();

    voltageValue = pzem.voltage();
    currentValue = pzem.current();
    powerValue   = pzem.power();
    energyValue  = pzem.energy();

    if (isnan(voltageValue)) voltageValue = 0;
    if (isnan(currentValue)) currentValue = 0;
    if (isnan(powerValue)) powerValue = 0;
    if (isnan(energyValue)) energyValue = 0;
}

// =========================================================
// GPS FUNCTIONS
// =========================================================

void switchToGPS()
{
    if (currentSensorMode == MODE_GPS)
    {
        return;
    }

    sensorSerial.end();

    sensorSerial.begin(
        9600,
        SERIAL_8N1,
        GPS_RX_PIN,
        GPS_TX_PIN
    );

    currentSensorMode = MODE_GPS;

    delay(100);
}

/**
 * Read and update GPS coordinates.
 */
void updateGPSData()
{
    switchToGPS();

    while (sensorSerial.available())
    {
        gps.encode(sensorSerial.read());
    }

    // =====================================
    // LOCATION
    // =====================================

    if (gps.location.isValid())
    {
        latitudeString  = String(gps.location.lat(), 6);
        longitudeString = String(gps.location.lng(), 6);

        isGpsValid = true;
    }
    else
    {
        isGpsValid = false;
    }

    // =====================================
    // SATELLITES
    // =====================================

    if (gps.satellites.isValid())
    {
        gpsSatelliteCount = gps.satellites.value();
    }

    // =====================================
    // HDOP
    // =====================================

    if (gps.hdop.isValid())
    {
        gpsHdopValue = gps.hdop.hdop();
    }
}

// =========================================================
// SCHEDULE FUNCTIONS
// =========================================================

/**
 * Convert time into total seconds.
 */
int convertToSeconds(int hour, int minute, int second)
{
    return (hour * 3600) +
           (minute * 60) +
           second;
}

/**
 * Update relay state based on RTC schedule.
 */
void updateRelaySchedule()
{
    // =====================================
    // SKIP IF MANUAL MODE
    // =====================================

    if (currentControlMode == MODE_MANUAL)
    {
        return;
    }
    
    DateTime now = rtc.now();

    int currentTimeSeconds = convertToSeconds(
        now.hour(),
        now.minute(),
        now.second()
    );

    int onTimeSeconds = convertToSeconds(
        onHour,
        onMinute,
        onSecond
    );

    int offTimeSeconds = convertToSeconds(
        offHour,
        offMinute,
        offSecond
    );

    bool newRelayState = false;

    // =====================================================
    // NORMAL MODE
    // Example:
    // ON  = 17:00
    // OFF = 06:00
    // =====================================================

    if (onTimeSeconds < offTimeSeconds)
    {
        newRelayState =
            (currentTimeSeconds >= onTimeSeconds) &&
            (currentTimeSeconds < offTimeSeconds);
    }

    // =====================================================
    // CROSS-DAY MODE
    // =====================================================

    else
    {
        newRelayState =
            (currentTimeSeconds >= onTimeSeconds) ||
            (currentTimeSeconds < offTimeSeconds);
    }

    // =====================================================
    // SEND STATE ONLY WHEN CHANGED
    // =====================================================

    if ((newRelayState != currentRelayState) ||
        isFirstScheduleSync)
    {
        currentRelayState = newRelayState;

        updateLCD(true);

        masterSerial.println(
            currentRelayState ? "ON" : "OFF"
        );

        Serial.print(currentTimeString);
        Serial.print(" -> SCHEDULE : ");
        Serial.println(currentRelayState ? "ON" : "OFF");

        isFirstScheduleSync = false;
    }
}

/**
 * Periodically resend current relay state
 * to ensure synchronization with slaves.
 */
void resendCurrentRelayState()
{
    if (millis() - lastStateRefreshMillis <
        STATE_REFRESH_INTERVAL_MS)
    {
        return;
    }

    lastStateRefreshMillis = millis();

    masterSerial.println(
        currentRelayState ? "ON" : "OFF"
    );

    Serial.print(currentTimeString);
    Serial.print(" -> REFRESH : ");
    Serial.println(currentRelayState ? "ON" : "OFF");
}

/**
 * Load saved schedule from NVS memory.
 */
void loadSavedSchedule()
{
    onHour =
        preferences.getInt(
            "on_hour",
            17
        );

    onMinute =
        preferences.getInt(
            "on_minute",
            0
        );

    onSecond =
        preferences.getInt(
            "on_second",
            0
        );

    offHour =
        preferences.getInt(
            "off_hour",
            5
        );

    offMinute =
        preferences.getInt(
            "off_minute",
            0
        );

    offSecond =
        preferences.getInt(
            "off_second",
            0
        );

    Serial.println(
        "SCHEDULE LOADED");
}

/**
 * Save schedule into NVS memory.
 */
void saveSchedule()
{
    preferences.putInt(
        "on_hour",
        onHour
    );

    preferences.putInt(
        "on_minute",
        onMinute
    );

    preferences.putInt(
        "on_second",
        onSecond
    );

    preferences.putInt(
        "off_hour",
        offHour
    );

    preferences.putInt(
        "off_minute",
        offMinute
    );

    preferences.putInt(
        "off_second",
        offSecond
    );

    Serial.println(
        "SCHEDULE SAVED");
}

// =========================================================
// SLAVE MONITORING FUNCTIONS
// =========================================================

/**
 * Count total active slave devices.
 */
int countActiveSlaves()
{
    int totalActiveSlaves = 0;

    for (int i = 0; i < MAX_SLAVE_COUNT; i++)
    {
        if (slaveStatusList[i].active)
        {
            totalActiveSlaves++;
        }
    }

    return totalActiveSlaves;
}

/**
 * Parse incoming slave monitoring data.
 *
 * Expected format:
 * DATA:1,ON,0
 */
void processIncomingSlaveData(const String& incomingData)
{
    String parsedData = incomingData;

    parsedData.replace("DATA:", "");

    int firstCommaIndex =
        parsedData.indexOf(',');

    int secondCommaIndex =
        parsedData.indexOf(',',
                           firstCommaIndex + 1);

    if (firstCommaIndex == -1 ||
        secondCommaIndex == -1)
    {
        return;
    }

    int slaveId =
        parsedData.substring(
            0,
            firstCommaIndex
        ).toInt();

    String relayStatus =
        parsedData.substring(
            firstCommaIndex + 1,
            secondCommaIndex
        );

    int ldrStatus =
        parsedData.substring(
            secondCommaIndex + 1
        ).toInt();

    bool relayIsOn = (relayStatus == "ON");

    bool lampStatusOk =
        (relayIsOn && ldrStatus == 0) ||
        (!relayIsOn && ldrStatus == 1);

    slaveStatusList[slaveId].slaveId         = slaveId;
    slaveStatusList[slaveId].relayState      = relayIsOn;
    slaveStatusList[slaveId].lampStatusOk    = lampStatusOk;
    slaveStatusList[slaveId].active          = true;
    slaveStatusList[slaveId].lastUpdateMillis = millis();
}

/**
 * Disable inactive slave devices after timeout.
 */
void checkSlaveTimeout()
{
    for (int i = 0; i < MAX_SLAVE_COUNT; i++)
    {
        if (!slaveStatusList[i].active)
        {
            continue;
        }

        if (millis() -
            slaveStatusList[i].lastUpdateMillis >
            SLAVE_TIMEOUT_MS)
        {
            slaveStatusList[i].active = false;

            Serial.print(currentTimeString);
            Serial.print(" -> SLAVE TIMEOUT : ");
            Serial.println(i);
        }
    }
}

// =========================================================
// SERVER RESPONSE FUNCTION
// =========================================================

/**
   * Helper Get Current Mode.
 */
String getControlModeString()
{
    switch (currentControlMode)
    {
        case MODE_SCHEDULE:
            return "SCHEDULE";

        case MODE_MANUAL:
            return "MANUAL";

        default:
            return "UNKNOWN";
    }
}

/**
 * Build formatted HH:MM:SS schedule string.
 */
String formatScheduleTime(
    int hour,
    int minute,
    int second)
{
    char buffer[16];

    sprintf(
        buffer,
        "%02d:%02d:%02d",
        hour,
        minute,
        second
    );

    return String(buffer);
}

/**
 * Process command response from server.
 */
void processServerResponse(
    const String& responseBody)
{
    DynamicJsonDocument doc(4096);

    DeserializationError error =
        deserializeJson(doc, responseBody);

    if (error)
    {
        Serial.print("JSON PARSE FAILED : ");
        Serial.println(error.c_str());
        return;
    }

    // =====================================
    // CHECK DATA OBJECT
    // =====================================

    if (!doc.containsKey("data"))
    {
        Serial.println("NO DATA FIELD");
        return;
    }

    JsonObject data = doc["data"];

    if (!data.containsKey("command"))
    {
        Serial.println("NO COMMAND");
        return;
    }

    JsonObject command = data["command"];

    // =====================================
    // READ COMMAND TYPE
    // =====================================

    String commandType = command["type"] | "NO COMMAND";

    Serial.print("COMMAND TYPE : ");

    Serial.println(commandType);

    // =====================================
    // RESTART DEVICE
    // =====================================
    
    if (commandType == "RESTART_DEVICE")
    {
        Serial.println();
        Serial.println("RESTART COMMAND RECEIVED");
    
        Serial.println("DEVICE RESTARTING...");
    
        delay(1000);
    
        ESP.restart();
    
        return;
    }

    // =====================================
    // SET MODE
    // =====================================

    if (commandType == "SET_MODE")
    {
        String typeMode = command["control_mode"] | "";

        Serial.print("SET MODE -> ");

        Serial.println(typeMode);

        if (typeMode == "SCHEDULE")
        {
            currentControlMode = MODE_SCHEDULE;
            Serial.println("CONTROL MODE = SCHEDULE");
        }
        else if (typeMode == "MANUAL")
        {
            currentControlMode = MODE_MANUAL;
            Serial.println("CONTROL MODE = MANUAL");
        }

        return;
    }

    // =====================================
    // SET SCHEDULE
    // =====================================
    
    if (commandType == "SET_SCHEDULE")
    {
        // =================================
        // ONLY ALLOWED IN SCHEDULE MODE
        // =================================
    
        if (currentControlMode != MODE_SCHEDULE)
        {
            return;
        }
    
        String onTime =
            command["on_time"] | "";
    
        String offTime =
            command["off_time"] | "";
    
        Serial.println("UPDATING SCHEDULE");
    
        // =================================
        // PARSE ON TIME
        // =================================
    
        sscanf(
            onTime.c_str(),
            "%d:%d:%d",
            &onHour,
            &onMinute,
            &onSecond
        );
    
        // =================================
        // PARSE OFF TIME
        // =================================
    
        sscanf(
            offTime.c_str(),
            "%d:%d:%d",
            &offHour,
            &offMinute,
            &offSecond
        );
    
        // =================================
        // SAVE TO NVS
        // =================================
    
        saveSchedule();
    
        Serial.print("ON SCHEDULE : ");
    
        Serial.println(
            formatScheduleTime(
                onHour,
                onMinute,
                onSecond
            ));
    
        Serial.print("OFF SCHEDULE : ");
    
        Serial.println(
            formatScheduleTime(
                offHour,
                offMinute,
                offSecond
            ));
    
        return;
    }

    // =====================================
    // MANUAL COMMAND ONLY
    // =====================================

    if (currentControlMode != MODE_MANUAL)
    {
        return;
    }

    // =====================================
    // STATE ON
    // =====================================

    if (commandType == "STATE_ON")
    {
        currentRelayState = true;
        masterSerial.println("ON");
        Serial.println("RELAY -> ON");
    }

    // =====================================
    // STATE OFF
    // =====================================

    else if (commandType == "STATE_OFF")
    {
        currentRelayState = false;
        masterSerial.println("OFF");
        Serial.println("RELAY -> OFF");
    }
}
 
/**
 * Send telemetry data to backend server.
 */
void sendTelemetry()
{
    // =========================================
    // CHECK WIFI
    // =========================================

    if (WiFi.status() != WL_CONNECTED)
    {
        Serial.println("TELEMETRY FAILED : WIFI DISCONNECTED");
        return;
    }

    // =========================================
    // INTERVAL CHECK
    // =========================================

    if (millis() - lastTelemetryMillis < TELEMETRY_INTERVAL_MS)
    {
        return;
    }

    lastTelemetryMillis = millis();

    String endpointUrl = String(SERVER_BASE_URL) + NODE_ID;

    HTTPClient http;

    http.begin(endpointUrl);

    // =========================================
    // HEADERS
    // =========================================

    http.addHeader("Content-Type", "application/json");
    http.addHeader("x-api-key", API_KEY);

    // =========================================
    // JSON DOCUMENT
    // =========================================

    DynamicJsonDocument doc(4096);

    doc["node_id"]          = NODE_ID;
    doc["gateway_state"]    = currentRelayState;
    doc["firmware_version"] = FIRMWARE_VERSION;
    doc["control_mode"]     = getControlModeString();

    if (currentControlMode == MODE_SCHEDULE)
    {
        doc["on_schedule"] =
            formatScheduleTime(
                onHour,
                onMinute,
                onSecond
            );
    
        doc["off_schedule"] =
            formatScheduleTime(
                offHour,
                offMinute,
                offSecond
            );
    }

    doc["wifi_rssi"] = WiFi.RSSI();

    doc["uptime"] = millis() / 1000;

    doc["free_heap"] = ESP.getFreeHeap();

    // =========================================
    // PZEM PLACEHOLDER
    // =========================================

    doc["voltage"] = voltageValue;
    doc["current"] = currentValue;
    doc["power"]   = powerValue;
    doc["energy"]  = energyValue;

    // =========================================
    // RTC
    // =========================================

    doc["rtc_time"] = currentTimeString;

    // =========================================
    // GPS
    // =========================================

    doc["gps_satellites"] = gpsSatelliteCount;
    doc["gps_hdop"] = gpsHdopValue;

    if (isGpsValid)
    {
        doc["latitude"] = latitudeString.toFloat();
        doc["longitude"] = longitudeString.toFloat();
    }
    else
    {
        doc["latitude"]  = "-";
        doc["longitude"] = "-";
    }

    // =========================================
    // SLAVE ARRAY
    // =========================================

    JsonArray slaveArray = doc.createNestedArray("slaves");

    for (int i = 0; i < MAX_SLAVE_COUNT; i++)
    {
        if (!slaveStatusList[i].active)
        {
            continue;
        }

        JsonObject slave = slaveArray.createNestedObject();

        slave["slave_id"] = slaveStatusList[i].slaveId;

        slave["state"] = slaveStatusList[i].relayState;

        slave["lamp_ok"] = slaveStatusList[i].lampStatusOk;
    }

    // =========================================
    // SERIALIZE JSON
    // =========================================

    String requestBody;

    serializeJson(doc, requestBody);

    // =========================================
    // SERIAL LOG REQUEST
    // =========================================

    Serial.println();
    Serial.println("========== TELEMETRY ==========");
    Serial.println(requestBody);

    // =========================================
    // HTTP POST
    // =========================================

    int httpCode = http.POST(requestBody);

    // =========================================
    // RESPONSE
    // =========================================

    Serial.print("HTTP CODE : ");
    Serial.println(httpCode);

    if (httpCode > 0)
    {
        String response = http.getString();

        Serial.println("SERVER RESPONSE :");
        Serial.println(response);
        processServerResponse(response);
    }
    else
    {
        Serial.print("HTTP ERROR : ");
        Serial.println(http.errorToString(httpCode));
    }

    http.end();

    Serial.println("===============================");
}

// =========================================================
// SETUP
// =========================================================

void setup()
{
    Serial.begin(115200);

    Serial.println();
    Serial.println("ESP32 GATEWAY STARTING");

    // =====================================================
    // PREFERENCES INITIALIZATION
    // =====================================================

    preferences.begin(
        "gateway-config",
        false
    );

    loadSavedSchedule();

    // =====================================================
    // LCD INITIALIZATION
    // =====================================================

    initializeLCD();

    // =====================================================
    // MASTER UART INITIALIZATION
    // =====================================================

    masterSerial.begin(
        115200,
        SERIAL_8N1,
        MASTER_RX_PIN,
        MASTER_TX_PIN
    );

    // =====================================================
    // RTC INITIALIZATION
    // =====================================================

    initializeRTC();

    // =====================================================
    // WIFI INITIALIZATION
    // =====================================================

    initializeWiFi();

    // =====================================================
    // RTC NTP SYNCHRONIZATION
    // =====================================================

    synchronizeRTCWithNTP();

    lcd.clear();

    lcd.setCursor(0, 0);
    lcd.print("SYSTEM READY");

    delay(2000);

    Serial.println();
    Serial.println("SYSTEM READY");
}

// =========================================================
// MAIN LOOP
// =========================================================

void loop()
{
    // =====================================================
    // PERIODIC SYSTEM UPDATE
    // =====================================================

    updateCurrentTime();

    updateGPSData();

    if (millis() - lastPzemMillis >= PZEM_INTERVAL_MS)
    {
        lastPzemMillis = millis();
        updatePZEMData();
    }

    checkSlaveTimeout();

    updateLCD(false);

    updateRelaySchedule();

    resendCurrentRelayState();

    sendTelemetry();

    // =====================================================
    // RECEIVE DATA FROM MASTER
    // =====================================================

    if (masterSerial.available())
    {
        String incomingData = masterSerial.readStringUntil('\n');

        incomingData.trim();

        if (incomingData.startsWith("DATA:"))
        {
            latestAckData = incomingData;

            processIncomingSlaveData(incomingData);

            Serial.print(currentTimeString);
            Serial.print(" -> ACK : ");
            Serial.println(incomingData);
        }
    }
}
