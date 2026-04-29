/*   Combined Biltong controller v1.63
     - WEIGHT_SCALE_FACTOR = 1.28 (target 1584g). Fast EMA on big rise for fewer readings to settle.
     - FIX: WEIGHT_SCALE_FACTOR (correct after tare); zero zone so empty scale shows 0.
     - WEIGHT_SCALE_FACTOR = 7.03 for 1584g; WEIGHT_SAMPLES = 15 to avoid loop blocking.
     - NEW: WEIGHT_SCALE_FACTOR so displayed weight matches actual when HX711 reads low.
     - FIX: Weight updated in main loop (not only in sendAllStatus) so value converges in seconds.
     - FIX: Weight outlier only on sudden drop (reject noise); adding weight now updates display quickly.
     - NEW: Weight smoothing — EMA + outlier rejection for stable load-cell reading.
     - NEW: WEIGHT_EMA_ALPHA, WEIGHT_OUTLIER_G, WEIGHT_SAMPLES (50) reduce noise and spikes.
     - FIX: Version history updated for HA weight UI fixes (dashboard, automation, tare button).
     - NOTE: Weight in status line and button behavior are configured in Home Assistant (see HA_Biltong_Weight_Fixes.md).
     - FIX: Duration calculation now uses millis() instead of getCurrentTime() for accuracy.
     - FIX: Duration now displays correctly instead of "--:--:--".
     - FIX: Uses systemStartMillis to calculate elapsed time since startTime was set.
     - NEW: ESP32 now calculates and sends duration as formatted string to HA.
     - NEW: Duration format: "HH:MM:SS" or "Nd HH:MM:SS" for days.
     - NEW: HA no longer needs to calculate duration - receives ready string.
     - FIX: Eliminated timezone calculation issues for duration display.
     - FIX: Duration updates every 30 seconds when MQTT is connected.
     - FIX: Fixed timezone conversion - HA sends UTC, ESP32 adds timezone offset for display.
     - FIX: Start time now displays correctly in Home Assistant (no more timezone offset).
     - FIX: ESP32 receives UTC timestamp from HA and converts to local time for display.
     - FIX: Fixed timezone conversion - startTime is already UTC from HA, no double conversion.
     - FIX: Start time now displays correctly in Home Assistant (no more timezone offset).
     - FIX: ESP32 receives UTC timestamp from HA and sends it back as-is.
     - FIX: Fixed timezone conversion for HA - ESP32 now sends UTC timestamp to HA.
     - FIX: Start time now displays correctly in Home Assistant (no more timezone offset).
     - FIX: ESP32 uses local timezone internally but sends UTC to HA for compatibility.
     - FIX: Fixed timezone issue - ESP32 now uses Israel timezone (UTC+2/UTC+3).
     - FIX: Start time now displays correctly in Home Assistant (no more 3-hour offset).
     - FIX: Added timezone configuration for proper local time display.
     - FIX: Fixed humidity display in web interface (was showing 0.0%).
     - FIX: Fixed Clear Debug Messages button - now actually clears messages.
     - FIX: Added descriptive text for reset reasons instead of just numbers.
     - FIX: Reset reason 3 now shows "Software reset" instead of just "3".
     - FIX: Added comprehensive debug messages to web interface.
     - FIX: Debug messages now show system startup, WiFi, MQTT, and periodic status.
     - FIX: Added heartbeat messages every 30 seconds with sensor readings.
     - FIX: Debug messages now appear in web interface when redirectSerialToWeb=true.
     - FIX: Replaced ESPAsyncWebServer with standard WebServer for better ESP32 compatibility.
     - FIX: Fixed MD5 authentication errors in web server.
     - FIX: Updated web server handlers to use standard WebServer API.
     - FIX: Fixed compilation errors for web interface variables.
     - FIX: Added global temperature, humidity, and RPM variables for web display.
     - FIX: Updated all sensor reading functions to update global variables.
     - NEW: Added OTA (Over-The-Air) update functionality with ArduinoOTA.
     - NEW: Added web server interface for system monitoring and control.
     - NEW: Added debug message redirection from Serial to web interface.
     - NEW: Added config.h file for private settings (passwords, IP addresses).
     - NEW: Web interface shows system status, debug messages, and controls.
     - NEW: Toggle between Serial and web debug output.
     - FIX: Fixed HA button behavior to match physical buttons exactly.
     - FIX: Heater button from HA now properly updates heaterButtonState.
     - FIX: Auto mode switch from HA now properly handles heater state transitions.
     - FIX: Power button from HA now resets heaterButtonState to ON.
     - FIX: Improved MQTT connection management - "System started - OK" alert only on first connection.
     - FIX: Added MQTT connection failure cooldown to prevent spam messages.
     - FIX: Separated initial connection from reconnection behavior.
     - FIX: Updated temperature and humidity thresholds per specification.
     - FIX: Changed tempMax from 32.0°C to 35.0°C per spec.
     - FIX: Updated humidity thresholds: targetHmax=50.0%, targetHmin=30.0% (30-50% ideal range).
     - FIX: Normalized RPM calculation to prevent extreme values (32000+ RPM).
     - FIX: Added RPM normalization logic - sets to 1800 RPM when pulses > 100.
     - FIX: Capped RPM at 2000 maximum to prevent calculation errors.
     - FIX: Improved RPM debug output to show normalized values.
     - NEW: Better RPM stability for TEUCER 1800 RPM fan.

     Version History:
     v1.1 - Fixed MQTT topics, RPM calculation, green button behavior.
     v1.2 - User updated version locally.
     v1.3 - Fixed fan duty cycle and RPM calculation, added HA topics.
     v1.4 - Fixed time and run time publishing, improved time sync logic.
     v1.5 - Removed separate time topics, simplified time sync, fixed CSV output.
     v1.6 - NEW: Advanced auto-control with humidity consideration.
          - NEW: Emergency shutdown & alert system (overheat, sensor fail, etc.).
          - NEW: MQTT alerts publish to biltong/alert topic.
          - NEW: Logic handles heater MOSFET absence gracefully.
          - MOD: Refactored autoControl() and button handlers for better logic.
          - MOD: All shutdown events now maintain minimum fan speed.
     v1.7 - NEW: Added dynamic temperature control via MQTT from Home Assistant using ArduinoJson.
          - MOD: Replaced hard-coded Tmin/Tmax with values received from HA.
     v1.8 - FIX: Resolved compiler errors for `mqtt` variable scope.
          - FIX: Corrected logical OR syntax ('|' changed to '||').
          - FIX: Corrected `snprintf` buffer declaration for LCD output.
     v1.9 - FIX: Further corrected logical OR syntax that remained.
          - FIX: Fixed `StaticJsonDocument` template size.
          - FIX: Corrected `snprintf` buffer to an array.
     v1.10 - FIX: Resolved all remaining syntax and variable declaration errors. The code should now compile cleanly.
     v1.11 - FIX: Final comprehensive fix for all remaining syntax and variable declaration errors. The code should now compile cleanly.
     v1.12 - FIX: Final comprehensive fix for all remaining syntax and variable declaration errors. The code should now compile cleanly.
     v1.20 - FIX: Improved power loss detection using esp_reset_reason().
          - FIX: Better time synchronization logic after power loss.
          - FIX: Enhanced temperature profile change logging.
          - FIX: Added initial alert status to prevent Unknown state in HA.
     v1.21 - FIX: Fixed start time transmission in biltong/status (was sending 0).
          - CLEANUP: Removed all separate MQTT transmissions of start time and run time.
          - MOD: Only biltong/status contains start time, HA calculates run time from it.
     v1.26 - FIX: Improved power loss detection and start time reset logic.
     v1.27 - MAJOR: Complete heater control system overhaul and test mode implementation.
     v1.28 - MAJOR: Fixed all critical control logic issues and RPM calculation.
     v1.29 - FIX: Corrected MQTT status format and final control logic improvements.
     v1.30 - FIX: Corrected MQTT status format - heater=button state, heaterState=actual heater state.
     v1.31 - FIX: Fixed MQTT heater state publishing and RPM calculation issues.
     v1.32 - FIX: Removed RPM reset to 0 and improved MQTT state publishing.
     v1.33 - FIX: Reverted MQTT status format to original - removed heaterState field.
     v1.34 - FIX: Heater button state updates always, added heater state back to status.
     v1.35 - FIX: Fixed transition from manual to auto mode - heater now follows conditions.
     v1.36 - NEW: Added advanced debug mode for system behavior analysis.
     v1.55 - NEW: Added load cell (HX711) weight measurement and publishing to MQTT.
          - NEW: Added weight field to biltong/status CSV (last field, grams).
          - NEW: Added MQTT command biltong/cmd/tare_weight to tare the scale from Home Assistant.
          - NEW: Web UI shows current weight.
     v1.56 - DOC: Version history updated; HA weight fixes documented in HA_Biltong_Weight_Fixes.md.
     v1.57 - NEW: Weight smoothing (EMA + outlier rejection); 50 samples per reading; stable display.
     v1.58 - FIX: Outlier only on sudden drop (noise); weight increase (e.g. add 1.5 kg) now follows quickly.
     v1.59 - FIX: Weight read in main loop (many updates/sec) so EMA converges fast; 25 samples to reduce block.
     v1.60 - NEW: WEIGHT_SCALE_FACTOR to correct load-cell reading (e.g. 6.0 if scale reads ~250g for 1.5 kg).
     v1.61 - WEIGHT_SCALE_FACTOR = 7.03 (1584g actual / 225g read). WEIGHT_SAMPLES = 15 to reduce block risk.
     v1.62 - FIX: WEIGHT_SCALE_FACTOR = 1.24 (after tare get_units ~1280 for 1584g). Zero zone for stable empty display.
     v1.63 - WEIGHT_SCALE_FACTOR = 1.28 (1584 was 1733). Fast EMA when rise > 150g for fewer readings to settle.
     v1.64 - NEW: HX711 calibration mode (zero + known weight) via MQTT/Web UI; saved to Preferences (hxOffset/hxScale).
*/

#define FIRMWARE_VERSION "1.66"

#include <Arduino.h>
#include <WiFi.h>
#include <PubSubClient.h>
#include <Preferences.h>
#include <Wire.h>
#include <hd44780.h>
#include <hd44780ioClass/hd44780_I2Cexp.h>
#include "DHT.h"
#include <ArduinoJson.h>
#include "HX711.h"
#include <esp_system.h>
#include <ArduinoOTA.h>
#include <WebServer.h>
#include <SPIFFS.h>
#include "config.h"
#include <time.h>
#include <sys/time.h>
#include <driver/ledc.h>


// ================= CONFIG =================
// WiFi / MQTT - Now loaded from config.h
// OTA and Web Server settings also in config.h

// Topics - Updated to match Home Assistant configuration
const char* topicGet           = "biltong/cmd/get_start";
const char* topicSet           = "biltong/cmd/set_start";
const char* topicStatus        = "biltong/status";  // Contains: temp,humidity,fan%,rpm,sys,heater,auto,start_time,heater_state,duration,weight_g
const char* topicAlertPub      = "biltong/alert"; // New topic for alerts
const char* topicSetProfile    = "biltong/cmd/set_profile"; // New topic for dynamic temperature control
const char* topicTareWeightCmd = "biltong/cmd/tare_weight"; // New topic for load cell tare
// HX711 calibration
const char* topicCalibZeroCmd  = "biltong/cmd/calib_zero";   // set offset with empty scale
const char* topicCalibWeightCmd= "biltong/cmd/calib_weight"; // payload: grams (e.g. "1584")

// Updated control topics to match HA configuration
const char* topicSystemStatePub = "biltong/state/power";
const char* topicSystemStateSet = "biltong/cmd/power";
const char* topicHeaterStatePub = "biltong/state/heater";
const char* topicHeaterStateSet = "biltong/cmd/heater";
const char* topicModePub        = "biltong/state/auto_mode";
const char* topicModeSet        = "biltong/cmd/auto_mode";

// LED control topics (ready for future MOSFET implementation)
const char* topicLedStatePub = "biltong/led_power";
const char* topicLedStateSet = "biltong/led_power/set";
const char* topicLedBrightnessPub = "biltong/led_brightness";
const char* topicLedBrightnessSet = "biltong/led_brightness/set";

// ================= Hardware pins (all const) =================
const int PIN_DHT           = 4;  // DHT data pin
const int PIN_TACH          = 5;  // tach input (interrupt) - fun RPM - green wire
const int PIN_PWM_FAN       = 16; // PWM pin for main fan - blue wire
const int PIN_HEATER        = 27; // Heater MOSFET control pin (placeholder)
const int PIN_HEATER_FAN    = 26; // Heater fan MOSFET control pin (placeholder)
// Load cell (HX711)
const int PIN_HX_DOUT       = 14; // HX711 DT pin
const int PIN_HX_SCK        = 25; // HX711 SCK pin
// Weight smoothing: EMA alpha (higher = smoother). Outlier = only on sudden DROP (noise spike).
const float WEIGHT_EMA_ALPHA     = 0.88f;   // when stable or small change
const float WEIGHT_EMA_ALPHA_FAST = 0.50f; // when big increase (weight added) — fewer readings to settle
const float WEIGHT_RISE_FAST_G   = 150.0f; // if (raw - currentWeight) > this, use ALPHA_FAST
const float WEIGHT_OUTLIER_G    = 120.0f;  // if (currentWeight - raw) > this → slow blend (reject drop spike)
const int   WEIGHT_SAMPLES      = 15;      // HX711 samples (fewer = faster, less chance of loop blocking)
// IMPORTANT: keep this at 1.0. Use calibration (hxOffset/hxScale) instead of a fudge factor.
const float WEIGHT_SCALE_FACTOR = 1.0f;
const float WEIGHT_ZERO_THRESHOLD = 45.0f; // display 0 when |weight| < this (stable reading when empty)
// const int PIN_PWM_LED       = 17; // FUTURE: PWM pin for LED MOSFET control

// Buttons
const int BTN_SYS_PIN       = 13; // green - master power - green wire
const int BTN_HEATER_PIN    = 33; // red   - manual heater on/off
const int BTN_MODE_PIN      = 32; // yellow - auto/manual

// Debounce time
const unsigned long DEBOUNCE_MS = 50UL;

// ================= Globals =================
// hd44780_I2Cexp lcd; // LCD disabled
DHT dht(PIN_DHT, DHT22);
HX711 scale;
float calibration_factor = 16.22; // Adjust after calibration
// Persisted HX711 calibration (Preferences keys: hxOffset/hxScale)
long  hxOffset = 0;
float hxScale  = 0.0f;
bool  hxCalLoaded = false;

volatile uint32_t tachCount = 0;
uint32_t lastRPMcalc = 0, currentRPM = 0;
int dutyCycle = 0; // 0-255 pwm for main fan

// control states - initialized to ON on startup
bool systemPower = true;
bool heaterPower = true;
bool autoMode    = true;

// MQTT client declaration moved to a global scope
WiFiClient wifiClient;
PubSubClient mqtt(wifiClient);

// New: configuration flags for hardware presence
const bool HEATER_HW_PRESENT = true; // Set to true when heater MOSFET arrives

// New: Target temperature/humidity for auto-control
float tempMin = 20.0; // Tmin from HA
float tempMax = 35.0; // Tmax from HA (updated per spec)
float targetHmax = 50.0; // Max humidity threshold (30-50% ideal range)
float targetHmin = 30.0; // Min humidity threshold (30-50% ideal range)

// Global sensor values for web interface
float temperature = 0.0;
float humidity = 0.0;
int rpm = 0;
float currentWeight = 0.0;       // smoothed weight (EMA + outlier rejection)
bool weightFirstReading = true;  // true until first raw reading applied (for init after tare)

// New: Fan speed presets
const int FAN_SPEED_MIN    = 400; // RPM - for reference
const int FAN_DUTY_MIN     = 40;  // PWM duty cycle equivalent (~400 RPM)
const int FAN_DUTY_MEDIUM  = 128; // 50% duty cycle
const int FAN_DUTY_HIGH    = 255; // 100% duty cycle

// New: Alerting mechanism
unsigned long lastAlertTime = 0;
const unsigned long ALERT_COOLDOWN_MS = 300000; // 5 minutes

// Heater fan delay mechanism
unsigned long heaterFanOffTime = 0;
const unsigned long HEATER_FAN_DELAY_MS = 30000; // 30 seconds
bool heaterFanDelayedOff = false;

// Test mode variables
bool testMode = false;
float testTemperature = 25.0;
float testHumidity = 50.0;

// Advanced debug mode variables
bool advancedDebugMode = false;
unsigned long lastDebugLog = 0;
const unsigned long DEBUG_LOG_INTERVAL = 5000; // 5 seconds

// Fan control variables
int lastFanSpeed = 0;
const int FAN_SPEED_TOLERANCE = 10; // Tolerance for fan speed changes

// MQTT connection tracking
bool mqttInitialConnection = true;
bool mqttConnectionFailed = false;
unsigned long lastMqttFailureTime = 0;
const unsigned long MQTT_FAILURE_COOLDOWN = 30000; // 30 seconds

// Heater button state tracking
bool heaterButtonState = true; // true = ON, false = OFF

// OTA and Web Server variables
WebServer webServer(webServerPort);
String debugMessages = "";
const int MAX_DEBUG_MESSAGES = 100;
bool serialRedirectEnabled = redirectSerialToWeb;

// Time management
Preferences prefs;
time_t startTime = 0;
unsigned long systemStartMillis = 0;
bool initialTimeSyncDone = false;
bool timeRequestSent = false;
bool powerLossReset = false;

// Forward declarations
void publishHeaterState();
void sendAllStatus();
void handleSerialCommands();
void printControlTable();
void logAdvancedDebug();
void addDebugMessage(String message);  // שינוי לטובת visual studio code

// ================= Test Mode Functions =================
void handleSerialCommands() {
  if (Serial.available()) {
    String command = Serial.readStringUntil('\n');
    command.trim();
    command.toLowerCase();
    
    if (command == "testmode") {
      testMode = true;
      addDebugMessage("Test mode activated. Use 'temp=X' and 'humidity=Y' to set values.");
      addDebugMessage("Use 'testend' to exit test mode.");
      addDebugMessage("Current test values: Temp=" + String(testTemperature, 1) + "°C, Humidity=" + String(testHumidity, 1) + "%");
    }
    else if (command == "testend") {
      testMode = false;
      addDebugMessage("Test mode deactivated. Using real sensor values.");
    }
    else if (command.startsWith("temp=")) {
      if (testMode) {
        float newTemp = command.substring(5).toFloat();
        if (newTemp >= 0 && newTemp <= 100) {
          testTemperature = newTemp;
          addDebugMessage("Test temperature set to " + String(testTemperature, 1) + "°C");
        } else {
          addDebugMessage("Invalid temperature. Use 0-100°C");
        }
      } else {
        addDebugMessage("Test mode not active. Use 'testmode' first.");
      }
    }
    else if (command.startsWith("humidity=")) {
      if (testMode) {
        float newHumidity = command.substring(9).toFloat();
        if (newHumidity >= 0 && newHumidity <= 100) {
          testHumidity = newHumidity;
          addDebugMessage("Test humidity set to " + String(testHumidity, 1) + "%");
        } else {
          addDebugMessage("Invalid humidity. Use 0-100%");
        }
      } else {
        addDebugMessage("Test mode not active. Use 'testmode' first.");
      }
    }
    else if (command == "debug") {
      advancedDebugMode = !advancedDebugMode;
      if (!advancedDebugMode) {
        addDebugMessage("Advanced debug mode: " + String(advancedDebugMode ? "ON" : "OFF"));
      }
    }
    else if (command == "debugoff") {
      advancedDebugMode = false;
      addDebugMessage("Advanced debug mode: OFF");
    }
    else if (command == "help") {
      addDebugMessage("Available commands:");
      addDebugMessage("  testmode - Enter test mode");
      addDebugMessage("  testend - Exit test mode");
      addDebugMessage("  temp=X - Set test temperature (X = 0-100)");
      addDebugMessage("  humidity=Y - Set test humidity (Y = 0-100)");
      addDebugMessage("  table - Show control table");
      addDebugMessage("  debug - Toggle advanced debug mode");
      addDebugMessage("  debugoff - Turn off advanced debug mode");
      addDebugMessage("  help - Show this help");
    }
    else if (command == "table") {
      printControlTable();
    }
  }
}

void printControlTable() {
  Serial.println("\n=== BILTONG CONTROL TABLE ===");
  Serial.println("Temperature | Humidity | Heater | Fan Speed | Fan % | Action");
  Serial.println("------------|----------|--------|----------|-------|--------");
  
  // Temperature ranges
  float temps[] = {15, 18, 20, 22, 25, 28, 30, 32, 35};
  float humids[] = {30, 40, 50, 60, 70, 80};
  
  for (int i = 0; i < 9; i++) {
    for (int j = 0; j < 6; j++) {
      float temp = temps[i];
      float humidity = humids[j];
      
      bool heater = (temp < tempMin);
      int fanSpeed = FAN_DUTY_MIN;
      int fanPercent = map(fanSpeed, 0, 255, 0, 100);
      String action = "Normal";
      
      if (temp < tempMin) {
        action = "Heat ON";
      } else if (temp > tempMax) {
        action = "Heat OFF";
      }
      
      if (humidity > targetHmax) {
        fanSpeed = FAN_DUTY_HIGH;
        fanPercent = map(fanSpeed, 0, 255, 0, 100);
        action = "High Fan";
      } else if (humidity < targetHmin) {
        fanSpeed = FAN_DUTY_MEDIUM;
        fanPercent = map(fanSpeed, 0, 255, 0, 100);
        action = "Med Fan";
      }
      
      Serial.printf("%11.1f | %8.1f | %6s | %8d | %5d%% | %s\n", 
                   temp, humidity, heater ? "ON" : "OFF", fanSpeed, fanPercent, action.c_str());
    }
  }
  Serial.printf("\nCurrent settings: tempMin=%.1f°C, tempMax=%.1f°C, humidityMin=%.1f%%, humidityMax=%.1f%%\n",
                tempMin, tempMax, targetHmin, targetHmax);
  Serial.println("===============================\n");
}

// ================= Advanced Debug Functions =================
void logAdvancedDebug() {
  if (!advancedDebugMode) return;
  
  unsigned long currentTime = millis();
  if (currentTime - lastDebugLog < DEBUG_LOG_INTERVAL) return;
  
  lastDebugLog = currentTime;
  
  // Read current sensor values
  float temp, humidity;
  if (testMode) {
    temp = testTemperature;
    humidity = testHumidity;
  } else {
    temp = dht.readTemperature();
    humidity = dht.readHumidity();
  }
  
  // Update global variables for web interface
  temperature = temp;
  ::humidity = humidity;
  
  // Calculate fan percentage
  int fanPercent = map(dutyCycle, 0, 255, 0, 100);
  
  // Get current time
  time_t now = time(nullptr);
  struct tm* timeinfo = localtime(&now);
  char timeStr[20];
  strftime(timeStr, sizeof(timeStr), "%H:%M:%S", timeinfo);
  
  // Log formatted data
  Serial.printf("DEBUG_LOG,%s,%.1f,%.1f,%d,%lu,%s,%s,%s,%s,%s,%s\n",
                timeStr,
                isnan(temp) ? 0.0 : temp,
                isnan(humidity) ? 0.0 : humidity,
                fanPercent,
                currentRPM,
                systemPower ? "ON" : "OFF",
                heaterButtonState ? "ON" : "OFF",
                heaterPower ? "ON" : "OFF",
                autoMode ? "AUTO" : "MANUAL",
                testMode ? "TEST" : "LIVE"
  );
}

void setMainFanSpeed(int newDutyCycle) {
  // Check if the change is significant enough
  if (abs(dutyCycle - newDutyCycle) < FAN_SPEED_TOLERANCE) {
    return; // No significant change, don't update
  }
  
  if (dutyCycle != newDutyCycle) {
    int oldDutyCycle = dutyCycle;
    dutyCycle = newDutyCycle;
    ledcWrite(PIN_PWM_FAN, dutyCycle);
    Serial.printf("Main fan speed changed from %d to %d (%d%%)\n", 
                  oldDutyCycle, dutyCycle, map(dutyCycle, 0, 255, 0, 100));
    lastFanSpeed = dutyCycle;
  }
}

void setHeaterState(bool state) {
  if (!HEATER_HW_PRESENT) {
    if (state) Serial.println("Heater ON command ignored: Hardware not present.");
    heaterPower = false;
    return;
  }
  if (heaterPower != state) {
    heaterPower = state;
    
    if (heaterPower) {
      // הדלקת חימום - מאורר החימום נדלק מיד
      digitalWrite(PIN_HEATER_FAN, LOW);
      digitalWrite(PIN_HEATER, LOW);
      heaterFanDelayedOff = false;
      Serial.println("Heater and heater fan turned ON");
    } else {
      // כיבוי חימום - החימום נכבה מיד, מאורר החימום יישאר דולק עוד 30 שניות
      digitalWrite(PIN_HEATER, HIGH);
      heaterFanOffTime = millis();
      heaterFanDelayedOff = true;
      Serial.println("Heater turned OFF, heater fan will turn off in 30 seconds");
    }
    
    Serial.printf("Heater state changed to %s\n", heaterPower ? "ON" : "OFF");
    publishHeaterState();
  }
}

void IRAM_ATTR tachISR() { tachCount++; }

// ================= Helper: publish state helpers =================
void publishStateString(const char* topic, const char* payload, bool retained = true) {
  if (mqtt.connected()) {
    mqtt.publish(topic, payload, retained);
    Serial.printf("MQTT pub %s -> %s\n", topic, payload);
  }
}

void publishStateInt(const char* topic, int value, bool retained = true) {
  if (mqtt.connected()) {
    mqtt.publish(topic, String(value).c_str(), retained);
    Serial.printf("MQTT pub %s -> %d\n", topic, value);
  }
}

// ================= NEW: Alerting function =================
void sendAlert(const char* message) {
  if (millis() - lastAlertTime > ALERT_COOLDOWN_MS) {
    if (mqtt.connected()) {
      publishStateString(topicAlertPub, message, false);
      Serial.printf("Alert sent: %s\n", message);
      lastAlertTime = millis();
    }
  }
}

void emergencyShutdown(const char* reason) {
  setHeaterState(false);
  setMainFanSpeed(FAN_DUTY_MIN);
  sendAlert(reason);
}

// ================= Time management functions =================
void requestTimeFromHA() {
  if (mqtt.connected() && !initialTimeSyncDone) {
    mqtt.publish(topicGet, "", false);
    timeRequestSent = true;
    Serial.println("Requested time synchronization from Home Assistant");
  }
}

time_t getEffectiveStartTime() {
  // Always return the current value of startTime, which will be 0 if not set
  return startTime;
}

time_t getCurrentTime() {
  time_t effectiveStart = getEffectiveStartTime();
  if (effectiveStart > 0) {
    // effectiveStart is in local time (UTC+3), but HA expects UTC
    // Convert back to UTC by subtracting timezone offset
    time_t localTime = effectiveStart + (millis() - systemStartMillis) / 1000;
    time_t utcTime = localTime - (3 * 3600); // Subtract 3 hours to get UTC
    return utcTime;
  } else {
    return 0; // No valid time available
  }
}

// ================= Missing function: sendAllStatus =================
void publishHeaterState() {
  const char* state = heaterButtonState ? "ON" : "OFF";
  publishStateString(topicHeaterStatePub, state);
    if (!advancedDebugMode) {
      Serial.printf("Published heater button state: %s\n", state);
    }
}

void autoControl() {
  if (!autoMode || !systemPower) return;
  
  // Read current sensor values (use test values if in test mode)
  float temp, humidity;
  if (testMode) {
    temp = testTemperature;
    humidity = testHumidity;
  } else {
    temp = dht.readTemperature();
    humidity = dht.readHumidity();
  }
  
  // Update global variables for web interface
  temperature = temp;
  ::humidity = humidity;
  
  // Check for sensor errors
  if (isnan(temp) || isnan(humidity)) {
    emergencyShutdown("Sensor reading failed");
    return;
  }
  
  // Temperature control logic
  if (temp < tempMin) {
    // Temperature too low - turn on heater only if button is ON
    if (!heaterPower && heaterButtonState) {
      setHeaterState(true);
      Serial.printf("Auto: Temperature %.1f°C < %.1f°C - Turning heater ON\n", temp, tempMin);
    } else if (heaterPower && !heaterButtonState) {
      // Heater is ON but button is OFF - turn off heater
      setHeaterState(false);
      Serial.printf("Auto: Temperature %.1f°C < %.1f°C - Turning heater OFF (button is OFF)\n", temp, tempMin);
    }
    // Don't print message if heater is already OFF and button is OFF
  } else if (temp > tempMax) {
    // Temperature too high - turn off heater
    if (heaterPower) {
      setHeaterState(false);
      Serial.printf("Auto: Temperature %.1f°C > %.1f°C - Turning heater OFF\n", temp, tempMax);
    }
  } else {
    // Temperature in normal range - turn off heater if it's on
    if (heaterPower) {
      setHeaterState(false);
      Serial.printf("Auto: Temperature %.1f°C in normal range (%.1f-%.1f°C) - Turning heater OFF\n", temp, tempMin, tempMax);
    }
  }
  
  // Humidity control logic (optional - for fan speed adjustment)
  if (humidity > targetHmax) {
    // High humidity - increase fan speed
    if (dutyCycle != FAN_DUTY_HIGH) {
      setMainFanSpeed(FAN_DUTY_HIGH);
      Serial.printf("Auto: Humidity %.1f%% > %.1f%% - Fan speed increased to HIGH\n", humidity, targetHmax);
    }
  } else if (humidity < targetHmin) {
    // Low humidity - decrease fan speed
    if (dutyCycle != FAN_DUTY_MEDIUM) {
      setMainFanSpeed(FAN_DUTY_MEDIUM);
      Serial.printf("Auto: Humidity %.1f%% < %.1f%% - Fan speed decreased to MEDIUM\n", humidity, targetHmin);
    }
  } else {
    // Normal humidity - set to minimum fan speed
    if (dutyCycle != FAN_DUTY_MIN) {
      setMainFanSpeed(FAN_DUTY_MIN);
      Serial.printf("Auto: Humidity %.1f%% normal - Fan speed set to MINIMUM\n", humidity);
    }
  }
}

// Weight for display/MQTT: 0 when near zero to avoid drift when scale is empty
float getDisplayWeight() {
  return (fabsf(currentWeight) < WEIGHT_ZERO_THRESHOLD) ? 0.0f : currentWeight;
}

void saveHxOffsetScale() {
  prefs.begin("biltong", false);
  prefs.putLong("hxOffset", hxOffset);
  if (hxScale > 0.0f) prefs.putFloat("hxScale", hxScale);
  prefs.end();
}

void sendAllStatus() {
  if (!mqtt.connected()) return;

  if (!advancedDebugMode) {
    Serial.printf("DEBUG: startTime inside sendAllStatus is %lu\n", startTime);
  }
  
  // Read DHT sensor (use test values if in test mode)
  float temp, humidity;
  if (testMode) {
    temp = testTemperature;
    humidity = testHumidity;
  } else {
    temp = dht.readTemperature();
    humidity = dht.readHumidity();
  }
  
  // Update global variables for web interface
  temperature = temp;
  ::humidity = humidity;

  // Weight is updated in loop() every time scale is ready (many times per 30s) — not here.
  // sendAllStatus() only uses the global currentWeight.

  // Calculate RPM if needed
  if (millis() - lastRPMcalc >= 1000) {
    // RPM calculation: (pulses per second) * 60 / pulses per revolution
    // TEUCER PC Fan: 1800 RPM max, 2 pulses per revolution
    uint32_t pulses = tachCount;
    
    // Normalize RPM calculation to prevent extreme values
    if (pulses > 100) {
      // If pulses are too high, normalize to reasonable range
      // For 1800 RPM fan: 1800 * 2 / 60 = 60 pulses per second
      // So 100+ pulses indicates noise or calculation error
      currentRPM = 1800; // Set to nominal fan speed
    } else if (pulses > 0) {
      currentRPM = (pulses * 60) / 2; // 2 pulses per revolution for PC fans
      // Cap RPM at reasonable maximum (2000 RPM)
      if (currentRPM > 2000) {
        currentRPM = 1800; // Set to nominal fan speed
      }
    } else {
      currentRPM = 0; // No pulses detected
    }
    
    // Reset counter after reading
    tachCount = 0;
    lastRPMcalc = millis();
    
    // Update global RPM variable for web interface
    rpm = currentRPM;
    
    // Debug output (suppressed in advanced debug mode)
    if (!advancedDebugMode) {
      Serial.printf("RPM calculation: %lu pulses in 1 second = %lu RPM (normalized)\n", pulses, currentRPM);
    }
  }
  
  // Calculate duration string
  String durationStr = "--:--:--";
  if (startTime > 0) {
    // Calculate duration in seconds using millis() for accuracy
    // startTime is in local time (UTC+3), systemStartMillis is when it was set
    unsigned long currentMillis = millis();
    unsigned long elapsedMillis = currentMillis - systemStartMillis;
    unsigned long duration = elapsedMillis / 1000; // Convert to seconds
    
    if (duration > 0) {
      // Convert to days, hours, minutes, seconds
      int days = duration / 86400;
      int hours = (duration % 86400) / 3600;
      int minutes = (duration % 3600) / 60;
      int seconds = duration % 60;
      
      if (days > 0) {
        durationStr = String(days) + "d " + 
                     String(hours < 10 ? "0" : "") + String(hours) + ":" +
                     String(minutes < 10 ? "0" : "") + String(minutes) + ":" +
                     String(seconds < 10 ? "0" : "") + String(seconds);
      } else {
        durationStr = String(hours < 10 ? "0" : "") + String(hours) + ":" +
                     String(minutes < 10 ? "0" : "") + String(minutes) + ":" +
                     String(seconds < 10 ? "0" : "") + String(seconds);
      }
    }
  }
  
  // Create status payload
  char startTimeStr[12]; // Buffer גדול מספיק ל-ULong
  
  // HA sends UTC timestamp, but we need to add timezone offset for display
  // Israel timezone: UTC+2 (winter) or UTC+3 (summer)
  time_t utcTime = startTime;
  struct tm* utcTm = gmtime(&utcTime);
  
  // Calculate timezone offset (Israel: +2 or +3 hours)
  // Simple approach: assume +3 hours (summer time) for now
  // TODO: Implement proper DST detection
  int timezoneOffset = 3; // 3 hours in summer
  
  time_t localTime = utcTime + (timezoneOffset * 3600); // Convert UTC to local
  
  sprintf(startTimeStr, "%lu", (unsigned long)localTime);
  
  char statusPayload[400]; // Increased buffer size for duration string
  snprintf(statusPayload, sizeof(statusPayload),
    "%.1f,%.1f,%d,%lu,%s,%s,%s,%s,%s,%s,%.1f",
    isnan(temp) ? 0.0 : temp,
    isnan(humidity) ? 0.0 : humidity,
    map(dutyCycle, 0, 255, 0, 100),
    currentRPM,
    systemPower ? "ON" : "OFF",
    heaterButtonState ? "ON" : "OFF",  // heater = button state
    autoMode ? "AUTO" : "MANUAL",
    startTimeStr,  // שימוש במחרוזת שהוכנה
    heaterPower ? "ON" : "OFF",  // heaterState = actual heater state
    durationStr.c_str(),  // duration as formatted string
    getDisplayWeight()
  );
  
  // Debug output for fan speed (suppressed in advanced debug mode)
  if (!advancedDebugMode) {
    Serial.printf("Sending status: temp=%.1f, humidity=%.1f, fan%%=%d, RPM=%lu, dutyCycle=%d, heater=%s, heaterState=%s, duration=%s\n",
                  isnan(temp) ? 0.0 : temp, isnan(humidity) ? 0.0 : humidity, 
                  map(dutyCycle, 0, 255, 0, 100), currentRPM, dutyCycle, 
                  heaterButtonState ? "ON" : "OFF", heaterPower ? "ON" : "OFF", durationStr.c_str());
  }
  
  publishStateString(topicStatus, statusPayload, false);
}

// ================= Button class =================
class DebouncedButton {
  public:
    DebouncedButton(const int pin, const char* pubTopic, const char* setTopic)
      : pin(pin), pubTopic(pubTopic), setTopic(setTopic),
        lastPhysical(HIGH), lastStable(false), lastDebounce(0) {}

    void begin() { pinMode(pin, INPUT_PULLUP); }

    void update() {
      int raw = digitalRead(pin);
      unsigned long now = millis();
      if (raw != lastPhysical) {
        lastDebounce = now;
        lastPhysical = raw;
      }
      if (now - lastDebounce >= DEBOUNCE_MS) {
        bool pressed = (raw == LOW);
        if (pressed && !lastStable) { onPressed(); }
        lastStable = pressed;
      }
    }

    void onPressed() {
      if (pin == BTN_SYS_PIN) {
        systemPower = !systemPower;
        if (!systemPower) {
          setHeaterState(false);
          setMainFanSpeed(FAN_DUTY_MIN); // Set fan to minimum when power off
          Serial.println("Power OFF - Fan set to minimum speed");
        } else {
          // When power ON, don't automatically turn on heater
          // Let the system decide based on mode and conditions
          Serial.println("Power ON - System ready");
          // Reset heater button state to ON when power is turned on
          heaterButtonState = true;
        }
        publishSystemState();
        publishHeaterState();
      } else if (pin == BTN_HEATER_PIN) {
        Serial.printf("Heater button pressed - systemPower: %d, heaterPower: %d, autoMode: %d\n", 
                      systemPower, heaterPower, autoMode);
        
        if (!systemPower) {
          systemPower = true;
          heaterButtonState = true;
          setHeaterState(true);
          Serial.println("System was off, turning on system and heater");
        } else {
          // Always toggle the heater button state
          heaterButtonState = !heaterButtonState;
          Serial.printf("Heater button toggled to %s\n", heaterButtonState ? "ON" : "OFF");
          
          if (!autoMode) {
            // Manual mode - direct control
            setHeaterState(heaterButtonState);
            Serial.printf("Manual mode: Heater set to %s\n", heaterButtonState ? "ON" : "OFF");
          } else {
            // Auto mode - button state affects auto control
            if (heaterButtonState) {
              Serial.println("Auto mode: Heater button ON - will heat if conditions require");
            } else {
              Serial.println("Auto mode: Heater button OFF - will not heat even if conditions require");
              // If button is OFF, turn off heater immediately
              if (heaterPower) {
                setHeaterState(false);
                Serial.println("Auto mode: Turning heater OFF (button is OFF)");
              }
            }
          }
        }
        publishSystemState();
        publishHeaterState();
      } else if (pin == BTN_MODE_PIN) {
        autoMode = !autoMode;
        
        if (!autoMode) {
          // When switching to manual mode, turn on heater if button is ON
          if (heaterButtonState && !heaterPower) {
            setHeaterState(true);
            Serial.println("Switched to manual mode - turning heater ON (button is ON)");
          } else if (!heaterButtonState) {
            Serial.println("Switched to manual mode - heater button is OFF, heater stays OFF");
          }
        } else {
          // When switching to auto mode, let auto control handle the heater
          Serial.println("Switched to auto mode - heater will be controlled by conditions");
          // Force auto control to run immediately to check current conditions
          autoControl();
        }
        
        publishModeState();
        publishHeaterState();
      }
      Serial.printf("Button pressed pin %d -> S:%d H:%d M:%d\n", pin, systemPower, heaterPower, autoMode);
    }

    void handleSetPayload(const String& payload) {
      String s = payload;
      s.toUpperCase();
      if (pin == BTN_SYS_PIN) {
        if (s == "ON" ||
            s == "1" ||
            s == "TRUE") {
          systemPower = true;
          // Reset heater button state to ON when power is turned on
          heaterButtonState = true;
        } else if (s == "OFF" ||
                   s == "0" ||
                   s == "FALSE") {
          systemPower = false;
          setHeaterState(false);
          setMainFanSpeed(FAN_DUTY_MIN);
        }
        publishSystemState();
        publishHeaterState();
      } else if (pin == BTN_HEATER_PIN) {
        if (s == "ON" ||
            s == "1" ||
            s == "TRUE") {
          // Always toggle the heater button state (like physical button)
          heaterButtonState = true;
          if (!systemPower) {
            systemPower = true;
          }
          // In manual mode, directly control heater
          if (!autoMode) {
            setHeaterState(true);
          }
        } else if (s == "OFF" ||
                   s == "0" ||
                   s == "FALSE") {
          // Always toggle the heater button state (like physical button)
          heaterButtonState = false;
          // In auto mode, turn off heater immediately if button is OFF
          if (autoMode && heaterPower) {
            setHeaterState(false);
          } else if (!autoMode) {
            setHeaterState(false);
          }
        }
        publishSystemState();
        publishHeaterState();
      } else if (pin == BTN_MODE_PIN) {
        if (s == "AUTO" ||
            s == "1" ||
            s == "ON") {
          autoMode = true;
        } else if (s == "MANUAL" ||
                   s == "0" ||
                   s == "OFF") {
          autoMode = false;
          // When switching to manual mode, turn on heater if button is ON
          if (heaterButtonState && !heaterPower) {
            setHeaterState(true);
          }
        }
        publishModeState();
        publishHeaterState();
      }
    }

    void publishSystemState() { publishStateString(topicSystemStatePub, systemPower ? "ON" : "OFF"); }
    void publishModeState() { publishStateString(topicModePub, autoMode ? "AUTO" : "MANUAL"); }

  private:
    int pin;
    const char* pubTopic;
    const char* setTopic;
    int lastPhysical;
    bool lastStable;
    unsigned long lastDebounce;
};

// instantiate buttons
DebouncedButton btnSys(BTN_SYS_PIN, topicSystemStatePub, topicSystemStateSet);
DebouncedButton btnHeater(BTN_HEATER_PIN, topicHeaterStatePub, topicHeaterStateSet);
DebouncedButton btnMode(BTN_MODE_PIN, topicModePub, topicModeSet);

// ================= MQTT callback =================
void mqttCallback(char* topic, byte* payload, unsigned int len) {
  String msg = String((char*)payload).substring(0, len);
  Serial.printf("MQTT recv %s -> %s\n", topic, msg.c_str());

  // NEW: Dynamic temperature profile from HA
  if (String(topic) == topicSetProfile) {
    StaticJsonDocument<200> doc;
    DeserializationError err = deserializeJson(doc, msg);
    if (!err) {
      float oldMin = tempMin;
      float oldMax = tempMax;
      tempMin = doc["min"] | tempMin;
      tempMax = doc["max"] | tempMax;
      Serial.printf("Updated temp profile via MQTT: min=%.1f->%.1f, max=%.1f->%.1f\n", 
                    oldMin, tempMin, oldMax, tempMax);
      // Also save to preferences for persistence
      prefs.begin("biltong", false);
      prefs.putFloat("tempMin", tempMin);
      prefs.putFloat("tempMax", tempMax);
      prefs.end();
    }
  }

  // NEW: Tare command for load cell from HA
  if (String(topic) == topicTareWeightCmd) {
    if (scale.is_ready()) {
      scale.tare();
      currentWeight = 0.0;
      weightFirstReading = true;  // next reading re-initializes smoothed value
      hxOffset = scale.get_offset();
      saveHxOffsetScale();
      addDebugMessage("Load cell tared via MQTT command");
      // Optionally send updated status to HA
      sendAllStatus();
    } else {
      addDebugMessage("Tare command received but load cell not ready");
    }
    return;
  }

  // NEW: HX711 calibration - zero (empty scale)
  if (String(topic) == topicCalibZeroCmd) {
    if (scale.is_ready()) {
      long off = scale.read_average(20);
      scale.set_offset(off);
      hxOffset = off;
      hxCalLoaded = true;
      saveHxOffsetScale();
      currentWeight = 0.0;
      weightFirstReading = true;
      addDebugMessage("HX711 calibrated ZERO: offset=" + String(hxOffset));
      sendAllStatus();
    } else {
      addDebugMessage("Calib ZERO received but load cell not ready");
    }
    return;
  }

  // NEW: HX711 calibration - known weight (payload grams, e.g. '1584')
  if (String(topic) == topicCalibWeightCmd) {
    float grams = msg.toFloat();
    if (grams <= 0.0f) {
      addDebugMessage("Calib WEIGHT invalid payload (grams): " + msg);
      return;
    }
    if (!scale.is_ready()) {
      addDebugMessage("Calib WEIGHT received but load cell not ready");
      return;
    }
    long reading = scale.read_average(20);
    long off = scale.get_offset();
    long diff = reading - off;
    if (diff == 0) {
      addDebugMessage("Calib WEIGHT failed: diff=0 (did you calibrate zero first?)");
      return;
    }
    float newScale = (float)diff / grams;
    if (newScale <= 0.0f) {
      addDebugMessage("Calib WEIGHT failed: computed scale <= 0");
      return;
    }
    scale.set_scale(newScale);
    hxScale = newScale;
    hxOffset = off;
    hxCalLoaded = true;
    saveHxOffsetScale();
    currentWeight = 0.0;
    weightFirstReading = true;
    addDebugMessage("HX711 calibrated WEIGHT: grams=" + String(grams, 1) + ", scale=" + String(hxScale, 6) + ", offset=" + String(hxOffset));
    sendAllStatus();
    return;
  }

  if (String(topic) == topicSet) {
    time_t ts = strtoul(msg.c_str(), nullptr, 10);
    if (ts > 0) {
      startTime = ts;
      systemStartMillis = millis();
      prefs.begin("biltong", false);
      prefs.putULong("startTime", startTime);
      prefs.putULong("systemStartMillis", systemStartMillis);
      prefs.end();
      Serial.printf("Time synchronized and saved: %lu (system millis: %lu)\n", startTime, systemStartMillis);
      timeRequestSent = false;
      initialTimeSyncDone = true;
      powerLossReset = false;
      
      // Immediately publish the new start time to HA via status
      sendAllStatus();
    }
    return;
  }
  if (String(topic) == topicSystemStateSet) { btnSys.handleSetPayload(msg); return; }
  if (String(topic) == topicHeaterStateSet) { btnHeater.handleSetPayload(msg); return; }
  if (String(topic) == topicModeSet) { btnMode.handleSetPayload(msg); return; }
}

// ================= WiFi/MQTT setup =================
void setupWiFi() { 
  WiFi.persistent(true); 
  WiFi.mode(WIFI_STA); 
  WiFi.begin(ssid, password); 
  addDebugMessage("WiFi connection started to: " + String(ssid));
}

void setupMQTT() { 
  mqtt.setServer(mqttServer, mqttPort); 
  mqtt.setCallback(mqttCallback); 
  addDebugMessage("MQTT server set to: " + String(mqttServer) + ":" + String(mqttPort));
}

bool connectMQTT() {
  if (!WiFi.isConnected()) return false;
  if (mqtt.connected()) return true;
  
  String clientId = "esp32-biltong-" + String(FIRMWARE_VERSION);
  if (mqtt.connect(clientId.c_str(), mqttUser, mqttPassword)) {
    Serial.printf("MQTT connected as %s\n", clientId.c_str());
    addDebugMessage("MQTT connected as: " + clientId);
    mqtt.subscribe(topicSet);
    mqtt.subscribe(topicSystemStateSet);
    mqtt.subscribe(topicHeaterStateSet);
    mqtt.subscribe(topicModeSet);
    mqtt.subscribe(topicSetProfile);
    mqtt.subscribe(topicTareWeightCmd);
    mqtt.subscribe(topicCalibZeroCmd);
    mqtt.subscribe(topicCalibWeightCmd);
    
    // Only publish initial states and alert on first connection after startup
    if (mqttInitialConnection) {
      publishStateString(topicSystemStatePub, systemPower ? "ON" : "OFF", true);
      publishStateString(topicHeaterStatePub, heaterButtonState ? "ON" : "OFF", true);
      publishStateString(topicModePub, autoMode ? "AUTO" : "MANUAL", true);
      // Send initial "OK" status only on first connection
      publishStateString(topicAlertPub, "System started - OK", false);
      mqttInitialConnection = false;
      Serial.println("Initial MQTT connection - published startup states");
      addDebugMessage("Initial MQTT connection - published startup states");
    } else {
      // Reconnection - only publish current states without alert
      publishStateString(topicSystemStatePub, systemPower ? "ON" : "OFF", true);
      publishStateString(topicHeaterStatePub, heaterButtonState ? "ON" : "OFF", true);
      publishStateString(topicModePub, autoMode ? "AUTO" : "MANUAL", true);
      Serial.println("MQTT reconnected - published current states");
    }
    
    // Reset failure tracking on successful connection
    mqttConnectionFailed = false;
    
    // If this is after power loss, send status immediately after requesting time
    if (powerLossReset) {
      Serial.println("Power loss reset detected - will send status after time sync");
    }
    
    requestTimeFromHA();
    return true;
  }
  
  // Track connection failures with cooldown
  unsigned long currentTime = millis();
  if (!mqttConnectionFailed || (currentTime - lastMqttFailureTime > MQTT_FAILURE_COOLDOWN)) {
    Serial.println("MQTT connection failed, retrying...");
    mqttConnectionFailed = true;
    lastMqttFailureTime = currentTime;
  }
  
  return false;
}

// ================= OTA and Web Server Functions =================
void setupOTA() {
  ArduinoOTA.setHostname(otaHostname);
  ArduinoOTA.setPassword(otaPassword);
  
  ArduinoOTA.onStart([]() {
    String type = (ArduinoOTA.getCommand() == U_FLASH) ? "sketch" : "filesystem";
    Serial.println("Start updating " + type);
    addDebugMessage("OTA Update started: " + type);
  });
  
  ArduinoOTA.onEnd([]() {
    Serial.println("\nEnd");
    addDebugMessage("OTA Update completed");
  });
  
  ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
    Serial.printf("Progress: %u%%\r", (progress / (total / 100)));
  });
  
  ArduinoOTA.onError([](ota_error_t error) {
    Serial.printf("Error[%u]: ", error);
    String errorMsg = "OTA Error: ";
    if (error == OTA_AUTH_ERROR) errorMsg += "Auth Failed";
    else if (error == OTA_BEGIN_ERROR) errorMsg += "Begin Failed";
    else if (error == OTA_CONNECT_ERROR) errorMsg += "Connect Failed";
    else if (error == OTA_RECEIVE_ERROR) errorMsg += "Receive Failed";
    else if (error == OTA_END_ERROR) errorMsg += "End Failed";
    addDebugMessage(errorMsg);
  });
  
  ArduinoOTA.begin();
  Serial.println("OTA Ready");
}

void addDebugMessage(String message) {
  if (serialRedirectEnabled) {
    String timestamp = String(millis() / 1000) + "s";
    String fullMessage = "[" + timestamp + "] " + message;
    
    debugMessages += fullMessage + "\n";
    
    // Keep only last MAX_DEBUG_MESSAGES lines
    int lineCount = 0;
    for (int i = 0; i < debugMessages.length(); i++) {
      if (debugMessages.charAt(i) == '\n') lineCount++;
    }
    
    if (lineCount > MAX_DEBUG_MESSAGES) {
      int firstNewline = debugMessages.indexOf('\n');
      if (firstNewline != -1) {
        debugMessages = debugMessages.substring(firstNewline + 1);
      }
    }
  } else {
    Serial.println(message);
  }
}

void setupWebServer() {
  // Serve main page
  webServer.on("/", HTTP_GET, []() {
    if (!webServer.authenticate(webUsername, webPassword)) {
      return webServer.requestAuthentication();
    }
    
    String html = "<!DOCTYPE html><html><head><title>Biltong Controller</title>";
    html += "<meta charset='UTF-8'>";
    html += "<meta name='viewport' content='width=device-width, initial-scale=1.0'>";
    html += "<style>";
    html += "body { font-family: Arial, sans-serif; margin: 20px; background-color: #f0f0f0; }";
    html += ".container { max-width: 800px; margin: 0 auto; background: white; padding: 20px; border-radius: 10px; box-shadow: 0 2px 10px rgba(0,0,0,0.1); }";
    html += "h1 { color: #333; text-align: center; }";
    html += ".status { background: #e8f5e8; padding: 15px; border-radius: 5px; margin: 10px 0; }";
    html += ".controls { margin: 20px 0; }";
    html += "button { background: #007bff; color: white; border: none; padding: 10px 20px; margin: 5px; border-radius: 5px; cursor: pointer; }";
    html += "button:hover { background: #0056b3; }";
    html += ".danger { background: #dc3545; }";
    html += ".danger:hover { background: #c82333; }";
    html += ".success { background: #28a745; }";
    html += ".success:hover { background: #218838; }";
    html += ".debug { background: #f8f9fa; border: 1px solid #dee2e6; padding: 15px; border-radius: 5px; margin: 10px 0; max-height: 400px; overflow-y: auto; }";
    html += ".debug pre { margin: 0; white-space: pre-wrap; font-size: 12px; }";
    html += "</style></head><body>";
    html += "<div class='container'>";
    html += "<h1>🍖 Biltong Controller v" + String(FIRMWARE_VERSION) + "</h1>";
    
    // System Status
    html += "<div class='status'>";
    html += "<h3>System Status</h3>";
    html += "<p><strong>Power:</strong> " + String(systemPower ? "ON" : "OFF") + "</p>";
    html += "<p><strong>Heater:</strong> " + String(heaterPower ? "ON" : "OFF") + " (Button: " + String(heaterButtonState ? "ON" : "OFF") + ")</p>";
    html += "<p><strong>Mode:</strong> " + String(autoMode ? "AUTO" : "MANUAL") + "</p>";
    html += "<p><strong>Temperature:</strong> " + String(temperature, 1) + "°C</p>";
    html += "<p><strong>Humidity:</strong> " + String(humidity, 1) + "%</p>";
    html += "<p><strong>Fan Speed:</strong> " + String(dutyCycle) + "/255</p>";
    html += "<p><strong>RPM:</strong> " + String(rpm) + "</p>";
    html += "<p><strong>Weight:</strong> " + String(getDisplayWeight(), 1) + " g</p>";
    html += "<p><strong>HX711 scale:</strong> " + String(hxScale, 6) + "</p>";
    html += "<p><strong>HX711 offset:</strong> " + String(hxOffset) + "</p>";
    html += "</div>";
    
    // Controls
    html += "<div class='controls'>";
    html += "<h3>Controls</h3>";
    html += "<button onclick='toggleSerial()'>Toggle Serial Output</button>";
    html += "<button onclick='clearDebug()'>Clear Debug Messages</button>";
    html += "<button onclick='tareWeight()' class='success'>Tare Weight</button>";
    html += "<button onclick='calibZero()' class='success'>Calibrate ZERO</button>";
    html += "<button onclick='calibWeight()' class='success'>Calibrate WEIGHT</button>";
    html += "<button onclick='restart()' class='danger'>Restart System</button>";
    html += "</div>";
    
    // Debug Messages
    html += "<div class='debug'>";
    html += "<h3>Debug Messages</h3>";
    html += "<pre id='debugContent'>" + debugMessages + "</pre>";
    html += "</div>";
    
    html += "</div>";
    
    // JavaScript
    html += "<script>";
    html += "function toggleSerial() { fetch('/api/toggle-serial').then(() => location.reload()); }";
    html += "function clearDebug() { fetch('/api/clear-debug').then(() => location.reload()); }";
    html += "function tareWeight() { fetch('/api/tare-weight', {method:'POST'}).then(() => location.reload()); }";
    html += "function calibZero() { fetch('/api/calib-zero', {method:'POST'}).then(() => location.reload()); }";
    html += "function calibWeight() { const g = prompt('Enter known weight in grams (e.g. 1584):'); if(!g) return; fetch('/api/calib-weight?grams=' + encodeURIComponent(g), {method:'POST'}).then(() => location.reload()); }";
    html += "function restart() { if(confirm('Are you sure?')) fetch('/api/restart').then(() => location.reload()); }";
    html += "setInterval(() => { fetch('/api/debug').then(r => r.text()).then(t => document.getElementById('debugContent').textContent = t); }, 2000);";
    html += "</script>";
    
    html += "</body></html>";
    
    webServer.send(200, "text/html", html);
  });
  
  // API endpoints
  webServer.on("/api/debug", HTTP_GET, []() {
    webServer.send(200, "text/plain", debugMessages);
  });
  
  webServer.on("/api/toggle-serial", HTTP_POST, []() {
    serialRedirectEnabled = !serialRedirectEnabled;
    addDebugMessage("Serial redirect " + String(serialRedirectEnabled ? "enabled" : "disabled"));
    webServer.send(200, "text/plain", "OK");
  });
  
  webServer.on("/api/clear-debug", HTTP_POST, []() {
    debugMessages = "";
    webServer.send(200, "text/plain", "OK");
    // Don't add debug message here to avoid immediate refill
  });
  
  webServer.on("/api/restart", HTTP_POST, []() {
    addDebugMessage("System restart requested");
    webServer.send(200, "text/plain", "Restarting...");
    delay(1000);
    ESP.restart();
  });

  webServer.on("/api/tare-weight", HTTP_POST, []() {
    if (scale.is_ready()) {
      scale.tare();
      currentWeight = 0.0;
      weightFirstReading = true;
      hxOffset = scale.get_offset();
      saveHxOffsetScale();
      addDebugMessage("Load cell tared via Web UI");
      webServer.send(200, "text/plain", "OK");
    } else {
      addDebugMessage("Web UI tare requested but load cell not ready");
      webServer.send(503, "text/plain", "Load cell not ready");
    }
  });

  webServer.on("/api/calib-zero", HTTP_POST, []() {
    if (!scale.is_ready()) {
      addDebugMessage("Web UI calib ZERO requested but load cell not ready");
      webServer.send(503, "text/plain", "Load cell not ready");
      return;
    }
    long off = scale.read_average(20);
    scale.set_offset(off);
    hxOffset = off;
    hxCalLoaded = true;
    saveHxOffsetScale();
    currentWeight = 0.0;
    weightFirstReading = true;
    addDebugMessage("HX711 calibrated ZERO via Web UI: offset=" + String(hxOffset));
    webServer.send(200, "text/plain", "OK");
  });

  webServer.on("/api/calib-weight", HTTP_POST, []() {
    if (!webServer.hasArg("grams")) {
      webServer.send(400, "text/plain", "Missing grams");
      return;
    }
    float grams = webServer.arg("grams").toFloat();
    if (grams <= 0.0f) {
      webServer.send(400, "text/plain", "Invalid grams");
      return;
    }
    if (!scale.is_ready()) {
      addDebugMessage("Web UI calib WEIGHT requested but load cell not ready");
      webServer.send(503, "text/plain", "Load cell not ready");
      return;
    }
    long reading = scale.read_average(20);
    long off = scale.get_offset();
    long diff = reading - off;
    if (diff == 0) {
      webServer.send(400, "text/plain", "diff=0; calibrate zero first");
      return;
    }
    float newScale = (float)diff / grams;
    if (newScale <= 0.0f) {
      webServer.send(400, "text/plain", "scale<=0");
      return;
    }
    scale.set_scale(newScale);
    hxScale = newScale;
    hxOffset = off;
    hxCalLoaded = true;
    saveHxOffsetScale();
    currentWeight = 0.0;
    weightFirstReading = true;
    addDebugMessage("HX711 calibrated WEIGHT via Web UI: grams=" + String(grams, 1) + ", scale=" + String(hxScale, 6) + ", offset=" + String(hxOffset));
    webServer.send(200, "text/plain", "OK");
  });
  
  webServer.begin();
  Serial.println("Web server started on port " + String(webServerPort));
  addDebugMessage("Web server started on port " + String(webServerPort));
}

// ================= Setup all hardware =================
String getResetReasonText(esp_reset_reason_t reason) {
  switch (reason) {
    case ESP_RST_POWERON:   return "Power-on reset";
    case ESP_RST_EXT:       return "External reset";
    case ESP_RST_SW:        return "Software reset";
    case ESP_RST_PANIC:     return "Exception/panic reset";
    case ESP_RST_INT_WDT:   return "Interrupt watchdog reset";
    case ESP_RST_TASK_WDT:  return "Task watchdog reset";
    case ESP_RST_WDT:       return "Other watchdog reset";
    case ESP_RST_DEEPSLEEP: return "Deep sleep reset";
    case ESP_RST_BROWNOUT:  return "Brownout reset";
    case ESP_RST_SDIO:      return "SDIO reset";
    default:                return "Unknown reset (" + String(reason) + ")";
  }
}

void setupAll() {
  // Always show firmware version on startup
  Serial.printf("Biltong Controller v%s starting...\n", FIRMWARE_VERSION);
  addDebugMessage("Biltong Controller v" + String(FIRMWARE_VERSION) + " starting...");
  
  // Check for power loss reset - be more inclusive
  esp_reset_reason_t resetReason = esp_reset_reason();
  String resetText = getResetReasonText(resetReason);
  Serial.printf("Reset reason: %d (%s)\n", resetReason, resetText.c_str());
  addDebugMessage("Reset reason: " + resetText);
  
  // Consider any reset except ESP_RST_SW (software reset) as power loss
  if (resetReason != ESP_RST_SW && resetReason != ESP_RST_DEEPSLEEP) {
    powerLossReset = true;
    Serial.printf("Power loss reset detected (reason: %d)\n", resetReason);
    addDebugMessage("Power loss reset detected: " + resetText);
  } else {
    Serial.printf("Software reset detected (reason: %d)\n", resetReason);
    addDebugMessage("Software reset detected: " + resetText);
  }
  
  // Set timezone to Israel (UTC+2/UTC+3)
  setenv("TZ", "IST-2IDT,M3.5.0,M10.5.0", 1);
  tzset();
  
  // Initialize preferences and handle time based on reset reason
  prefs.begin("biltong", false);
  
  if (powerLossReset) {
    // After power loss - clear old time and request new from HA
    startTime = 0;
    systemStartMillis = millis();
    initialTimeSyncDone = false;
    Serial.println("Power loss detected - cleared start time, will request from HA");
    // Clear saved time from preferences and save the changes
    prefs.remove("startTime");
    prefs.remove("systemStartMillis");
  } else {
    // Normal boot - try to restore time from preferences
    startTime = prefs.getULong("startTime", 0);
    systemStartMillis = prefs.getULong("systemStartMillis", millis());
    if (startTime > 0) {
      initialTimeSyncDone = true;
      Serial.printf("Restored start time from preferences: %lu\n", startTime);
    } else {
      Serial.println("No valid start time in preferences");
      initialTimeSyncDone = false;
    }
  }
  
  tempMin = prefs.getFloat("tempMin", 20.0);
  tempMax = prefs.getFloat("tempMax", 32.0);
  prefs.end();
  
  // Initialize hardware
  Serial.begin(115200);
  dht.begin();

  // Initialize load cell (HX711)
  scale.begin(PIN_HX_DOUT, PIN_HX_SCK);
  // Load saved HX711 calibration (if exists)
  prefs.begin("biltong", false);
  float savedScale = prefs.getFloat("hxScale", -1.0f);
  long savedOffset = prefs.getLong("hxOffset", 0L);
  prefs.end();
  if (savedScale > 0.0f && savedOffset != 0L) {
    hxScale = savedScale;
    hxOffset = savedOffset;
    hxCalLoaded = true;
  } else {
    hxScale = calibration_factor;
    hxOffset = 0;
    hxCalLoaded = false;
  }

  scale.set_scale(hxScale);
  delay(5000); // Warm-up time
  if (hxCalLoaded) {
    scale.set_offset(hxOffset);
    addDebugMessage("Load cell (HX711) initialized with saved calibration (scale/offset)");
  } else {
    scale.tare();
    hxOffset = scale.get_offset();
    saveHxOffsetScale(); // save offset so empty reading is stable after reboot
    addDebugMessage("Load cell (HX711) initialized and tared (no saved calibration)");
  }
  
  // Initialize LCD - DISABLED
  // int lcdStatus = lcd.begin(20, 4);
  // if (lcdStatus != 0) {
  //   Serial.printf("LCD initialization failed: %d\n", lcdStatus);
  // }
  
  // Initialize PWM for fan control
  // ledcAttach(PIN_PWM_FAN, 25000, 8); // 25kHz PWM, 8-bit resolution // change to support the CPP
  ledcSetup(0, 25000, 8);           // channel 0, 25kHz, 8-bit resolution
  ledcAttachPin(PIN_PWM_FAN, 0);   // חיבר את PIN_PWM_FAN ל‑channel 0


  setMainFanSpeed(FAN_DUTY_MIN);
  
  
  // Initialize buttons
  btnSys.begin();
  btnHeater.begin();
  btnMode.begin();
  
  // Initialize tachometer interrupt
  pinMode(PIN_TACH, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(PIN_TACH), tachISR, RISING);
  
  // Initialize heater pins (even if hardware not present)
  pinMode(PIN_HEATER, OUTPUT);
  pinMode(PIN_HEATER_FAN, OUTPUT);
  digitalWrite(PIN_HEATER, HIGH); // HIGH = כבוי
  digitalWrite(PIN_HEATER_FAN, HIGH); // HIGH = כבוי
  
  // Setup WiFi and MQTT
  setupWiFi();
  setupMQTT();
  
  // Setup OTA and Web Server
  setupOTA();
  setupWebServer();
  
  Serial.println("Setup complete");
  Serial.printf("Current startTime value: %lu\n", startTime);
}

// ================= Arduino setup function =================
void setup() {
  setupAll();
}
 
// ================= Main loop =================
void loop() {

   // Handle OTA updates
  ArduinoOTA.handle();
  
  // Handle web server requests
  webServer.handleClient();

  // WiFi connection handling
  if (!WiFi.isConnected()) {
    Serial.println("WiFi disconnected, reconnecting...");
    addDebugMessage("WiFi disconnected, reconnecting...");
    setupWiFi();
    delay(5000);
    return;
  }
  
  // MQTT connection handling
  if (!connectMQTT()) {
    Serial.println("MQTT connection failed, retrying...");
    addDebugMessage("MQTT connection failed, retrying...");
    delay(5000);
    return;
  }
  
  mqtt.loop();
  
  
  // Handle serial commands
  handleSerialCommands();
  
  // Advanced debug logging
  logAdvancedDebug();
  
  // Update buttons
  btnSys.update();
  btnHeater.update();
  btnMode.update();
  
  // Update weight in main loop (often) so EMA converges quickly when user adds load
  if (scale.is_ready()) {
    float raw = scale.get_units(WEIGHT_SAMPLES) * WEIGHT_SCALE_FACTOR;
    if (weightFirstReading) {
      currentWeight = raw;
      weightFirstReading = false;
    } else {
      float drop = currentWeight - raw;
      if (drop > WEIGHT_OUTLIER_G) {
        currentWeight = 0.97f * currentWeight + 0.03f * raw;
      } else {
        // Big increase (weight added): use fast alpha so we settle in few readings
        float rise = raw - currentWeight;
        float alpha = (rise > WEIGHT_RISE_FAST_G) ? WEIGHT_EMA_ALPHA_FAST : WEIGHT_EMA_ALPHA;
        currentWeight = alpha * currentWeight + (1.0f - alpha) * raw;
      }
    }
  }

  // Add periodic debug message every 30 seconds
  static unsigned long lastDebugHeartbeat = 0;
  if (millis() - lastDebugHeartbeat >= 30000) {
    addDebugMessage("System running - Temp: " + String(temperature, 1) + "°C, Humidity: " + String(humidity, 1) + "%, RPM: " + String(rpm) + ", Weight: " + String(getDisplayWeight(), 1) + " g");
    lastDebugHeartbeat = millis();
  }
  
  // Run automatic control
  autoControl();
  
  // Check heater fan delay
  if (heaterFanDelayedOff && (millis() - heaterFanOffTime >= HEATER_FAN_DELAY_MS)) {
    digitalWrite(PIN_HEATER_FAN, HIGH);
    heaterFanDelayedOff = false;
    Serial.println("Heater fan turned OFF after 30 second delay");
  }
  
  // Send status periodically
  static unsigned long lastStatusUpdate = 0;
  if (millis() - lastStatusUpdate >= 30000) { // Every 30 seconds
    sendAllStatus();
    lastStatusUpdate = millis();
  }
  
  delay(100);
}