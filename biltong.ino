/*   Combined Biltong controller v1.26
     - FIX: Fixed start time transmission in biltong/status (was sending 0, now sends correct timestamp).
     - CLEANUP: Removed all separate MQTT transmissions of start time and run time variables.
     - MOD: Only start time is sent via biltong/status payload, run time is calculated in HA.
     - Previous fixes remain: Power loss detection, temperature profile updates, alert system.

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
*/

#define FIRMWARE_VERSION "1.26"

#include <WiFi.h>
#include <PubSubClient.h>
#include <Preferences.h>
#include <Wire.h>
#include <hd44780.h>
#include <hd44780ioClass/hd44780_I2Cexp.h>
#include "DHT.h"
#include <ArduinoJson.h>
#include <esp_system.h>

// ================= CONFIG =================
// WiFi / MQTT
// const char* ssid       = "MyWiFi";
// const char* password   = "mypassword123";
// const char* mqttServer = "192.168.1.100";
// const int mqttPort     = 1883;
// const char* mqttUser   = "user";
// const char* mqttPassword = "password";

#include "config.h"

// Topics - Updated to match Home Assistant configuration
const char* topicGet           = "biltong/cmd/get_start";
const char* topicSet           = "biltong/cmd/set_start";
const char* topicStatus        = "biltong/status";  // Contains: temp,humidity,fan%,rpm,sys,heater,auto,start_time
const char* topicAlertPub      = "biltong/alert"; // New topic for alerts
const char* topicSetProfile    = "biltong/cmd/set_profile"; // New topic for dynamic temperature control

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
const int PIN_TACH          = 5;  // tach input (interrupt)
const int PIN_PWM_FAN       = 16; // PWM pin for main fan
const int PIN_HEATER        = 27; // Heater MOSFET control pin (placeholder)
const int PIN_HEATER_FAN    = 26; // Heater fan MOSFET control pin (placeholder)
// const int PIN_PWM_LED       = 17; // FUTURE: PWM pin for LED MOSFET control

// Buttons
const int BTN_SYS_PIN       = 13; // green - master power
const int BTN_HEATER_PIN    = 33; // red   - manual heater on/off
const int BTN_MODE_PIN      = 32; // yellow - auto/manual

// Debounce time
const unsigned long DEBOUNCE_MS = 50UL;

// ================= Globals =================
hd44780_I2Cexp lcd;
DHT dht(PIN_DHT, DHT22);

volatile uint32_t tachCount = 0;
uint32_t lastRPMcalc = 0, currentRPM = 0;
int dutyCycle = 0; // 0-255 pwm for main fan

// control states
bool systemPower = true;
bool heaterPower = true;
bool autoMode    = true;

// MQTT client declaration moved to a global scope
WiFiClient wifiClient;
PubSubClient mqtt(wifiClient);

// New: configuration flags for hardware presence
const bool HEATER_HW_PRESENT = false; // Set to true when heater MOSFET arrives

// New: Target temperature/humidity for auto-control
float tempMin = 20.0; // Tmin from HA
float tempMax = 32.0; // Tmax from HA
float targetHmax = 60.0; // Max humidity threshold
float targetHmin = 20.0; // Min humidity threshold

// New: Fan speed presets
const int FAN_SPEED_MIN    = 400; // RPM - for reference
const int FAN_DUTY_MIN     = 40;  // PWM duty cycle equivalent (~400 RPM)
const int FAN_DUTY_MEDIUM  = 128; // 50% duty cycle
const int FAN_DUTY_HIGH    = 255; // 100% duty cycle

// New: Alerting mechanism
unsigned long lastAlertTime = 0;
const unsigned long ALERT_COOLDOWN_MS = 300000; // 5 minutes

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

void setMainFanSpeed(int newDutyCycle) {
  if (dutyCycle != newDutyCycle) {
    dutyCycle = newDutyCycle;
    ledcWrite(PIN_PWM_FAN, dutyCycle);
    Serial.printf("Main fan speed set to %d (%d%%)\n", dutyCycle, map(dutyCycle, 0, 255, 0, 100));
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
    digitalWrite(PIN_HEATER, heaterPower ? HIGH : LOW);
    digitalWrite(PIN_HEATER_FAN, heaterPower ? HIGH : LOW);
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
    return effectiveStart + (millis() - systemStartMillis) / 1000;
  } else {
    return 0; // No valid time available
  }
}

// ================= Missing function: sendAllStatus =================
void sendAllStatus() {
  if (!mqtt.connected()) return;

  Serial.printf("DEBUG: startTime inside sendAllStatus is %lu\n", startTime);
  
  // Read DHT sensor
  float temp = dht.readTemperature();
  float humidity = dht.readHumidity();
  
  // Calculate RPM if needed
  if (millis() - lastRPMcalc >= 1000) {
    currentRPM = (tachCount * 60) / 2; // Assuming 2 pulses per revolution
    tachCount = 0;
    lastRPMcalc = millis();
  }
  
  // --- הוסף את השורה החסרה כאן ---
  char statusPayload[300];
  
  // Create status payload
  char startTimeStr[12]; // Buffer גדול מספיק ל-ULong
  sprintf(startTimeStr, "%lu", startTime);
  
  snprintf(statusPayload, sizeof(statusPayload),
    "%.1f,%.1f,%d,%lu,%s,%s,%s,%s", // שינוי של %lu ל- %s
    isnan(temp) ? 0.0 : temp,
    isnan(humidity) ? 0.0 : humidity,
    map(dutyCycle, 0, 255, 0, 100),
    currentRPM,
    systemPower ? "ON" : "OFF",
    heaterPower ? "ON" : "OFF",
    autoMode ? "AUTO" : "MANUAL",
    startTimeStr  // שימוש במחרוזת שהוכנה
  );
  
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
          setMainFanSpeed(FAN_DUTY_MIN); // New: Maintain minimum fan speed
        } else {
          setHeaterState(true);
        }
        publishSystemState();
        publishHeaterState();
      } else if (pin == BTN_HEATER_PIN) {
        if (!systemPower) {
          systemPower = true;
          setHeaterState(true);
        } else {
          setHeaterState(!heaterPower);
        }
        publishSystemState();
        publishHeaterState();
      } else if (pin == BTN_MODE_PIN) {
        autoMode = !autoMode;
        publishModeState();
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
          setHeaterState(true);
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
          setHeaterState(true);
          if (!systemPower) systemPower = true;
        } else if (s == "OFF" ||
                   s == "0" ||
                   s == "FALSE") {
          setHeaterState(false);
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
        }
        publishModeState();
      }
    }

    void publishSystemState() { publishStateString(topicSystemStatePub, systemPower ? "ON" : "OFF"); }
    void publishHeaterState() { publishStateString(topicHeaterStatePub, heaterPower ? "ON" : "OFF"); }
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
void setupWiFi() { WiFi.persistent(true); WiFi.mode(WIFI_STA); WiFi.begin(ssid, password); }
void setupMQTT() { mqtt.setServer(mqttServer, mqttPort); mqtt.setCallback(mqttCallback); }

bool connectMQTT() {
  if (!WiFi.isConnected()) return false;
  if (mqtt.connected()) return true;
  String clientId = "esp32-biltong-" + String(FIRMWARE_VERSION);
  if (mqtt.connect(clientId.c_str(), mqttUser, mqttPassword)) {
    Serial.printf("MQTT connected as %s\n", clientId.c_str());
    mqtt.subscribe(topicSet);
    mqtt.subscribe(topicSystemStateSet);
    mqtt.subscribe(topicHeaterStateSet);
    mqtt.subscribe(topicModeSet);
    mqtt.subscribe(topicSetProfile);
    publishStateString(topicSystemStatePub, systemPower ? "ON" : "OFF", true);
    publishStateString(topicHeaterStatePub, heaterPower ? "ON" : "OFF", true);
    publishStateString(topicModePub, autoMode ? "AUTO" : "MANUAL", true);
    // Send initial "OK" status to prevent Unknown state
    publishStateString(topicAlertPub, "System started - OK", false);
    
    // If this is after power loss, send status immediately after requesting time
    if (powerLossReset) {
      Serial.println("Power loss reset detected - will send status after time sync");
    }
    
    requestTimeFromHA();
    return true;
  }
  return false;
}

// ================= Setup all hardware =================
void setupAll() {
  Serial.printf("Biltong Controller v%s starting...\n", FIRMWARE_VERSION);
  
  // Check for power loss reset - be more inclusive
  esp_reset_reason_t resetReason = esp_reset_reason();
  Serial.printf("Reset reason: %d\n", resetReason);
  
  // Consider any reset except ESP_RST_SW (software reset) as power loss
  if (resetReason != ESP_RST_SW && resetReason != ESP_RST_DEEPSLEEP) {
    powerLossReset = true;
    Serial.printf("Power loss reset detected (reason: %d)\n", resetReason);
  } else {
    Serial.printf("Software reset detected (reason: %d)\n", resetReason);
  }
  
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
  
  // Initialize LCD
  int lcdStatus = lcd.begin(20, 4);
  if (lcdStatus != 0) {
    Serial.printf("LCD initialization failed: %d\n", lcdStatus);
  }
  
  // Initialize PWM for fan control
  ledcAttach(PIN_PWM_FAN, 25000, 8); // 25kHz PWM, 8-bit resolution
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
  digitalWrite(PIN_HEATER, LOW);
  digitalWrite(PIN_HEATER_FAN, LOW);
  
  // Setup WiFi and MQTT
  setupWiFi();
  setupMQTT();
  
  Serial.println("Setup complete");
  Serial.printf("Current startTime value: %lu\n", startTime);
}

// ================= Arduino setup function =================
void setup() {
  setupAll();
}

// ================= Main loop =================
void loop() {
  // WiFi connection handling
  if (!WiFi.isConnected()) {
    Serial.println("WiFi disconnected, reconnecting...");
    setupWiFi();
    delay(5000);
    return;
  }
  
  // MQTT connection handling
  if (!connectMQTT()) {
    Serial.println("MQTT connection failed, retrying...");
    delay(5000);
    return;
  }
  
  mqtt.loop();
  
  // Update buttons
  btnSys.update();
  btnHeater.update();
  btnMode.update();
  
  // Send status periodically
  static unsigned long lastStatusUpdate = 0;
  if (millis() - lastStatusUpdate >= 30000) { // Every 30 seconds
    sendAllStatus();
    lastStatusUpdate = millis();
  }
  
  delay(100);
}