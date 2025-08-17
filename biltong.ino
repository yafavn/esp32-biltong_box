/*  Combined Biltong controller v1.5
    - Original code integrated with 3 buttons (debounced)
    - Buttons publish states to MQTT and accept set commands from MQTT
    - LCD updated with system/heater/mode states
    - No persistent storage of states after power off (as requested)
    - Added LED MQTT topics structure (hardware pending)
    
    Version History:
    v1.1 - Fixed MQTT topics to match Home Assistant configuration
         - Fixed RPM calculation and display
         - Fixed green button behavior to turn on heater when system starts
    v1.2 - User updated version locally
    v1.3 - Fixed fan duty cycle not resetting when turning system back on
         - Fixed RPM readings with better calculation (pulses * 15)
         - Added proper HA topics for start time and run time
         - Improved system state management for proper fan control
    v1.4 - FIXED: Time and run time now published regardless of date validation
         - System requests time from HA/NTP if not yet synchronized
         - FIXED: Manual mode fan control - duty cycle properly restored when system turns on
         - Improved time synchronization logic with fallback to millis() based timing
    v1.5 - REMOVED: Separate start_time and run_time MQTT topics (data available in status CSV)
         - REMOVED: Unnecessary time validation logic - always sync on startup
         - FIXED: Time synchronization now happens on every ESP32 startup with HA connection
         - Simplified code by removing duplicate time publishing mechanisms
         - Start time and duration now available only through status CSV (index 9)
*/

#define FIRMWARE_VERSION "1.5"

#include <WiFi.h>
#include <PubSubClient.h>
#include <Preferences.h>
#include <Wire.h>
#include <hd44780.h>
#include <hd44780ioClass/hd44780_I2Cexp.h>
#include "DHT.h"

// ================= CONFIG =================
// WiFi / MQTT

// const char* ssid         = "MyWiFi";
// const char* password     = "mypassword123";
// const char* mqttServer   = "192.168.1.100";

#include "config.h"

// Topics - Updated to match Home Assistant configuration
const char* topicGet     = "biltong/cmd/get_start";
const char* topicSet     = "biltong/cmd/set_start";
const char* topicStatus  = "biltong/status";

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

Preferences prefs;
time_t startTime = 0;
bool timeRequestSent = false;  // Track if we've requested time from HA
unsigned long systemStartMillis = 0;  // Track when system was started
bool initialTimeSyncDone = false;  // Track if initial sync completed

// ================= Hardware pins (all const as requested) =================
const int PIN_DHT         = 4;   // DHT data pin
const int PIN_TACH        = 5;   // tach input (interrupt)
const int PIN_PWM_FAN     = 16;  // PWM channel/pin used earlier in your code
// const int PIN_PWM_LED     = 17;  // FUTURE: PWM pin for LED MOSFET control (not implemented yet)

// Buttons (use the pins you used previously / prefer)
const int BTN_SYS_PIN     = 13;  // green - master power 
const int BTN_HEATER_PIN  = 33;  // red   - manual heater on/off 
const int BTN_MODE_PIN    = 32;  // yellow - auto/manual 

// Debounce time
const unsigned long DEBOUNCE_MS = 50UL;

// ================= Globals =================
hd44780_I2Cexp lcd;
DHT dht(PIN_DHT, DHT22);

volatile uint32_t tachCount = 0;
uint32_t lastRPMcalc = 0, currentRPM = 0;
int dutyCycle = 0; // 0-255 pwm for fan
int lastManualDutyCycle = 128; // Remember last manual duty cycle setting

WiFiClient wifiClient;
PubSubClient mqtt(wifiClient);

// control states
bool systemPower = true;   // master on/off
bool heaterPower = true;   // manual heater on/off - start with heater on when system starts
bool autoMode    = true;   // true = automatic control (autoControl()), false = manual

// LED control states (ready for future implementation)
bool ledPower = false;     // LED on/off (MQTT only for now)
int ledBrightness = 128;   // LED brightness (0-255) (MQTT only for now)

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

// ================= Time management functions =================
void requestTimeFromHA() {
  if (mqtt.connected() && !initialTimeSyncDone) {
    mqtt.publish(topicGet, "", false);
    timeRequestSent = true;
    Serial.println("Requested time synchronization from HA on startup");
  }
}

time_t getEffectiveStartTime() {
  if (startTime > 0) {
    return startTime;
  } else {
    // Use system start time based on millis() if no valid timestamp
    return systemStartMillis / 1000;
  }
}

time_t getCurrentTime() {
  time_t effectiveStart = getEffectiveStartTime();
  return effectiveStart + (millis() - systemStartMillis) / 1000;
}

// ================= LED Control Functions (ready for future implementation) =================
void publishLedState() {
  const char* p = ledPower ? "ON" : "OFF";
  publishStateString(topicLedStatePub, p);
  publishStateInt(topicLedBrightnessPub, ledBrightness);
}

// ================= Button class (per-instance debounce & publish) =================
class DebouncedButton {
  public:
    DebouncedButton(const int pin, const char* pubTopic, const char* setTopic)
      : pin(pin), pubTopic(pubTopic), setTopic(setTopic),
        lastPhysical(HIGH), lastStable(false), lastDebounce(0) {}

    void begin() {
      pinMode(pin, INPUT_PULLUP);
      // publish current state immediately (MQTT) if connected later
    }

    // call often in loop()
    void update() {
      int raw = digitalRead(pin);
      unsigned long now = millis();
      if (raw != lastPhysical) {
        lastDebounce = now;
        lastPhysical = raw;
      }
      if (now - lastDebounce >= DEBOUNCE_MS) {
        bool pressed = (raw == LOW); // active LOW
        if (pressed && !lastStable) {
          // falling edge -> a press
          onPressed();
        }
        lastStable = pressed;
      }
    }

    // invoked on a stable press (edge)
    void onPressed() {
      // route to actions by checking which topic/pin
      if (pin == BTN_SYS_PIN) {
        systemPower = !systemPower;
        if (!systemPower) {
          // turning system off -> save current duty cycle if in manual mode
          if (!autoMode) {
            lastManualDutyCycle = dutyCycle;
          }
          // turn off heater and stop fan PWM
          heaterPower = false;
          dutyCycle = 0;
          ledcWrite(PIN_PWM_FAN, dutyCycle);
          Serial.printf("System OFF: dutyCycle set to 0, saved lastManual=%d\n", lastManualDutyCycle);
        } else {
          // FIXED: when turning system ON
          heaterPower = true;
          if (!autoMode) {
            // FIXED: In manual mode, restore previous duty cycle
            dutyCycle = lastManualDutyCycle;
            ledcWrite(PIN_PWM_FAN, dutyCycle);
            Serial.printf("System ON (Manual): Restored dutyCycle to %d\n", dutyCycle);
          } else {
            // In auto mode, let autoControl() handle it
            Serial.printf("System ON (Auto): Forcing auto control recalculation\n");
          }
        }
        publishSystemState();
        publishHeaterState(); // publish both states when green button pressed
      } else if (pin == BTN_HEATER_PIN) {
        if (!systemPower) {
          // if system is off and we press heater button -> turn on whole system
          systemPower = true;
          heaterPower = true;
          if (!autoMode) {
            dutyCycle = lastManualDutyCycle;
            ledcWrite(PIN_PWM_FAN, dutyCycle);
          }
        } else {
          // system is on -> toggle heater
          heaterPower = !heaterPower;
        }
        publishSystemState();
        publishHeaterState();
      } else if (pin == BTN_MODE_PIN) {
        autoMode = !autoMode;
        if (!autoMode && systemPower) {
          // Switching to manual mode - maintain current duty cycle
          lastManualDutyCycle = dutyCycle;
          Serial.printf("Switched to Manual mode: dutyCycle=%d\n", dutyCycle);
        }
        publishModeState();
      }
      // update LCD immediately
      // (LCD updated in main loop displayOnLCD)
      Serial.printf("Button pressed pin %d -> S:%d H:%d M:%d DC:%d\n", pin, systemPower, heaterPower, autoMode, dutyCycle);
    }

    // when MQTT set message arrives for this button, invoke handler
    void handleSetPayload(const String& payload) {
      String s = payload;
      s.toUpperCase();
      if (pin == BTN_SYS_PIN) {
        if (s == "ON" || s == "1" || s == "TRUE") {
          systemPower = true;
          heaterPower = true; // FIXED: when system turns on via MQTT, also turn heater on
          if (!autoMode) {
            // FIXED: In manual mode, restore previous duty cycle
            dutyCycle = lastManualDutyCycle;
            ledcWrite(PIN_PWM_FAN, dutyCycle);
            Serial.printf("System ON via MQTT (Manual): Restored dutyCycle to %d\n", dutyCycle);
          } else {
            Serial.printf("System ON via MQTT (Auto): Forcing auto control recalculation\n");
          }
        } else if (s == "OFF" || s == "0" || s == "FALSE") {
          if (!autoMode) {
            lastManualDutyCycle = dutyCycle;
          }
          systemPower = false;
          heaterPower = false;
          dutyCycle = 0;
          ledcWrite(PIN_PWM_FAN, dutyCycle);
        }
        publishSystemState();
        publishHeaterState(); // publish both states
      } else if (pin == BTN_HEATER_PIN) {
        if (s == "ON" || s == "1" || s == "TRUE") {
          heaterPower = true;
          if (!systemPower) {
            systemPower = true;
            if (!autoMode) {
              dutyCycle = lastManualDutyCycle;
              ledcWrite(PIN_PWM_FAN, dutyCycle);
            }
          }
        } else if (s == "OFF" || s == "0" || s == "FALSE") {
          heaterPower = false;
        }
        publishSystemState();
        publishHeaterState();
      } else if (pin == BTN_MODE_PIN) {
        if (s == "AUTO") {
          autoMode = true;
        } else if (s == "MANUAL") {
          if (autoMode && systemPower) {
            // Switching to manual - save current duty cycle
            lastManualDutyCycle = dutyCycle;
          }
          autoMode = false;
        } else if (s == "1" || s == "ON") {
          autoMode = true;
        } else if (s == "0" || s == "OFF") {
          if (autoMode && systemPower) {
            lastManualDutyCycle = dutyCycle;
          }
          autoMode = false;
        }
        publishModeState();
      }
    }

    // publishers
    void publishSystemState() {
      const char* p = systemPower ? "ON" : "OFF";
      publishStateString(topicSystemStatePub, p);
    }
    void publishHeaterState() {
      const char* p = heaterPower ? "ON" : "OFF";
      publishStateString(topicHeaterStatePub, p);
    }
    void publishModeState() {
      const char* p = autoMode ? "AUTO" : "MANUAL";
      publishStateString(topicModePub, p);
    }

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

// ================= MQTT LED handlers (ready for future implementation) =================
void handleLedSetPayload(const String& payload) {
  String s = payload;
  s.toUpperCase();
  if (s == "ON" || s == "1" || s == "TRUE") {
    ledPower = true;
    if (!systemPower) systemPower = true;
  } else if (s == "OFF" || s == "0" || s == "FALSE") {
    ledPower = false;
  }
  // TODO: Update physical LED output when MOSFET arrives
  publishLedState();
}

void handleLedBrightnessSetPayload(const String& payload) {
  int brightness = payload.toInt();
  if (brightness >= 0 && brightness <= 255) {
    ledBrightness = brightness;
    // TODO: Update physical LED brightness when MOSFET arrives
    publishLedState();
  }
}

// ================= MQTT callback (merge with existing handling for start time) =================
void mqttCallback(char* topic, byte* payload, unsigned int len) {
  String msg = String((char*)payload).substring(0, len);
  Serial.printf("MQTT recv %s -> %s\n", topic, msg.c_str());

  // existing topicSet handling (start time)
  if (String(topic) == topicSet) {
    time_t ts = strtoul(msg.c_str(), nullptr, 10);
    if (ts > 0) {  // Accept any reasonable timestamp
      startTime = ts;
      systemStartMillis = millis();  // Reset system start reference
      prefs.begin("biltong", false);
      prefs.putULong("startTime", startTime);
      prefs.end();
      Serial.printf("Time synchronized: %lu\n", startTime);
      timeRequestSent = false;  // Reset for future requests
      initialTimeSyncDone = true;  // Mark initial sync as complete
    }
    return;
  }

  // control topics handling
  if (String(topic) == topicSystemStateSet) {
    btnSys.handleSetPayload(msg);
    return;
  }
  if (String(topic) == topicHeaterStateSet) {
    btnHeater.handleSetPayload(msg);
    return;
  }
  if (String(topic) == topicModeSet) {
    btnMode.handleSetPayload(msg);
    return;
  }

  // LED control topics (ready for future implementation)
  if (String(topic) == topicLedStateSet) {
    handleLedSetPayload(msg);
    return;
  }
  if (String(topic) == topicLedBrightnessSet) {
    handleLedBrightnessSetPayload(msg);
    return;
  }

  // other mqtt topics can be handled here...
}

// ================= WiFi/MQTT setup =================
void setupWiFi() {
  WiFi.persistent(true);
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
}

void setupMQTT() {
  mqtt.setServer(mqttServer, mqttPort);
  mqtt.setCallback(mqttCallback);
}

bool connectMQTT() {
  if (!WiFi.isConnected()) return false;
  if (mqtt.connected()) return true;
  String clientId = "esp32-biltong-" + String(FIRMWARE_VERSION);
  if (mqtt.connect(clientId.c_str(), mqttUser, mqttPassword)) {
    Serial.printf("MQTT connected as %s\n", clientId.c_str());
    
    // subscribe to relevant set topics
    mqtt.subscribe(topicSet); // existing time set
    mqtt.subscribe(topicSystemStateSet);
    mqtt.subscribe(topicHeaterStateSet);
    mqtt.subscribe(topicModeSet);
    
    // LED subscriptions (ready for future implementation)
    mqtt.subscribe(topicLedStateSet);
    mqtt.subscribe(topicLedBrightnessSet);

    // publish initial states (retained)
    publishStateString(topicSystemStatePub, systemPower ? "ON" : "OFF", true);
    publishStateString(topicHeaterStatePub, heaterPower ? "ON" : "OFF", true);
    publishStateString(topicModePub, autoMode ? "AUTO" : "MANUAL", true);
    
    // LED initial state publishing (ready for future implementation)
    publishStateString(topicLedStatePub, ledPower ? "ON" : "OFF", true);
    publishStateInt(topicLedBrightnessPub, ledBrightness, true);

    // FIXED: Always request time synchronization on startup
    requestTimeFromHA();
    
    return true;
  }
  return false;
}

// ================= Setup all hardware (based on your original) =================
void setupAll() {
  Serial.printf("Biltong Controller v%s starting...\n", FIRMWARE_VERSION);
  
  systemStartMillis = millis();  // Record system start time
  
  prefs.begin("biltong", false);
  startTime = prefs.getULong("startTime", 0);
  prefs.end();

  setupWiFi();
  setupMQTT();
  dht.begin();

  lcd.begin(20, 4);
  lcd.backlight();
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Biltong v");
  lcd.print(FIRMWARE_VERSION);
  lcd.setCursor(0, 1);
  lcd.print("Starting...");
  delay(2000);
  lcd.clear();

  pinMode(PIN_TACH, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(PIN_TACH), tachISR, FALLING);

  // your original ledc attach/write (kept as-is)
  ledcAttach(PIN_PWM_FAN, 25000, 8);
  ledcWrite(PIN_PWM_FAN, dutyCycle);

  // TODO: LED PWM setup when MOSFET arrives
  // ledcAttach(PIN_PWM_LED, 25000, 8);
  // ledcWrite(PIN_PWM_LED, 0);

  // init buttons
  btnSys.begin();
  btnHeater.begin();
  btnMode.begin();
}

void setup() {
  Serial.begin(115200);
  while (!Serial) delay(10);
  setupAll();
}

// ================= Compute RPM - FIXED calculation =================
void computeRPM() {
  if (millis() - lastRPMcalc >= 1000) {
    uint32_t pulses = tachCount;
    tachCount = 0;
    lastRPMcalc = millis();
    // FIXED: Adjusted RPM calculation based on user testing
    currentRPM = pulses * 15; // Updated from user feedback
    
    // Debug output
    //Serial.printf("RPM Debug: pulses=%lu, calculated RPM=%lu, dutyCycle=%d, systemPower=%d\n", 
    //              pulses, currentRPM, dutyCycle, systemPower);
  }
}

// ================= Auto control (temperature->fan) =================
void autoControl() {
  if (!systemPower) {
    // system off => ensure fan off
    if (dutyCycle != 0) {
      dutyCycle = 0;
      ledcWrite(PIN_PWM_FAN, dutyCycle);
      Serial.printf("Auto control: System OFF, fan stopped\n");
    }
    return;
  }

  if (autoMode) {
    float t = dht.readTemperature();
    if (!isnan(t)) {
      int newDutyCycle;
      if (t <= 20.0) newDutyCycle = 0;
      else if (t >= 35.0) newDutyCycle = 255;
      else newDutyCycle = map(int(t * 100), 2000, 3500, 0, 255);
      
      // Only update if there's a significant change or if system just turned on
      if (abs(newDutyCycle - dutyCycle) > 5 || dutyCycle == 0) {
        dutyCycle = newDutyCycle;
        ledcWrite(PIN_PWM_FAN, dutyCycle);
        Serial.printf("Auto control: T=%.1fC, dutyCycle=%d (%d%%)\n", 
                     t, dutyCycle, map(dutyCycle, 0, 255, 0, 100));
      }
    }
    
    // In auto mode, LEDs can be controlled independently or follow logic
    // For now, LEDs are manually controlled even in auto mode
    // TODO: Add LED automation logic here if needed when MOSFET arrives
    
  } else {
    // FIXED: manual mode - maintain current dutyCycle, don't change it here
    // The duty cycle is set by button press or MQTT commands
    // Just ensure the PWM output matches the current duty cycle
    ledcWrite(PIN_PWM_FAN, dutyCycle);
  }

  // TODO: Update LED output based on current states when MOSFET arrives
}

// ================= Display functions (updated to show states) =================
void displayLineTempHum() {
  lcd.setCursor(0, 0);
  lcd.printf("T:%4.1fC H:%4.1f%%", dht.readTemperature(), dht.readHumidity());
}

void displayLineFanRPM() {
  lcd.setCursor(0, 1);
  // FIXED: Better RPM display formatting
  if (currentRPM >= 10000) {
    lcd.printf("Fan:%3d%% RPM:%4.1fK", map(dutyCycle, 0, 255, 0, 100), currentRPM/1000.0);
  } else {
    lcd.printf("Fan:%3d%% RPM:%4lu", map(dutyCycle, 0, 255, 0, 100), currentRPM);
  }
}

void displayLineRunTime() {
  time_t effectiveStart = getEffectiveStartTime();
  time_t currentTime = getCurrentTime();
  time_t diff = currentTime - effectiveStart;
  
  int hh = diff / 3600;
  int mm = (diff % 3600) / 60;
  int ss = diff % 60;
  
  lcd.setCursor(0, 2);
  lcd.printf("Run %02d:%02d:%02d", hh, mm, ss);
  
  // Show sync status - only for debugging
  // lcd.setCursor(15, 2);
  // lcd.print(startTime > 0 ? "S" : "L");
}

void displayLineStates() {
  // compact states line: S:ON H:OFF M:AUT
  lcd.setCursor(0, 3);
  char buf[21];
  snprintf(buf, sizeof(buf), "S:%s H:%s M:%s",
           systemPower ? "ON " : "OFF",
           heaterPower ? "ON " : "OFF",
           autoMode ? "AUT" : "MAN");
  lcd.print(buf);
}

void displayOnLCD() {
  displayLineTempHum();
  displayLineFanRPM();
  displayLineRunTime();
  displayLineStates();
}

// ================= Send status and time info =================
void sendAllStatus() {
  if (mqtt.connected()) {
    // Send main status CSV - includes start time in index 9
    String payload = String(dht.readTemperature(),1) + "," +
                     String(dht.readHumidity(),1) + "," +
                     String(map(dutyCycle,0,255,0,100)) + "," +
                     String(currentRPM) + "," +
                     String(systemPower ? 1 : 0) + "," +
                     String(heaterPower ? 1 : 0) + "," +
                     String(autoMode ? 1 : 0) + "," +
                     String(ledPower ? 1 : 0) + "," +
                     String(ledBrightness) + "," +
                     String(getEffectiveStartTime());
    mqtt.publish(topicStatus, payload.c_str(), true);

    // Periodic retry for time sync if not yet completed
    static unsigned long lastTimeRequest = 0;
    if (!initialTimeSyncDone && millis() - lastTimeRequest > 30000) {
      requestTimeFromHA();
      lastTimeRequest = millis();
    }
  }
}

// ================= Loop =================
void loop() {
  static uint32_t lastWifiCheck = 0, lastSend = 0;
  if (!WiFi.isConnected() && millis() - lastWifiCheck > 30000) {
    WiFi.disconnect();
    WiFi.reconnect();
    lastWifiCheck = millis();
  }

  // MQTT housekeeping
  mqtt.loop();
  connectMQTT();

  // compute RPM, update buttons and control
  computeRPM();
  btnSys.update();
  btnHeater.update();
  btnMode.update();

  // control logic: if systemPower off ensure fan off
  if (!systemPower) {
    dutyCycle = 0;
    ledcWrite(PIN_PWM_FAN, dutyCycle);
    // TODO: Turn off LEDs when MOSFET arrives
  } else {
    // if heaterPower influences fan desired behavior, adjust here
    autoControl();
  }

  displayOnLCD();

  if (millis() - lastSend > 5000) {
    sendAllStatus();
    lastSend = millis();
  }
}