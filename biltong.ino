/*  Combined Biltong controller
    - Original code integrated with 3 buttons (debounced)
    - Buttons publish states to MQTT and accept set commands from MQTT
    - LCD updated with system/heater/mode states
    - No persistent storage of states after power off (as requested)
*/

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

// Topics
const char* topicGet     = "biltong/cmd/get_start";
const char* topicSet     = "biltong/cmd/set_start";
const char* topicReport  = "biltong/start_ts";
const char* topicStatus  = "biltong/status";

// new control topics
const char* topicSystemStatePub = "biltong/system_power";
const char* topicSystemStateSet = "biltong/system_power/set";
const char* topicHeaterStatePub = "biltong/heater_power";
const char* topicHeaterStateSet = "biltong/heater_power/set";
const char* topicModePub        = "biltong/mode";
const char* topicModeSet        = "biltong/mode/set";

const time_t JULY_2025_TS = 1751366400UL;

Preferences prefs;
time_t startTime = 0;

// ================= Hardware pins (all const as requested) =================
const int PIN_DHT         = 4;   // DHT data pin
const int PIN_TACH        = 5;   // tach input (interrupt)
const int PIN_PWM_FAN     = 16;  // PWM channel/pin used earlier in your code

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
int dutyCycle = 0; // 0-255 pwm

WiFiClient wifiClient;
PubSubClient mqtt(wifiClient);

// control states
bool systemPower = true;   // master on/off
bool heaterPower = false;  // manual heater on/off
bool autoMode    = true;   // true = automatic control (autoControl()), false = manual

void IRAM_ATTR tachISR() { tachCount++; }

// ================= Helper: publish state helpers =================
void publishStateString(const char* topic, const char* payload, bool retained = true) {
  if (mqtt.connected()) mqtt.publish(topic, payload, retained);
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
          // turning system off -> also turn off heater and stop fan PWM
          heaterPower = false;
          dutyCycle = 0;
          ledcWrite(PIN_PWM_FAN, dutyCycle);
        }
        // if turning system on, we leave heaterPower as-is (could be off)
        publishSystemState();
      } else if (pin == BTN_HEATER_PIN) {
        heaterPower = !heaterPower;
        if (heaterPower && !systemPower) {
          // enable system when user turns heater on
          systemPower = true;
        }
        publishHeaterState();
      } else if (pin == BTN_MODE_PIN) {
        autoMode = !autoMode;
        publishModeState();
      }
      // update LCD immediately
      // (LCD updated in main loop displayOnLCD)
      Serial.printf("Button pressed pin %d -> S:%d H:%d M:%d\n", pin, systemPower, heaterPower, autoMode);
    }

    // when MQTT set message arrives for this button, invoke handler
    void handleSetPayload(const String& payload) {
      String s = payload;
      s.toUpperCase();
      if (pin == BTN_SYS_PIN) {
        if (s == "ON" || s == "1" || s == "TRUE") systemPower = true;
        else if (s == "OFF" || s == "0" || s == "FALSE") {
          systemPower = false;
          heaterPower = false;
          dutyCycle = 0;
          ledcWrite(PIN_PWM_FAN, dutyCycle);
        }
        publishSystemState();
      } else if (pin == BTN_HEATER_PIN) {
        if (s == "ON" || s == "1" || s == "TRUE") {
          heaterPower = true;
          if (!systemPower) systemPower = true;
        } else if (s == "OFF" || s == "0" || s == "FALSE") {
          heaterPower = false;
        }
        publishHeaterState();
      } else if (pin == BTN_MODE_PIN) {
        if (s == "AUTO") autoMode = true;
        else if (s == "MANUAL") autoMode = false;
        else if (s == "1" || s == "ON") autoMode = true;
        else if (s == "0" || s == "OFF") autoMode = false;
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

// ================= MQTT callback (merge with existing handling for start time) =================
void mqttCallback(char* topic, byte* payload, unsigned int len) {
  String msg = String((char*)payload).substring(0, len);
  Serial.printf("MQTT recv %s -> %s\n", topic, msg.c_str());

  // existing topicSet handling (start time)
  if (String(topic) == topicSet) {
    time_t ts = strtoul(msg.c_str(), nullptr, 10);
    if (ts >= JULY_2025_TS) {
      startTime = ts;
      prefs.begin("biltong", false);
      prefs.putULong("startTime", startTime);
      prefs.end();
      mqtt.publish(topicReport, String(startTime).c_str(), true);
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
  if (mqtt.connect("esp32", mqttUser, mqttPassword)) {
    // subscribe to relevant set topics
    mqtt.subscribe(topicSet); // existing time set
    mqtt.subscribe(topicSystemStateSet);
    mqtt.subscribe(topicHeaterStateSet);
    mqtt.subscribe(topicModeSet);

    // publish initial states (retained)
    publishStateString(topicSystemStatePub, systemPower ? "ON" : "OFF", true);
    publishStateString(topicHeaterStatePub, heaterPower ? "ON" : "OFF", true);
    publishStateString(topicModePub, autoMode ? "AUTO" : "MANUAL", true);

    if (startTime >= JULY_2025_TS)
      mqtt.publish(topicReport, String(startTime).c_str(), true);
    else
      mqtt.publish(topicGet, "", false);
    return true;
  }
  return false;
}

// ================= Setup all hardware (based on your original) =================
void setupAll() {
  prefs.begin("biltong", false);
  startTime = prefs.getULong("startTime", 0);
  prefs.end();

  setupWiFi();
  setupMQTT();
  dht.begin();

  lcd.begin(20, 4);
  lcd.backlight();
  lcd.clear();

  pinMode(PIN_TACH, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(PIN_TACH), tachISR, FALLING);

  // your original ledc attach/write (kept as-is)
  ledcAttach(PIN_PWM_FAN, 25000, 8);
  ledcWrite(PIN_PWM_FAN, dutyCycle);

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

// ================= Compute RPM =================
void computeRPM() {
  if (millis() - lastRPMcalc >= 1000) {
    uint32_t pulses = tachCount;
    tachCount = 0;
    lastRPMcalc = millis();
    currentRPM = pulses * 60000UL / (1000UL * 2UL);
  }
}

// ================= Auto control (temperature->fan) =================
void autoControl() {
  if (!systemPower) {
    // system off => ensure fan off
    dutyCycle = 0;
    ledcWrite(PIN_PWM_FAN, dutyCycle);
    return;
  }

  if (autoMode) {
    float t = dht.readTemperature();
    if (!isnan(t)) {
      if (t <= 20.0) dutyCycle = 0;
      else if (t >= 35.0) dutyCycle = 255;
      else dutyCycle = map(int(t * 100), 2000, 3500, 0, 255);
      ledcWrite(PIN_PWM_FAN, dutyCycle);
    }
  } else {
    // manual mode: if systemPower==true, keep current dutyCycle
    // (no auto adjustment)
    ledcWrite(PIN_PWM_FAN, dutyCycle);
  }

  // if heaterPower == false and you want fan to still run, keep it
  // if you want fan to follow heater, you can add logic here
}

// ================= Display functions (updated to show states) =================
void displayLineTempHum() {
  lcd.setCursor(0, 0);
  lcd.printf("T:%4.1fC H:%4.1f%%", dht.readTemperature(), dht.readHumidity());
}

void displayLineFanRPM() {
  lcd.setCursor(0, 1);
  lcd.printf("Fan:%3d%% RPM:%4lu", map(dutyCycle, 0, 255, 0, 100), currentRPM);
}

void displayLineRunTime() {
  time_t base = startTime >= JULY_2025_TS ? startTime : (time_t)(millis() / 1000);
  time_t now = base + millis() / 1000;
  time_t diff = now - base;
  int hh = diff / 3600;
  int mm = (diff % 3600) / 60;
  int ss = diff % 60;
  lcd.setCursor(0, 2);
  lcd.printf("Run %02d:%02d:%02d", hh, mm, ss);
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

// ================= Send status (CSV) - extended with states =================
void sendAllStatus() {
  if (mqtt.connected()) {
    String payload = String(dht.readTemperature(),1) + "," +
                     String(dht.readHumidity(),1) + "," +
                     String(map(dutyCycle,0,255,0,100)) + "," +
                     String(currentRPM) + "," +
                     String(systemPower ? 1 : 0) + "," +
                     String(heaterPower ? 1 : 0) + "," +
                     String(autoMode ? 1 : 0) + "," +
                     String(startTime);
    mqtt.publish(topicStatus, payload.c_str(), true);
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
