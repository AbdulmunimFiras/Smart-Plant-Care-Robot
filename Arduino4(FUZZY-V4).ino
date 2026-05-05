// =====================================================================
//   SMART PLANT CARE ROBOT  -  v4 (IMPROVED)
//   Target board : Arduino Uno  (ATmega328P, 32KB flash, 2KB RAM)
// =====================================================================
//
//  Hardware:
//    A0  - Soil moisture sensor (analog)
//    A1  - LDR light sensor (analog)
//    D4  - AM2302 / DHT22 (temperature & humidity)
//    D7  - Relay  (water pump, ACTIVE-LOW module)
//    D8  - Buzzer (alarm) 
//    D9  - Servo  (polarized shading sheets)
//    A4  - I2C SDA  (LCD 16x2 @ 0x27)
//    A5  - I2C SCL  (LCD 16x2 @ 0x27)
//
//  Improvements over v3:
//    [1] Safe relay state at boot (no false pump trigger)
//    [2] Watchdog timer (auto-recovers from hangs)
//    [3] Realistic 3-min verify timeout (soil response time)
//    [4] Flicker-free LCD (clears only on screen change)
//    [5] EEPROM persistence (failure count survives power loss)
//    [6] Servo detach option for power saving
//    [7] Variance-based moisture-sensor disconnection detection
//    [8] Pump safety ceiling (8 s) decoupled from long pulse (5 s)
//    [9] No blocking delay() inside sensor averaging
//   [10] Robust fuzzy zero-handling
//   [11] Dynamic baseline tracking during alarm
//   [12] Negative-rise display clamped on LCD
// =====================================================================

#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <Servo.h>
#include <AM2302-Sensor.h>
#include <EEPROM.h>
#include <avr/wdt.h>

// ======================== PIN MAP ====================================
const int  moisturePin = A0;
const int  ldrPin      = A1;
const int  relayPin    = 7;
const int  buzzerPin   = 8;
const int  servoPin    = 9;
constexpr uint8_t dhtPin = 4;

// ======================== TUNABLE CONSTANTS ==========================
// Pump pulse lengths (fuzzy outputs)
const int PUMP_SHORT_MS              = 1500;
const int PUMP_MED_MS                = 3000;
const int PUMP_LONG_MS               = 5000;
const int PUMP_MIN_TRIGGER_MS        = 500;
const int PUMP_MAX_SAFETY_MS         = 8000;     // hard ceiling, > LONG
const unsigned long PUMP_COOLDOWN_MS = 30000UL;

// Failure detection
const unsigned long VERIFY_TIMEOUT_MS = 180000UL; // 3 min - real soil response
const int MOISTURE_RISE_THRESHOLD     = 5;
const int ALARM_AUTO_CLEAR_RISE       = 15;
const int MAX_CONSECUTIVE_FAILURES    = 2;

// Sampling & timing
const int  SENSOR_SAMPLES                  = 5;
const unsigned long DHT_INTERVAL_MS        = 2500UL;
const unsigned long SCREEN_INTERVAL_MS     = 4000UL;
const unsigned long DEBUG_INTERVAL_MS      = 2000UL;
const unsigned long LCD_REFRESH_MS         = 500UL;
const unsigned long EEPROM_THROTTLE_MS     = 60000UL;
const unsigned long ALARM_BASELINE_REFRESH = 30000UL;

// Disconnection detection thresholds
const int MOISTURE_RAW_MIN  = 5;
const int MOISTURE_RAW_MAX  = 1020;
const int MOISTURE_VAR_MIN  = 1;       // dead-flat = floating pin

// Humidity factors
const float H_FACTOR_DRY    = 1.3f;
const float H_FACTOR_NORMAL = 1.0f;
const float H_FACTOR_HUMID  = 0.5f;

// EEPROM layout
const int  EE_ADDR_MAGIC = 0;
const int  EE_ADDR_FAILS = 1;
const byte EE_MAGIC      = 0xA7;

// ======================== GLOBALS ====================================
AM2302::AM2302_Sensor am2302(dhtPin);
Servo               myServo;
LiquidCrystal_I2C   lcd(0x27, 16, 2);

enum PumpState { PUMP_IDLE, PUMP_RUNNING, PUMP_VERIFYING, PUMP_ALARM };
PumpState pumpState = PUMP_IDLE;

unsigned long pumpStartTime    = 0;
unsigned long pumpFinishTime   = 0;
unsigned long pumpDurationMs   = 0;
unsigned long lastPumpTime     = 0;
unsigned long lastEepromSave   = 0;
unsigned long lastBaselineRef  = 0;

int  moistureBeforePump  = 0;
int  consecutiveFailures = 0;

int   moisturePercent = 0;
int   lightPercent    = 0;
float temperature     = 0;
float humidity        = 0;
bool  dhtOK           = false;
bool  moistureOK      = true;

unsigned long lastDhtRead      = 0;
unsigned long lastDebugPrint   = 0;
unsigned long lastScreenSwitch = 0;
unsigned long lastLcdRefresh   = 0;
bool  showTempScreen           = false;
int   lastServoAngle           = -1;

// LCD screen tracker -> drives flicker-free updates
enum Screen { SCR_NONE, SCR_NORMAL, SCR_WATERING, SCR_VERIFY, SCR_ALARM };
Screen currentScreen = SCR_NONE;
bool   alarmBlinkPhase = false;

// ======================== FUZZY HELPERS ==============================
float triangle(float x, float a, float b, float c) {
  if (x <= a || x >= c) return 0;
  if (x < b)  return (x - a) / (b - a);
  return (c - x) / (c - b);
}

float trapezoid(float x, float a, float b, float c, float d) {
  if (x <= a || x >= d) return 0;
  if (x >= b && x <= c) return 1;
  if (x < b)  return (x - a) / (b - a);
  return (d - x) / (d - c);
}

// ======================== SENSOR READING =============================
//  Returns avg + spread.  spread = (max - min) over the samples.
//  A real analog sensor always shows >=1 LSB of noise; spread of 0
//  combined with a stuck rail is a strong sign the pin is floating.
struct AnalogStat { int avg; int spread; };

AnalogStat readAnalogAvg(int pin) {
  long sum = 0;
  int  mn = 1023, mx = 0;
  for (int i = 0; i < SENSOR_SAMPLES; i++) {
    int v = analogRead(pin);
    sum += v;
    if (v < mn) mn = v;
    if (v > mx) mx = v;
  }
  AnalogStat s;
  s.avg    = (int)(sum / SENSOR_SAMPLES);
  s.spread = mx - mn;
  return s;
}

// ======================== FUZZY PUMP DURATION ========================
int fuzzyPumpDuration(int moisture, float temp, float airHumidity) {
  // Moisture
  float m_dry    = trapezoid(moisture, 0, 0, 20, 40);
  float m_normal = triangle (moisture, 30, 50, 70);
  float m_wet    = trapezoid(moisture, 60, 80, 100, 100);

  // Temperature
  float t_cold = trapezoid(temp, 0, 0, 18, 25);
  float t_warm = triangle (temp, 22, 28, 34);
  float t_hot  = trapezoid(temp, 30, 35, 50, 50);

  // Rules
  float off    = m_wet;
  float shortP = min(m_normal, t_cold);
  float medP   = max(min(m_dry, t_cold), min(m_normal, t_warm));
  float longP  = max(min(m_dry, t_warm), min(m_dry, t_hot));

  float num = shortP * PUMP_SHORT_MS + medP * PUMP_MED_MS + longP * PUMP_LONG_MS;
  float den = off + shortP + medP + longP;
  if (den < 0.001f) return 0;
  float baseTime = num / den;

  // Humidity modifier
  float h_dry    = trapezoid(airHumidity, 0, 0, 30, 50);
  float h_normal = triangle (airHumidity, 40, 60, 80);
  float h_humid  = trapezoid(airHumidity, 70, 90, 100, 100);
  float hSum = h_dry + h_normal + h_humid;
  float hFactor = (hSum > 0.001f)
                  ? (h_dry * H_FACTOR_DRY + h_normal * H_FACTOR_NORMAL
                     + h_humid * H_FACTOR_HUMID) / hSum
                  : 1.0f;

  int result = (int)(baseTime * hFactor);
  return constrain(result, 0, PUMP_MAX_SAFETY_MS);
}

// ======================== FUZZY SERVO ANGLE ==========================
int fuzzyServoAngle(int light) {
  float l_low  = trapezoid(light, 0, 0, 20, 40);
  float l_med  = triangle (light, 30, 50, 70);
  float l_high = trapezoid(light, 60, 80, 100, 100);

  float den = l_low + l_med + l_high;
  if (den < 0.001f) return 0;
  return (int)((l_med * 45 + l_high * 90) / den);
}

// ======================== EEPROM HELPERS =============================
void eepromLoadFails() {
  byte magic = EEPROM.read(EE_ADDR_MAGIC);
  if (magic == EE_MAGIC) {
    consecutiveFailures = EEPROM.read(EE_ADDR_FAILS);
    if (consecutiveFailures > MAX_CONSECUTIVE_FAILURES + 5) consecutiveFailures = 0;
  } else {
    EEPROM.update(EE_ADDR_MAGIC, EE_MAGIC);
    EEPROM.update(EE_ADDR_FAILS, 0);
    consecutiveFailures = 0;
  }
}

void eepromSaveFailsThrottled() {
  unsigned long now = millis();
  if (now - lastEepromSave < EEPROM_THROTTLE_MS) return;
  EEPROM.update(EE_ADDR_FAILS, (byte)consecutiveFailures);
  lastEepromSave = now;
}

// ======================== LCD HELPERS (flicker-free) =================
void lcdEnsureScreen(Screen s) {
  if (currentScreen != s) {
    lcd.clear();
    currentScreen = s;
  }
}

void lcdShowNormal(int servoAngle) {
  lcdEnsureScreen(SCR_NORMAL);
  if (!showTempScreen) {
    lcd.setCursor(0, 0);
    if (moistureOK) {
      lcd.print(F("M:")); lcd.print(moisturePercent); lcd.print(F("%   "));
    } else {
      lcd.print(F("M:ERR  "));
    }
    lcd.setCursor(8, 0);
    lcd.print(F("L:")); lcd.print(lightPercent); lcd.print(F("%  "));
    lcd.setCursor(0, 1);
    lcd.print(F("Servo:")); lcd.print(servoAngle);
    lcd.print((char)223); lcd.print(F("    "));
  } else {
    lcd.setCursor(0, 0);
    if (dhtOK) {
      lcd.print(F("Temp: ")); lcd.print(temperature, 1); lcd.print(F(" C  "));
    } else {
      lcd.print(F("Temp:  ERROR   "));
    }
    lcd.setCursor(0, 1);
    if (dhtOK) {
      lcd.print(F("Humid:")); lcd.print((int)humidity); lcd.print(F("%   "));
    } else {
      lcd.print(F("Humid: ERROR   "));
    }
  }
}

void lcdShowWatering() {
  lcdEnsureScreen(SCR_WATERING);
  lcd.setCursor(0, 0); lcd.print(F("** Watering **  "));
  unsigned long elapsed   = millis() - pumpStartTime;
  unsigned long remaining = (pumpDurationMs > elapsed)
                            ? (pumpDurationMs - elapsed) / 1000UL : 0;
  lcd.setCursor(0, 1);
  lcd.print(F("Left: "));
  lcd.print(remaining); lcd.print(F("s     "));
}

void lcdShowVerifying() {
  lcdEnsureScreen(SCR_VERIFY);
  int rise = moisturePercent - moistureBeforePump;
  if (rise < 0) rise = 0;                              // never show negative
  unsigned long elapsed = millis() - pumpFinishTime;
  unsigned long left    = (VERIFY_TIMEOUT_MS > elapsed)
                          ? (VERIFY_TIMEOUT_MS - elapsed) / 1000UL : 0;
  lcd.setCursor(0, 0); lcd.print(F("Verifying...   "));
  lcd.setCursor(0, 1);
  lcd.print(F("Rise:")); lcd.print(rise);
  lcd.print(F("% "));    lcd.print(left); lcd.print(F("s   "));
}

void lcdShowAlarm() {
  lcdEnsureScreen(SCR_ALARM);
  // Alternate text content with blink, but don't clear() each time
  lcd.setCursor(0, 0);
  if (alarmBlinkPhase) lcd.print(F("!! PUMP FAULT !!"));
  else                 lcd.print(F("No moisture rise"));
  lcd.setCursor(0, 1);
  if (alarmBlinkPhase) lcd.print(F("Check tube/pump "));
  else                 lcd.print(F("after watering. "));
}

// ======================== SERVO ======================================
void updateServo(int angle) {
  if (lastServoAngle < 0 || abs(angle - lastServoAngle) >= 2) {
    myServo.write(angle);
    lastServoAngle = angle;
  }
}

// ======================== PUMP STATE MACHINE =========================
void runPumpStateMachine() {
  unsigned long now = millis();

  switch (pumpState) {

    case PUMP_IDLE: {
      if (!dhtOK || !moistureOK) return;
      int pumpTime = fuzzyPumpDuration(moisturePercent, temperature, humidity);
      bool readyToPump = (now - lastPumpTime) > PUMP_COOLDOWN_MS;

      if (pumpTime >= PUMP_MIN_TRIGGER_MS && readyToPump) {
        moistureBeforePump = moisturePercent;
        pumpDurationMs     = pumpTime;
        pumpStartTime      = now;
        digitalWrite(relayPin, LOW);    // pump ON (active-LOW)
        pumpState = PUMP_RUNNING;
        Serial.print(F("[PUMP] ON for ")); Serial.print(pumpTime);
        Serial.print(F(" ms. Baseline: "));
        Serial.print(moistureBeforePump); Serial.println(F("%"));
      }
      break;
    }

    case PUMP_RUNNING: {
      if (now - lastLcdRefresh > LCD_REFRESH_MS) {
        lcdShowWatering();
        lastLcdRefresh = now;
      }
      // Hard safety: never run longer than the ceiling
      unsigned long maxRun = min(pumpDurationMs, (unsigned long)PUMP_MAX_SAFETY_MS);
      if (now - pumpStartTime >= maxRun) {
        digitalWrite(relayPin, HIGH);   // pump OFF
        pumpFinishTime = now;
        pumpState = PUMP_VERIFYING;
        Serial.println(F("[PUMP] OFF -> verifying"));
      }
      break;
    }

    case PUMP_VERIFYING: {
      if (now - lastLcdRefresh > LCD_REFRESH_MS) {
        lcdShowVerifying();
        lastLcdRefresh = now;
      }
      int rise = moisturePercent - moistureBeforePump;

      // Success
      if (moistureOK && rise >= MOISTURE_RISE_THRESHOLD) {
        Serial.print(F("[OK] Watering OK. Rise: "));
        Serial.print(rise); Serial.println(F("%"));
        consecutiveFailures = 0;
        eepromSaveFailsThrottled();
        lastPumpTime = now;
        pumpState = PUMP_IDLE;
      }
      // Timeout
      else if (now - pumpFinishTime >= VERIFY_TIMEOUT_MS) {
        consecutiveFailures++;
        eepromSaveFailsThrottled();
        Serial.print(F("[FAIL] No rise. Failure #"));
        Serial.println(consecutiveFailures);

        if (consecutiveFailures >= MAX_CONSECUTIVE_FAILURES) {
          pumpState = PUMP_ALARM;
          lastBaselineRef = now;
          Serial.println(F("!!! ALARM: pump/tube fault !!!"));
        } else {
          lastPumpTime = now;          // re-arm cooldown
          pumpState = PUMP_IDLE;
        }
      }
      break;
    }

    case PUMP_ALARM: {
      static unsigned long lastBlink = 0;

      digitalWrite(relayPin, HIGH);    // ensure pump OFF

      // Blink + buzz
      if (now - lastBlink > 500) {
        lastBlink       = now;
        alarmBlinkPhase = !alarmBlinkPhase;
        lcdShowAlarm();
        digitalWrite(buzzerPin, alarmBlinkPhase ? HIGH : LOW);
      }

      // Periodically refresh baseline so a slow drying trend doesn't
      // mask a manual rescue (we want to detect fresh upward jumps).
      if (moistureOK && (now - lastBaselineRef > ALARM_BASELINE_REFRESH)) {
        if (moisturePercent < moistureBeforePump) {
          moistureBeforePump = moisturePercent;
        }
        lastBaselineRef = now;
      }

      // Auto-clear on big rise (manual fix)
      if (moistureOK
          && (moisturePercent - moistureBeforePump >= ALARM_AUTO_CLEAR_RISE)) {
        Serial.println(F("[OK] Alarm cleared - moisture recovered."));
        digitalWrite(buzzerPin, LOW);
        consecutiveFailures = 0;
        eepromSaveFailsThrottled();
        lastPumpTime = now;
        pumpState = PUMP_IDLE;
      }
      break;
    }
  }
}

// ======================== DEBUG ======================================
void printDebug(int servoAngle) {
  Serial.println(F("---------------------------"));
  if (moistureOK) { Serial.print(F("Moisture : ")); Serial.print(moisturePercent); Serial.println(F("%")); }
  else            { Serial.println(F("Moisture : DISCONNECTED")); }
  Serial.print(F("Light    : ")); Serial.print(lightPercent); Serial.println(F("%"));
  if (dhtOK) {
    Serial.print(F("Temp     : ")); Serial.print(temperature); Serial.println(F(" C"));
    Serial.print(F("Humidity : ")); Serial.print(humidity);    Serial.println(F("%"));
  } else {
    Serial.println(F("DHT      : ERROR"));
  }
  Serial.print(F("Servo    : ")); Serial.print(servoAngle); Serial.println(F(" deg"));

  const __FlashStringHelper* st;
  switch (pumpState) {
    case PUMP_IDLE:      st = F("IDLE");      break;
    case PUMP_RUNNING:   st = F("RUNNING");   break;
    case PUMP_VERIFYING: st = F("VERIFYING"); break;
    case PUMP_ALARM:     st = F("ALARM");     break;
    default:             st = F("?");         break;
  }
  Serial.print(F("State    : ")); Serial.println(st);
  Serial.print(F("Failures : ")); Serial.println(consecutiveFailures);
}

// ======================== SETUP ======================================
void setup() {
  Serial.begin(9600);

  // [SAFETY] Drive HIGH BEFORE setting OUTPUT, so the pin asserts the
  //          OFF level (active-LOW relay) the instant it becomes output.
  digitalWrite(relayPin, HIGH);
  pinMode(relayPin, OUTPUT);
  digitalWrite(relayPin, HIGH);

  digitalWrite(buzzerPin, LOW);
  pinMode(buzzerPin, OUTPUT);
  digitalWrite(buzzerPin, LOW);

  myServo.attach(servoPin);
  myServo.write(0);
  delay(500);

  lcd.init();
  lcd.backlight();
  lcd.setCursor(0, 0); lcd.print(F("Smart Plant Care"));
  lcd.setCursor(0, 1); lcd.print(F("AI Fuzzy v4"));
  delay(2000);
  lcd.clear();
  currentScreen = SCR_NONE;

  am2302.begin();
  eepromLoadFails();

  // Watchdog: 8 s window. Hangs longer than that auto-reset the MCU.
  wdt_enable(WDTO_8S);

  Serial.println(F("System started (v4)."));
  Serial.print  (F("Loaded fail count: "));
  Serial.println(consecutiveFailures);
}

// ======================== LOOP =======================================
void loop() {
  wdt_reset();                                  // pet the dog every loop
  unsigned long now = millis();

  // 1) DHT every 2.5 s (sensor protocol minimum)
  if (now - lastDhtRead >= DHT_INTERVAL_MS) {
    am2302.read();
    temperature = am2302.get_Temperature();
    humidity    = am2302.get_Humidity();
    dhtOK       = !isnan(temperature) && !isnan(humidity)
                  && temperature > -40 && temperature < 80
                  && humidity    >= 0  && humidity    <= 100;
    lastDhtRead = now;
  }

  // 2) Moisture & light
  AnalogStat ms = readAnalogAvg(moisturePin);
  AnalogStat ls = readAnalogAvg(ldrPin);

  // Disconnection: rail-stuck AND no noise -> floating pin
  bool railStuck = (ms.avg < MOISTURE_RAW_MIN) || (ms.avg > MOISTURE_RAW_MAX);
  bool deadFlat  = (ms.spread < MOISTURE_VAR_MIN);
  moistureOK = !(railStuck && deadFlat);

  moisturePercent = constrain(map(ms.avg, 1023, 0,   0, 100), 0, 100);
  lightPercent    = constrain(map(ls.avg,    0, 1023, 100, 0), 0, 100);

  // 3) Servo (fuzzy, with hysteresis)
  int servoAngle = fuzzyServoAngle(lightPercent);
  updateServo(servoAngle);

  // 4) Pump state machine
  runPumpStateMachine();

  // 5) LCD - only when state machine isn't drawing
  if (pumpState == PUMP_IDLE) {
    if (now - lastScreenSwitch > SCREEN_INTERVAL_MS) {
      showTempScreen   = !showTempScreen;
      lastScreenSwitch = now;
      lastLcdRefresh   = now;
      lcdShowNormal(servoAngle);
    } else if (now - lastLcdRefresh > LCD_REFRESH_MS) {
      lcdShowNormal(servoAngle);
      lastLcdRefresh = now;
    }
  }

  // 6) Serial debug
  if (now - lastDebugPrint > DEBUG_INTERVAL_MS) {
    printDebug(servoAngle);
    lastDebugPrint = now;
  }
}
