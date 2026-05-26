// =====================================================================
//   SMART PLANT CARE ROBOT  -  v5.3 (MEGA + 3-ZONE + PI SUPERVISOR)
//   Target board : Arduino Mega 2560
// =====================================================================
//
//  v5.3 changes (audit fixes vs v5.2). Each marked [FIX AJ..AP]:
//   [AJ] Screen cycling no longer duplicates MOIST at startup. The
//        index now advances BEFORE assignment, so the initial render
//        plus first interval produce MOIST -> TEMP instead of
//        MOIST -> MOIST -> TEMP.
//   [AK] Manual servo override uses elapsed-delta timing instead of
//        a future-absolute deadline, so a millis() wraparound during
//        the 60s window doesn't fool the comparison.
//   [AL] Replaced (PumpState)0xFF sentinel (technically UB in C++)
//        with an explicit `hasReportedYet` bool.
//   [AM] Servo no longer detaches DURING manual override window, so
//        gravity/load can't cause it to drift away from the commanded
//        position. Re-enables normal detach once override expires.
//   [AN] Removed unreachable isnan branch from formatFixed1 (its only
//        caller is already guarded by dhtOK).
//   [AO] Centralised the manual-override check in servoManualActive()
//        so all three callers (loop, reportToPi, printDebug) use the
//        same logic and the wraparound fix lives in one place.
//   [AP] showAlarmNext initialised to true, so an alarm active at
//        boot (restored from EEPROM) appears on the LCD within ~4s
//        instead of waiting through a full normal-screen cycle.
//
//  Version chain:
//   v5.0: J..R   — Mega + 3 zones + Pi link, hydraulic safety.
//   v5.1: S..AC  — DHT fallback, JSON robustness, pump-hours odometer.
//   v5.2: AD..AI — fixed alarm weave, real servo override, fixed-width
//                  LCD, EE_MAGIC reverted to preserve fail history.
//   v5.3: AJ..AP — current.
// =====================================================================

#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <Servo.h>
#include <AM2302-Sensor.h>
#include <EEPROM.h>
#include <avr/wdt.h>
#include <stdio.h>
#include <string.h>
#include <math.h>

// ======================== CONFIG =====================================
const uint8_t NUM_ZONES = 3;

// ======================== PIN MAP ====================================
const uint8_t moisturePin[NUM_ZONES] = {A0, A1, A2};
const uint8_t valvePin[NUM_ZONES]    = {22, 24, 26};
const int     ldrPin       = A3;
const int     pumpPin      = 7;
const int     buzzerPin    = 8;
const int     servoPin     = 9;
constexpr uint8_t dhtPin   = 4;

// ======================== POLARITY ===================================
const uint8_t PUMP_ON   = HIGH;
const uint8_t PUMP_OFF  = LOW;
const uint8_t VALVE_ON  = HIGH;
const uint8_t VALVE_OFF = LOW;

// ======================== TUNABLE CONSTANTS ==========================
const int PUMP_SHORT_MS              = 1500;
const int PUMP_MED_MS                = 3000;
const int PUMP_LONG_MS               = 5000;
const int PUMP_MIN_TRIGGER_MS        = 500;
const int PUMP_MAX_SAFETY_MS         = 8000;
const unsigned long PUMP_COOLDOWN_MS = 30000UL;

const unsigned long VALVE_OPEN_DELAY_MS  = 200;
const unsigned long VALVE_CLOSE_DELAY_MS = 250;
const unsigned long VERIFY_PRE_DELAY_MS  = 200;

const unsigned long VERIFY_TIMEOUT_MS = 10000UL;
const int MOISTURE_RISE_THRESHOLD     = 5;
const int ALARM_AUTO_CLEAR_RISE       = 15;
const int MAX_CONSECUTIVE_FAILURES    = 2;

const int  SENSOR_SAMPLES                  = 5;
const unsigned long DHT_INTERVAL_MS        = 2500UL;
const unsigned long SCREEN_INTERVAL_MS     = 4000UL;
const unsigned long DEBUG_INTERVAL_MS      = 2000UL;
const unsigned long LCD_REFRESH_MS         = 500UL;
const unsigned long EEPROM_THROTTLE_MS     = 60000UL;
const unsigned long ALARM_BASELINE_REFRESH = 30000UL;
const unsigned long SERVO_DETACH_DELAY_MS  = 500UL;
const unsigned long PI_REPORT_MS           = 5000UL;
const unsigned long PUMP_HOUR_TICK_MS      = 3600000UL;

const unsigned long SERVO_MANUAL_OVERRIDE_MS = 60000UL;

const int MOISTURE_RAW_MIN  = 5;
const int MOISTURE_RAW_MAX  = 1020;
const int MOISTURE_VAR_MIN  = 1;

const float H_FACTOR_DRY    = 1.3f;
const float H_FACTOR_NORMAL = 1.0f;
const float H_FACTOR_HUMID  = 0.5f;

const float DHT_FALLBACK_TEMP = 22.0f;
const float DHT_FALLBACK_HUM  = 50.0f;
const float DHT_VALID_T_MIN = -40.0f;
const float DHT_VALID_T_MAX =  80.0f;
const float DHT_VALID_H_MIN =   0.0f;
const float DHT_VALID_H_MAX = 100.0f;

const int   SERVO_DEADBAND_DEG    = 2;
const int   ALARM_BLINK_PERIOD_MS = 500;
const float FUZZY_EPSILON         = 0.001f;

const int  EE_ADDR_MAGIC      = 0;
const int  EE_ADDR_FAILS0     = 1;
const int  EE_ADDR_PUMP_HRS_H = 4;
const int  EE_ADDR_PUMP_HRS_L = 5;
const byte EE_MAGIC           = 0xB5;

// ======================== TYPES ======================================
struct AnalogStat { int avg; int spread; };

struct Zone {
  int   moisturePercent;
  int   moistureBeforePump;
  int   consecutiveFailures;
  bool  moistureOK;
  bool  inAlarm;
  unsigned long lastPumpTime;
  unsigned long alarmBaselineTime;
  bool  alarmBaselineNeedsInit;
};

// ======================== GLOBALS ====================================
AM2302::AM2302_Sensor am2302(dhtPin);
Servo               myServo;
LiquidCrystal_I2C   lcd(0x27, 16, 2);

enum PumpState {
  PUMP_IDLE,
  VALVE_OPENING,
  PUMP_RUNNING,
  PUMP_STOPPING,
  VALVE_CLOSING,
  PUMP_VERIFYING
};
PumpState pumpState = PUMP_IDLE;

int  activeZone = -1;
Zone zones[NUM_ZONES];

unsigned long stateStartTime  = 0;
unsigned long pumpStartTime   = 0;
unsigned long pumpFinishTime  = 0;
unsigned long pumpDurationMs  = 0;
unsigned long lastEepromSave  = 0;

uint16_t      pumpHours          = 0;
unsigned long pumpRunAccumulator = 0;

int   lightPercent = 0;
float temperature  = 0;
float humidity     = 0;
bool  dhtOK        = false;

unsigned long lastDhtRead      = 0;
unsigned long lastDebugPrint   = 0;
unsigned long lastScreenSwitch = 0;
unsigned long lastLcdRefresh   = 0;
unsigned long lastPiReport     = 0;
uint8_t       screenIndex      = 0;
int           lastServoAngle   = -1;

uint8_t normalScreenIdx = 0;
// [FIX AP] Start with showAlarmNext=true so an EEPROM-restored alarm
// becomes visible on the very first cycle, not after a normal screen.
bool    showAlarmNext   = true;

// [FIX AK] Delta-based manual override (immune to millis() wraparound)
unsigned long servoManualStart   = 0;
bool          servoManualEngaged = false;

unsigned long servoMoveTime   = 0;
bool          servoIsAttached = false;

enum Screen { SCR_NONE, SCR_MOIST, SCR_TEMP, SCR_LIGHT, SCR_ALARMS,
              SCR_WATERING, SCR_VERIFY };
Screen currentScreen   = SCR_NONE;
bool   alarmBlinkPhase = false;

char    piBuf[96];
uint8_t piBufLen   = 0;
bool    piOverflow = false;

// [FIX AL] Explicit "have we reported yet" instead of sentinel cast
bool      hasReportedYet   = false;
PumpState lastReportedState = PUMP_IDLE;
int       lastReportedZone  = -1;

// ======================== SMALL HELPERS ==============================
bool anyAlarm() {
  for (uint8_t i = 0; i < NUM_ZONES; i++) if (zones[i].inAlarm) return true;
  return false;
}

void allValvesAndPumpOff() {
  digitalWrite(pumpPin, PUMP_OFF);
  for (uint8_t i = 0; i < NUM_ZONES; i++)
    digitalWrite(valvePin[i], VALVE_OFF);
}

// [FIX AO] Single source of truth for "is the Pi-set manual override
// currently active?". Auto-disengages when the window expires.
// Uses (now - start) so a millis() wrap during the window is safe.
bool servoManualActive() {
  if (!servoManualEngaged) return false;
  if ((millis() - servoManualStart) >= SERVO_MANUAL_OVERRIDE_MS) {
    servoManualEngaged = false;
    return false;
  }
  return true;
}

// [FIX AN] No isnan branch; callers (lcdShowTemp, reportToPi) guard
// against NaN via dhtOK before calling.
void printFixed1(Print &out, float v) {
  if (v < 0) { out.print('-'); v = -v; }
  long scaled = (long)(v * 10.0f + 0.5f);
  out.print(scaled / 10);
  out.print('.');
  out.print(scaled % 10);
}

void formatFixed1(char *buf, size_t bufsize, float v) {
  if (bufsize == 0) return;
  buf[0] = '\0';
  if (v < 0) {
    long s = (long)((-v) * 10.0f + 0.5f);
    snprintf(buf, bufsize, "-%ld.%ld", s / 10, s % 10);
  } else {
    long s = (long)(v * 10.0f + 0.5f);
    snprintf(buf, bufsize, "%ld.%ld", s / 10, s % 10);
  }
}

int parseIntField(const char *line, const char *key, int defaultVal) {
  const char *p = strstr(line, key);
  if (!p) return defaultVal;
  p = strchr(p, ':');
  if (!p) return defaultVal;
  p++;
  while (*p == ' ' || *p == '\t') p++;
  if (!(*p == '-' || (*p >= '0' && *p <= '9'))) return defaultVal;
  return atoi(p);
}

// ======================== FUZZY MEMBERSHIP ===========================
float triangle(float x, float a, float b, float c) {
  if (x <= a || x >= c) return 0;
  if (x < b)  return (x - a) / (b - a);
  return (c - x) / (c - b);
}

float trapezoid(float x, float a, float b, float c, float d) {
  if (x > b && x < c) return 1;
  if (x <= a || x >= d) return 0;
  if (x < b)  return (x - a) / (b - a);
  return (d - x) / (d - c);
}

// ======================== SENSOR READING =============================
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

// ======================== FUZZY OUTPUTS ==============================
int fuzzyPumpDuration(int moisture, float temp, float airHumidity) {
  float m_dry    = trapezoid(moisture, 0, 0, 20, 40);
  float m_normal = triangle (moisture, 30, 50, 70);
  float m_wet    = trapezoid(moisture, 60, 80, 100, 100);

  float t_cold = trapezoid(temp, 0, 0, 18, 25);
  float t_warm = triangle (temp, 22, 28, 34);
  float t_hot  = trapezoid(temp, 30, 35, 50, 50);

  float off    = m_wet;
  float shortP = min(m_normal, t_cold);
  float medP   = max(min(m_dry, t_cold), min(m_normal, t_warm));
  float longP  = max(min(m_dry, t_warm), min(m_dry, t_hot));

  float num = shortP * PUMP_SHORT_MS + medP * PUMP_MED_MS + longP * PUMP_LONG_MS;
  float den = off + shortP + medP + longP;
  if (den < FUZZY_EPSILON) return 0;
  float baseTime = num / den;

  float h_dry    = trapezoid(airHumidity, 0, 0, 30, 50);
  float h_normal = triangle (airHumidity, 40, 60, 80);
  float h_humid  = trapezoid(airHumidity, 70, 90, 100, 100);
  float hSum = h_dry + h_normal + h_humid;
  float hFactor = (hSum > FUZZY_EPSILON)
                  ? (h_dry * H_FACTOR_DRY + h_normal * H_FACTOR_NORMAL
                     + h_humid * H_FACTOR_HUMID) / hSum
                  : 1.0f;

  int result = (int)(baseTime * hFactor);
  return constrain(result, 0, PUMP_MAX_SAFETY_MS);
}

int fuzzyServoAngle(int light) {
  float l_low  = trapezoid(light, 0, 0, 20, 40);
  float l_med  = triangle (light, 30, 50, 70);
  float l_high = trapezoid(light, 60, 80, 100, 100);

  float den = l_low + l_med + l_high;
  if (den < FUZZY_EPSILON) return 0;
  return (int)((l_med * 45 + l_high * 90) / den);
}

// ======================== EEPROM =====================================
void eepromLoad() {
  byte magic = EEPROM.read(EE_ADDR_MAGIC);
  if (magic == EE_MAGIC) {
    for (uint8_t i = 0; i < NUM_ZONES; i++) {
      int f = EEPROM.read(EE_ADDR_FAILS0 + i);
      if (f > MAX_CONSECUTIVE_FAILURES + 5) f = 0;
      zones[i].consecutiveFailures    = f;
      zones[i].inAlarm                = (f >= MAX_CONSECUTIVE_FAILURES);
      zones[i].alarmBaselineNeedsInit = zones[i].inAlarm;
    }
    byte hi = EEPROM.read(EE_ADDR_PUMP_HRS_H);
    byte lo = EEPROM.read(EE_ADDR_PUMP_HRS_L);
    pumpHours = ((uint16_t)hi << 8) | lo;
    if (pumpHours == 0xFFFF) pumpHours = 0;
  } else {
    EEPROM.update(EE_ADDR_MAGIC, EE_MAGIC);
    for (uint8_t i = 0; i < NUM_ZONES; i++) {
      EEPROM.update(EE_ADDR_FAILS0 + i, 0);
      zones[i].consecutiveFailures    = 0;
      zones[i].inAlarm                = false;
      zones[i].alarmBaselineNeedsInit = false;
    }
    EEPROM.update(EE_ADDR_PUMP_HRS_H, 0);
    EEPROM.update(EE_ADDR_PUMP_HRS_L, 0);
    pumpHours = 0;
  }
}

void eepromSaveThrottled() {
  unsigned long now = millis();
  if (now - lastEepromSave < EEPROM_THROTTLE_MS) return;
  for (uint8_t i = 0; i < NUM_ZONES; i++)
    EEPROM.update(EE_ADDR_FAILS0 + i, (byte)zones[i].consecutiveFailures);
  EEPROM.update(EE_ADDR_PUMP_HRS_H, (byte)(pumpHours >> 8));
  EEPROM.update(EE_ADDR_PUMP_HRS_L, (byte)(pumpHours & 0xFF));
  lastEepromSave = now;
}

// ======================== LCD HELPERS ================================
void lcdEnsureScreen(Screen s) {
  if (currentScreen != s) {
    lcd.clear();
    currentScreen = s;
  }
}

void lcdShowMoisture() {
  lcdEnsureScreen(SCR_MOIST);
  char buf[17];

  if (zones[0].moistureOK && zones[1].moistureOK) {
    snprintf(buf, sizeof(buf), "Z1:%3d%% Z2:%3d%%",
             zones[0].moisturePercent, zones[1].moisturePercent);
  } else {
    snprintf(buf, sizeof(buf), "Z1:%s Z2:%s    ",
             zones[0].moistureOK ? "OK " : "ERR",
             zones[1].moistureOK ? "OK " : "ERR");
  }
  lcd.setCursor(0, 0); lcd.print(buf);

  if (activeZone >= 0) {
    if (zones[2].moistureOK) {
      snprintf(buf, sizeof(buf), "Z3:%3d%% Wat:Z%d ",
               zones[2].moisturePercent, activeZone + 1);
    } else {
      snprintf(buf, sizeof(buf), "Z3:ERR  Wat:Z%d  ", activeZone + 1);
    }
  } else {
    if (zones[2].moistureOK) {
      snprintf(buf, sizeof(buf), "Z3:%3d%% Idle   ",
               zones[2].moisturePercent);
    } else {
      snprintf(buf, sizeof(buf), "Z3:ERR   Idle   ");
    }
  }
  lcd.setCursor(0, 1); lcd.print(buf);
}

void lcdShowTemp() {
  lcdEnsureScreen(SCR_TEMP);
  if (dhtOK) {
    char num[8];
    formatFixed1(num, sizeof(num), temperature);
    char line[17];
    snprintf(line, sizeof(line), "Temp: %-5s C   ", num);
    lcd.setCursor(0, 0); lcd.print(line);
    snprintf(line, sizeof(line), "Humid: %3d%%      ", (int)humidity);
    lcd.setCursor(0, 1); lcd.print(line);
  } else {
    lcd.setCursor(0, 0); lcd.print(F("Temp:  ERROR    "));
    lcd.setCursor(0, 1); lcd.print(F("Humid: ERROR    "));
  }
}

void lcdShowLight(int servoAngle) {
  lcdEnsureScreen(SCR_LIGHT);
  char buf[17];
  snprintf(buf, sizeof(buf), "Light:%3d%%       ", lightPercent);
  lcd.setCursor(0, 0); lcd.print(buf);
  snprintf(buf, sizeof(buf), "Servo:%3d%c       ", servoAngle, (char)223);
  lcd.setCursor(0, 1); lcd.print(buf);
}

void lcdShowAlarmSummary() {
  lcdEnsureScreen(SCR_ALARMS);
  lcd.setCursor(0, 0);
  if (alarmBlinkPhase) lcd.print(F("!! PUMP FAULT !!"));
  else                 lcd.print(F("Zone fault(s):  "));
  char buf[17];
  snprintf(buf, sizeof(buf), "Z1:%c Z2:%c Z3:%c   ",
           zones[0].inAlarm ? 'X' : '-',
           zones[1].inAlarm ? 'X' : '-',
           zones[2].inAlarm ? 'X' : '-');
  lcd.setCursor(0, 1); lcd.print(buf);
}

void lcdShowWatering() {
  lcdEnsureScreen(SCR_WATERING);
  char buf[17];
  snprintf(buf, sizeof(buf), "** Watering Z%d **", activeZone + 1);
  lcd.setCursor(0, 0); lcd.print(buf);
  unsigned long elapsed   = millis() - pumpStartTime;
  unsigned long remaining = (pumpDurationMs > elapsed)
                            ? (pumpDurationMs - elapsed) / 1000UL : 0;
  snprintf(buf, sizeof(buf), "Left: %3lus       ", remaining);
  lcd.setCursor(0, 1); lcd.print(buf);
}

void lcdShowVerifying() {
  lcdEnsureScreen(SCR_VERIFY);
  int z = activeZone;
  int rise = (z >= 0) ? (zones[z].moisturePercent - zones[z].moistureBeforePump) : 0;
  if (rise < 0) rise = 0;
  unsigned long elapsed = millis() - pumpFinishTime;
  unsigned long left    = (VERIFY_TIMEOUT_MS > elapsed)
                          ? (VERIFY_TIMEOUT_MS - elapsed) / 1000UL : 0;

  char buf[17];
  snprintf(buf, sizeof(buf), "Verify Z%d...    ", z + 1);
  lcd.setCursor(0, 0); lcd.print(buf);
  snprintf(buf, sizeof(buf), "Rise:%-3d%% %3lus  ", rise, left);
  lcd.setCursor(0, 1); lcd.print(buf);
}

// ======================== SERVO ======================================
// [FIX AM] During manual override, don't detach — a load on the
// shading sheet could otherwise back-drive the servo a few degrees.
void updateServo(int angle) {
  if (lastServoAngle < 0 || abs(angle - lastServoAngle) >= SERVO_DEADBAND_DEG) {
    if (!servoIsAttached) {
      myServo.attach(servoPin);
      servoIsAttached = true;
    }
    myServo.write(angle);
    lastServoAngle = angle;
    servoMoveTime  = millis();
  }
  if (servoIsAttached
      && !servoManualActive()
      && (millis() - servoMoveTime >= SERVO_DETACH_DELAY_MS)) {
    myServo.detach();
    servoIsAttached = false;
  }
}

// ======================== ZONE SELECTION =============================
int selectZoneToWater() {
  float tempForFuzzy = dhtOK ? temperature : DHT_FALLBACK_TEMP;
  float humForFuzzy  = dhtOK ? humidity    : DHT_FALLBACK_HUM;

  int           bestZone     = -1;
  int           bestDuration = PUMP_MIN_TRIGGER_MS - 1;
  unsigned long now          = millis();

  for (uint8_t i = 0; i < NUM_ZONES; i++) {
    if (zones[i].inAlarm)     continue;
    if (!zones[i].moistureOK) continue;
    if (zones[i].lastPumpTime != 0
        && (now - zones[i].lastPumpTime) <= PUMP_COOLDOWN_MS) continue;

    int dur = fuzzyPumpDuration(zones[i].moisturePercent,
                                tempForFuzzy, humForFuzzy);
    if (dur >= PUMP_MIN_TRIGGER_MS && dur > bestDuration) {
      bestDuration = dur;
      bestZone     = i;
    }
  }
  return bestZone;
}

// ======================== PUMP/VALVE STATE MACHINE ===================
void runStateMachine() {
  unsigned long now = millis();

  switch (pumpState) {

    case PUMP_IDLE: {
      int z = selectZoneToWater();
      if (z < 0) return;

      float tempForFuzzy = dhtOK ? temperature : DHT_FALLBACK_TEMP;
      float humForFuzzy  = dhtOK ? humidity    : DHT_FALLBACK_HUM;
      int dur = fuzzyPumpDuration(zones[z].moisturePercent,
                                  tempForFuzzy, humForFuzzy);

      activeZone                  = z;
      zones[z].moistureBeforePump = zones[z].moisturePercent;
      pumpDurationMs              = dur;

      digitalWrite(valvePin[z], VALVE_ON);
      stateStartTime = now;
      pumpState      = VALVE_OPENING;
      Serial.print(F("[VALVE] Z")); Serial.print(z + 1);
      Serial.println(F(" opening"));
      break;
    }

    case VALVE_OPENING: {
      if (now - stateStartTime >= VALVE_OPEN_DELAY_MS) {
        digitalWrite(pumpPin, PUMP_ON);
        pumpStartTime = now;
        pumpState     = PUMP_RUNNING;
        Serial.print(F("[PUMP] ON for ")); Serial.print(pumpDurationMs);
        Serial.print(F(" ms (Z"));        Serial.print(activeZone + 1);
        Serial.print(F("). Baseline: "));
        Serial.print(zones[activeZone].moistureBeforePump); Serial.println(F("%"));
      }
      break;
    }

    case PUMP_RUNNING: {
      if (now - lastLcdRefresh > LCD_REFRESH_MS) {
        lcdShowWatering();
        lastLcdRefresh = now;
      }
      unsigned long maxRun = min(pumpDurationMs, (unsigned long)PUMP_MAX_SAFETY_MS);
      if (now - pumpStartTime >= maxRun) {
        digitalWrite(pumpPin, PUMP_OFF);
        pumpFinishTime = now;
        stateStartTime = now;
        pumpState      = PUMP_STOPPING;

        pumpRunAccumulator += (pumpFinishTime - pumpStartTime);
        while (pumpRunAccumulator >= PUMP_HOUR_TICK_MS) {
          pumpRunAccumulator -= PUMP_HOUR_TICK_MS;
          if (pumpHours < 0xFFFE) pumpHours++;
        }
        Serial.println(F("[PUMP] OFF -> draining"));
      }
      break;
    }

    case PUMP_STOPPING: {
      if (now - stateStartTime >= VALVE_CLOSE_DELAY_MS) {
        digitalWrite(valvePin[activeZone], VALVE_OFF);
        stateStartTime = now;
        pumpState      = VALVE_CLOSING;
        Serial.print(F("[VALVE] Z")); Serial.print(activeZone + 1);
        Serial.println(F(" closed"));
      }
      break;
    }

    case VALVE_CLOSING: {
      if (now - stateStartTime >= VERIFY_PRE_DELAY_MS) {
        pumpState = PUMP_VERIFYING;
      }
      break;
    }

    case PUMP_VERIFYING: {
      if (now - lastLcdRefresh > LCD_REFRESH_MS) {
        lcdShowVerifying();
        lastLcdRefresh = now;
      }
      int z    = activeZone;
      int rise = zones[z].moisturePercent - zones[z].moistureBeforePump;

      if (zones[z].moistureOK && rise >= MOISTURE_RISE_THRESHOLD) {
        Serial.print(F("[OK] Z")); Serial.print(z + 1);
        Serial.print(F(" rise: "));  Serial.print(rise); Serial.println(F("%"));
        zones[z].consecutiveFailures = 0;
        eepromSaveThrottled();
        zones[z].lastPumpTime = now;
        activeZone            = -1;
        pumpState             = PUMP_IDLE;
      }
      else if (now - pumpFinishTime >= VERIFY_TIMEOUT_MS) {
        zones[z].consecutiveFailures++;
        eepromSaveThrottled();
        Serial.print(F("[FAIL] Z")); Serial.print(z + 1);
        Serial.print(F(" no rise. Failure #"));
        Serial.println(zones[z].consecutiveFailures);

        if (zones[z].consecutiveFailures >= MAX_CONSECUTIVE_FAILURES) {
          zones[z].inAlarm                = true;
          zones[z].alarmBaselineTime      = now;
          zones[z].alarmBaselineNeedsInit = false;
          Serial.print(F("!!! ALARM Z")); Serial.print(z + 1);
          Serial.println(F(" !!!"));
        }
        zones[z].lastPumpTime = now;
        activeZone            = -1;
        pumpState             = PUMP_IDLE;
      }
      break;
    }
  }
}

// ======================== PER-ZONE ALARM HANDLING ====================
void updateAlarms() {
  unsigned long now  = millis();
  bool          any  = false;

  for (uint8_t i = 0; i < NUM_ZONES; i++) {
    if (!zones[i].inAlarm) continue;
    any = true;

    if (zones[i].alarmBaselineNeedsInit && zones[i].moistureOK) {
      zones[i].moistureBeforePump     = zones[i].moisturePercent;
      zones[i].alarmBaselineTime      = now;
      zones[i].alarmBaselineNeedsInit = false;
    }

    if (!zones[i].alarmBaselineNeedsInit && zones[i].moistureOK
        && (now - zones[i].alarmBaselineTime > ALARM_BASELINE_REFRESH)) {
      if (zones[i].moisturePercent < zones[i].moistureBeforePump) {
        zones[i].moistureBeforePump = zones[i].moisturePercent;
      }
      zones[i].alarmBaselineTime = now;
    }

    if (!zones[i].alarmBaselineNeedsInit && zones[i].moistureOK
        && (zones[i].moisturePercent - zones[i].moistureBeforePump
            >= ALARM_AUTO_CLEAR_RISE)) {
      Serial.print(F("[OK] Z")); Serial.print(i + 1);
      Serial.println(F(" alarm cleared - moisture recovered."));
      zones[i].inAlarm             = false;
      zones[i].consecutiveFailures = 0;
      eepromSaveThrottled();
    }
  }

  static unsigned long lastBlink = 0;
  if (any) {
    if (now - lastBlink > ALARM_BLINK_PERIOD_MS) {
      lastBlink       = now;
      alarmBlinkPhase = !alarmBlinkPhase;
      digitalWrite(buzzerPin, alarmBlinkPhase ? HIGH : LOW);
    }
  } else {
    digitalWrite(buzzerPin, LOW);
    alarmBlinkPhase = false;
  }
}

// ======================== PI COMMUNICATION ===========================
const __FlashStringHelper* stateName(PumpState s) {
  switch (s) {
    case PUMP_IDLE:      return F("IDLE");
    case VALVE_OPENING:  return F("VALVE_OPENING");
    case PUMP_RUNNING:   return F("RUNNING");
    case PUMP_STOPPING:  return F("PUMP_STOPPING");
    case VALVE_CLOSING:  return F("VALVE_CLOSING");
    case PUMP_VERIFYING: return F("VERIFYING");
    default:             return F("?");
  }
}

void reportToPi() {
  Serial.print(F("{\"t\":"));    Serial.print(millis());
  Serial.print(F(",\"state\":\""));
  Serial.print(stateName(pumpState));
  Serial.print('"');
  Serial.print(F(",\"zone\":")); Serial.print(activeZone);
  Serial.print(F(",\"m\":["));
  for (uint8_t i = 0; i < NUM_ZONES; i++) {
    if (i) Serial.print(',');
    Serial.print(zones[i].moistureOK ? zones[i].moisturePercent : -1);
  }
  Serial.print(F("],\"l\":")); Serial.print(lightPercent);
  Serial.print(F(",\"temp\":"));
  if (dhtOK) printFixed1(Serial, temperature);
  else       Serial.print(F("null"));
  Serial.print(F(",\"hum\":"));
  if (dhtOK) Serial.print((int)humidity);
  else       Serial.print(F("null"));
  Serial.print(F(",\"fails\":["));
  for (uint8_t i = 0; i < NUM_ZONES; i++) {
    if (i) Serial.print(',');
    Serial.print(zones[i].consecutiveFailures);
  }
  Serial.print(F("],\"alarms\":["));
  for (uint8_t i = 0; i < NUM_ZONES; i++) {
    if (i) Serial.print(',');
    Serial.print(zones[i].inAlarm ? 1 : 0);
  }
  Serial.print(F("],\"pump_hours\":"));
  Serial.print(pumpHours);
  Serial.print(F(",\"servo_manual\":"));
  Serial.print(servoManualActive() ? 1 : 0);
  Serial.println('}');

  lastReportedState = pumpState;
  lastReportedZone  = activeZone;
  lastPiReport      = millis();
  hasReportedYet    = true;
}

void handlePiCommand(const char* line) {
  const char* p = strstr(line, "\"cmd\"");
  if (!p) return;
  p = strchr(p, ':'); if (!p) return; p++;
  while (*p == ' ' || *p == '"') p++;

  if (strncmp(p, "clear_alarm", 11) == 0) {
    int zone = parseIntField(line, "\"zone\"", -1);
    if (zone < 0 || zone >= (int)NUM_ZONES) {
      Serial.println(F("[PI] clear_alarm rejected: bad zone"));
      return;
    }
    zones[zone].inAlarm                = false;
    zones[zone].consecutiveFailures    = 0;
    zones[zone].alarmBaselineNeedsInit = false;
    eepromSaveThrottled();
    Serial.print(F("[PI] Cleared alarm Z")); Serial.println(zone + 1);
  }
  else if (strncmp(p, "water", 5) == 0) {
    int zone = parseIntField(line, "\"zone\"", -1);
    int ms   = parseIntField(line, "\"ms\"",    2000);

    if (zone < 0 || zone >= (int)NUM_ZONES) {
      Serial.print(F("[PI] water rejected: bad zone "));
      Serial.println(zone);
      return;
    }
    if (pumpState != PUMP_IDLE) {
      Serial.print(F("[PI] water Z")); Serial.print(zone + 1);
      Serial.print(F(" rejected: pump busy ("));
      Serial.print(stateName(pumpState));
      Serial.println(F(")"));
      return;
    }
    if (zones[zone].inAlarm) {
      Serial.print(F("[PI] water Z")); Serial.print(zone + 1);
      Serial.println(F(" rejected: zone in alarm"));
      return;
    }

    activeZone                       = zone;
    zones[zone].moistureBeforePump   = zones[zone].moisturePercent;
    pumpDurationMs                   = constrain(ms,
                                          PUMP_MIN_TRIGGER_MS,
                                          PUMP_MAX_SAFETY_MS);
    digitalWrite(valvePin[zone], VALVE_ON);
    stateStartTime = millis();
    pumpState      = VALVE_OPENING;
    Serial.print(F("[PI] Manual water Z")); Serial.print(zone + 1);
    Serial.print(F(" for ")); Serial.print(pumpDurationMs);
    Serial.println(F(" ms"));
  }
  else if (strncmp(p, "set_servo", 9) == 0) {
    int ang = parseIntField(line, "\"angle\"", -1);
    if (ang < 0 || ang > 180) {
      Serial.print(F("[PI] set_servo rejected: bad angle "));
      Serial.println(ang);
      return;
    }
    servoManualStart   = millis();
    servoManualEngaged = true;
    updateServo(ang);
    Serial.print(F("[PI] Servo -> "));     Serial.print(ang);
    Serial.print(F(" (manual hold "));
    Serial.print(SERVO_MANUAL_OVERRIDE_MS / 1000UL);
    Serial.println(F("s)"));
  }
  else {
    Serial.println(F("[PI] unknown cmd"));
  }
}

void readPiSerial() {
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\n' || c == '\r') {
      if (!piOverflow && piBufLen > 0) {
        piBuf[piBufLen] = '\0';
        if (piBuf[0] == '{') handlePiCommand(piBuf);
      }
      piBufLen   = 0;
      piOverflow = false;
    } else if (piOverflow) {
      // discard until newline
    } else if (piBufLen < sizeof(piBuf) - 1) {
      piBuf[piBufLen++] = c;
    } else {
      piOverflow = true;
      Serial.println(F("[WARN] piBuf overflow; line discarded"));
    }
  }
}

// ======================== DEBUG ======================================
void printDebug(int servoAngle) {
  Serial.println(F("---------------------------"));
  for (uint8_t i = 0; i < NUM_ZONES; i++) {
    Serial.print(F("Zone ")); Serial.print(i + 1); Serial.print(F(": "));
    if (zones[i].moistureOK) {
      Serial.print(zones[i].moisturePercent); Serial.print('%');
    } else {
      Serial.print(F("DISCONNECTED"));
    }
    Serial.print(F("  fails=")); Serial.print(zones[i].consecutiveFailures);
    if (zones[i].inAlarm) Serial.print(F(" [ALARM]"));
    Serial.println();
  }
  Serial.print(F("Light : ")); Serial.print(lightPercent); Serial.println('%');
  if (dhtOK) {
    Serial.print(F("Temp  : ")); printFixed1(Serial, temperature);
    Serial.println(F(" C"));
    Serial.print(F("Humid : ")); Serial.print(humidity); Serial.println('%');
  } else {
    Serial.println(F("DHT   : ERROR (using fallback for irrigation)"));
  }
  Serial.print(F("Servo : ")); Serial.print(servoAngle);
  if (servoManualActive()) Serial.println(F(" deg (MANUAL OVERRIDE)"));
  else                     Serial.println(F(" deg"));
  Serial.print(F("State : ")); Serial.print(stateName(pumpState));
  if (activeZone >= 0) {
    Serial.print(F(" (Z")); Serial.print(activeZone + 1); Serial.print(')');
  }
  Serial.println();
  Serial.print(F("Pump-h: ")); Serial.println(pumpHours);
}

// ======================== SETUP ======================================
void setup() {
  MCUSR  = 0;
  wdt_disable();

  Serial.begin(115200);

  digitalWrite(pumpPin, PUMP_OFF);
  pinMode(pumpPin, OUTPUT);
  digitalWrite(pumpPin, PUMP_OFF);
  for (uint8_t i = 0; i < NUM_ZONES; i++) {
    digitalWrite(valvePin[i], VALVE_OFF);
    pinMode(valvePin[i], OUTPUT);
    digitalWrite(valvePin[i], VALVE_OFF);
  }

  digitalWrite(buzzerPin, LOW);
  pinMode(buzzerPin, OUTPUT);
  digitalWrite(buzzerPin, LOW);

  myServo.attach(servoPin);
  servoIsAttached = true;
  myServo.write(0);
  lastServoAngle = 0;
  servoMoveTime  = millis();
  delay(500);
  myServo.detach();
  servoIsAttached = false;

  lcd.init();
  lcd.backlight();
  lcd.setCursor(0, 0); lcd.print(F("Smart Plant Care"));
  lcd.setCursor(0, 1); lcd.print(F("v5.3 Mega 3-zone"));
  delay(2000);
  lcd.clear();
  currentScreen = SCR_NONE;

  am2302.begin();

  for (uint8_t i = 0; i < NUM_ZONES; i++) {
    zones[i].moisturePercent        = 0;
    zones[i].moistureBeforePump     = 0;
    zones[i].consecutiveFailures    = 0;
    zones[i].moistureOK             = true;
    zones[i].inAlarm                = false;
    zones[i].lastPumpTime           = 0;
    zones[i].alarmBaselineTime      = millis();
    zones[i].alarmBaselineNeedsInit = false;
  }

  eepromLoad();
  for (uint8_t i = 0; i < NUM_ZONES; i++) {
    if (zones[i].inAlarm) {
      Serial.print(F("[BOOT] Z")); Serial.print(i + 1);
      Serial.println(F(" restored to ALARM."));
    }
  }
  Serial.print(F("[BOOT] Pump-hours odometer: "));
  Serial.println(pumpHours);

  wdt_enable(WDTO_8S);

  Serial.println(F("System started (v5.3 - Mega, 3 zones, Pi link)."));
}

// ======================== LOOP =======================================
void loop() {
  wdt_reset();
  unsigned long now = millis();

  // 1) DHT every 2.5 s
  if (now - lastDhtRead >= DHT_INTERVAL_MS) {
    wdt_reset();
    int8_t st   = am2302.read();
    temperature = am2302.get_Temperature();
    humidity    = am2302.get_Humidity();
    dhtOK = (st == 0)
            && !isnan(temperature) && !isnan(humidity)
            && temperature > DHT_VALID_T_MIN && temperature < DHT_VALID_T_MAX
            && humidity    >= DHT_VALID_H_MIN && humidity    <= DHT_VALID_H_MAX;
    lastDhtRead = now;
  }

  // 2) Moisture + light
  for (uint8_t i = 0; i < NUM_ZONES; i++) {
    AnalogStat ms = readAnalogAvg(moisturePin[i]);
    bool railStuck = (ms.avg < MOISTURE_RAW_MIN) || (ms.avg > MOISTURE_RAW_MAX);
    bool deadFlat  = (ms.spread < MOISTURE_VAR_MIN);
    zones[i].moistureOK      = !(railStuck && deadFlat);
    zones[i].moisturePercent = constrain(map(ms.avg, 1023, 0, 0, 100), 0, 100);
  }
  AnalogStat ls = readAnalogAvg(ldrPin);
  lightPercent = constrain(map(ls.avg, 0, 1023, 100, 0), 0, 100);

  // 3) Servo — honour manual override window
  int servoAngle = fuzzyServoAngle(lightPercent);
  if (servoManualActive()) {
    if (lastServoAngle >= 0) updateServo(lastServoAngle);
  } else {
    updateServo(servoAngle);
  }

  // 4) State machine + alarms
  runStateMachine();
  updateAlarms();

  // 5) LCD — only when state machine isn't drawing its own screens
  if (pumpState == PUMP_IDLE) {
    if (now - lastScreenSwitch > SCREEN_INTERVAL_MS) {
      // [FIX AJ] Pre-increment so initial-render + first-interval
      // produce MOIST -> TEMP, not MOIST -> MOIST -> TEMP.
      // [FIX AP] showAlarmNext starts true (in globals), so an alarm
      // present at boot shows on the very first cycle.
      if (anyAlarm()) {
        if (showAlarmNext) {
          screenIndex   = 3;
          showAlarmNext = false;
        } else {
          normalScreenIdx = (normalScreenIdx + 1) % 3;
          screenIndex     = normalScreenIdx;
          showAlarmNext   = true;
        }
      } else {
        normalScreenIdx = (normalScreenIdx + 1) % 3;
        screenIndex     = normalScreenIdx;
        showAlarmNext   = true;   // pre-arm for future alarm
      }
      lastScreenSwitch = now;
      lastLcdRefresh   = now;
    }
    if (now - lastLcdRefresh > LCD_REFRESH_MS || currentScreen == SCR_NONE) {
      switch (screenIndex) {
        case 0:  lcdShowMoisture();        break;
        case 1:  lcdShowTemp();            break;
        case 2:  lcdShowLight(servoAngle); break;
        case 3:  lcdShowAlarmSummary();    break;
        default: screenIndex = 0; lcdShowMoisture(); break;
      }
      lastLcdRefresh = now;
    }
  }

  // 6) Pi <-> Mega comms
  readPiSerial();

  // [FIX AL] Explicit "first report" flag, no UB enum sentinel
  if (!hasReportedYet
      || lastReportedState != pumpState
      || lastReportedZone  != activeZone) {
    reportToPi();
  } else if (now - lastPiReport > PI_REPORT_MS) {
    reportToPi();
  }

  // 7) Human-readable debug
  if (now - lastDebugPrint > DEBUG_INTERVAL_MS) {
    printDebug(servoAngle);
    lastDebugPrint = now;
  }
}
