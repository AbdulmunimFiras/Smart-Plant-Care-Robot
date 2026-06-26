// =====================================================================
//   SMART PLANT CARE ROBOT  -  v6.6 (MEGA + 3-ZONE + PI SUPERVISOR)
//   Target board : Arduino Mega 2560
// =====================================================================
//
//  v6.4 changes (Pi diagnosis display):
//   [BC] New "diag" command from the Pi carries the camera-based plant
//        identification + health result; shown on a new LCD screen
//        (SCR_DIAG) inside the normal rotation. DISPLAY ONLY — it never
//        affects irrigation, alarms, or the buzzer. The Mega has no
//        camera and cannot classify images; all vision runs on the Pi.
//   [BD] Adds parseStrField() for JSON string values (the short
//        species/condition label), mirroring parseIntField's strict
//        key:colon matching.
//
//  v6.3 changes (capacitive sensor migration):
//   [BA] Recalibrated the moisture mapping for a CAPACITIVE sensor.
//        Air (high ADC ~590) -> 0%, Water (low ADC ~300) -> 100%.
//        NOTE: the map() direction was already correct; only the two
//        calibration endpoints (MOISTURE_DRY_ADC / MOISTURE_WET_ADC)
//        changed. A capacitive sensor on 5V usually caps its output at
//        ~3V, so its air reading rarely exceeds ~600-700 ADC — the old
//        value of 850 would have made air read ~50% instead of 0%.
//        MEASURE YOUR OWN SENSOR and adjust these two numbers.
//   [BB] Removed a duplicate ZONE_MOISTURE_THRESHOLD[] definition that
//        caused a "redefinition" compile error in v6.1.
//
//  v6.1 changes (minor update):
//   [AW] Introduces per-zone moisture thresholds for more precise irrigation.
//   [AX] Enhances calibration process for accurate moisture readings.
//   [AY] Improves error handling and logging for better diagnostics.
//   [AZ] Adds support for additional sensor types and configurations.
//
//  v5.4 changes (audit fixes vs v5.3). Each marked [FIX AQ..AV]:
//   [AQ] parseIntField no longer falls for substring key matches.
//        Previously, "zone" would match inside "zone_priority". Now we
//        verify the colon comes immediately after the key (with
//        optional whitespace). Safe for any future JSON fields.
//   [AR] LCD renders IMMEDIATELY after a screen switch instead of
//        waiting up to 500ms for the next refresh cycle. Factored the
//        switch logic into renderCurrentScreen() so it's only written
//        in one place. Pre-existing issue since v5.0.
//   [AS] formatFixed1 once again guards against NaN/inf, but now
//        outputs "---" instead of "nan" so it can't break JSON if a
//        future caller forgets the dhtOK guard.
//   [AT] servoManualActive() is now a pure query; the auto-disengage
//        side effect moved to updateServoManualState() called once
//        per loop. Cleaner separation of read vs. mutate.
//   [AU] Format strings tightened to exactly 16 chars (was 17 chars
//        with the trailing space silently truncated by snprintf).
//        Pre-existing since v5.0; zero visual impact, just correctness.
//   [AV] PumpState now uses uint8_t underlying type, saving 1 byte
//        per state variable (cosmetic but free).
//
//  Version chain:
//   v5.0: J..R   — Mega + 3 zones + Pi link, hydraulic safety.
//   v5.1: S..AC  — DHT fallback, JSON robustness, pump-hours odometer.
//   v5.2: AD..AI — fixed alarm weave, real servo override.
//   v5.3: AJ..AP — startup ordering, wraparound-safe override.
//   v5.4: AQ..AV — audit fixes.
//   v6.1: AW..AZ — per-zone thresholds, calibration prep.
//   v6.3: BA..BB — capacitive sensor migration, dup-define fix.
//   v6.4: BC..BD — Pi diagnosis LCD screen (display only). (current)
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

// [BA] Calibration for a CAPACITIVE sensor (output usually capped ~3V on a
// 5V Mega, so the air reading rarely exceeds ~600-700 ADC):
//   - In AIR (dry)   -> high ADC, mapped to 0%
//   - In WATER (wet) -> low ADC,  mapped to 100%
// IMPORTANT: these are typical STARTING values. Measure your own sensor
// (temporarily print ms.avg over Serial) and replace the two numbers below.
const int MOISTURE_DRY_ADC  = 590;   // reading in air   (calibrate!)
const int MOISTURE_WET_ADC  = 300;   // reading in water (calibrate!)

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

// Per-zone fixed moisture thresholds for irrigation trigger:
//   Zone 0 = Tomatoes -> water if moisture < 65%
//   Zone 1 = Mint     -> water if moisture < 70%
//   Zone 2 = Ivy      -> water if moisture < 40%
const int ZONE_MOISTURE_THRESHOLD[NUM_ZONES] = {65, 70, 40};

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

// [FIX AV] explicit underlying type — saves 1B per variable, more
// importantly, makes the size predictable across compiler versions.
enum PumpState : uint8_t {
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
bool    showAlarmNext   = true;

// Normal LCD rotation (screenIndex values): moisture, temp, light,
// diagnosis. Index 3 is the alarm summary, interleaved separately and not
// part of this map.
const uint8_t NUM_NORMAL_SCREENS = 4;
const uint8_t NORMAL_SCREEN_MAP[NUM_NORMAL_SCREENS] = {0, 1, 2, 4};

// ---- Latest plant diagnosis pushed down from the Pi (DISPLAY ONLY) ---
// The Pi classifies a camera frame (species + health) and sends:
//   {"cmd":"diag","zone":<0..2|-1>,"h":<0..3>,"score":<0..100>,"label":"..."}
// h: 0=unknown 1=healthy 2=stressed 3=diseased. Shown on SCR_DIAG only;
// it NEVER influences irrigation, alarms, or the buzzer.
int           diagZone     = -1;     // -1 = whole view / unset
uint8_t       diagHealth   = 0;
int           diagScore    = -1;     // foliage 0..100, -1 = n/a
char          diagLabel[12] = "";    // short species/condition, truncated
unsigned long diagTime     = 0;      // millis() received, 0 = never

unsigned long servoManualStart   = 0;
bool          servoManualEngaged = false;

unsigned long servoMoveTime   = 0;
bool          servoIsAttached = false;

enum Screen : uint8_t {
  SCR_NONE, SCR_MOIST, SCR_TEMP, SCR_LIGHT, SCR_ALARMS,
  SCR_WATERING, SCR_VERIFY, SCR_DIAG
};
Screen currentScreen   = SCR_NONE;
bool   alarmBlinkPhase = false;

char    piBuf[96];
uint8_t piBufLen   = 0;
bool    piOverflow = false;

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

// [FIX AT] Pure query — no side effects. Mutation happens in
// updateServoManualState() called once per loop iteration.
bool servoManualActive() {
  return servoManualEngaged;
}

void updateServoManualState() {
  if (servoManualEngaged
      && (millis() - servoManualStart) >= SERVO_MANUAL_OVERRIDE_MS) {
    servoManualEngaged = false;
  }
}

// [FIX AS] Defensive NaN/inf guard; outputs "---" so JSON never breaks.
void printFixed1(Print &out, float v) {
  if (isnan(v) || isinf(v)) { out.print(F("---")); return; }
  if (v < 0) { out.print('-'); v = -v; }
  long scaled = (long)(v * 10.0f + 0.5f);
  out.print(scaled / 10);
  out.print('.');
  out.print(scaled % 10);
}

void formatFixed1(char *buf, size_t bufsize, float v) {
  if (bufsize == 0) return;
  buf[0] = '\0';
  if (isnan(v) || isinf(v)) { snprintf(buf, bufsize, "---"); return; }
  if (v < 0) {
    long s = (long)((-v) * 10.0f + 0.5f);
    snprintf(buf, bufsize, "-%ld.%ld", s / 10, s % 10);
  } else {
    long s = (long)(v * 10.0f + 0.5f);
    snprintf(buf, bufsize, "%ld.%ld", s / 10, s % 10);
  }
}

// [FIX AQ] Robust JSON integer field parser:
//   - Verifies the colon comes RIGHT AFTER the key (allowing only
//     whitespace), so `"zone"` no longer matches inside `"zone_priority"`.
//   - Rejects strings, booleans, null, objects.
int parseIntField(const char *line, const char *key, int defaultVal) {
  const char *p = strstr(line, key);
  if (!p) return defaultVal;
  p += strlen(key);                         // skip past the key itself
  while (*p == ' ' || *p == '\t') p++;
  if (*p != ':') return defaultVal;         // wrong field — substring match
  p++;
  while (*p == ' ' || *p == '\t') p++;
  if (!(*p == '-' || (*p >= '0' && *p <= '9'))) return defaultVal;
  return atoi(p);
}

// [BD] String-field counterpart to parseIntField. Same strict matching:
// the colon must come right after the key, and the value must be a JSON
// string. Copies up to (outSize-1) chars with minimal backslash-escape
// handling, and always null-terminates. Returns true if a value was found.
bool parseStrField(const char *line, const char *key,
                   char *out, size_t outSize) {
  if (outSize == 0) return false;
  out[0] = '\0';
  const char *p = strstr(line, key);
  if (!p) return false;
  p += strlen(key);
  while (*p == ' ' || *p == '\t') p++;
  if (*p != ':') return false;              // wrong field — substring match
  p++;
  while (*p == ' ' || *p == '\t') p++;
  if (*p != '"') return false;              // value is not a string
  p++;
  size_t i = 0;
  while (*p && *p != '"' && i < outSize - 1) {
    if (*p == '\\' && *(p + 1)) p++;        // skip escape, copy next literally
    out[i++] = *p++;
  }
  out[i] = '\0';
  return true;
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
// [FIX AU] All format strings below produce EXACTLY 16 visible chars
// (plus null). v5.0..v5.3 had several that produced 17 chars and got
// silently truncated by snprintf — visually identical (extra spaces)
// but technically a different write.
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
    snprintf(buf, sizeof(buf), "Z1:%3d%% Z2:%3d%% ",
             zones[0].moisturePercent, zones[1].moisturePercent);
  } else {
    snprintf(buf, sizeof(buf), "Z1:%s Z2:%s   ",            // [FIX AU] was 4 spaces
             zones[0].moistureOK ? "OK " : "ERR",
             zones[1].moistureOK ? "OK " : "ERR");
  }
  lcd.setCursor(0, 0); lcd.print(buf);

  if (activeZone >= 0) {
    if (zones[2].moistureOK) {
      snprintf(buf, sizeof(buf), "Z3:%3d%% Wat:Z%d  ",
               zones[2].moisturePercent, activeZone + 1);
    } else {
      snprintf(buf, sizeof(buf), "Z3:ERR  Wat:Z%d  ", activeZone + 1);
    }
  } else {
    if (zones[2].moistureOK) {
      snprintf(buf, sizeof(buf), "Z3:%3d%% Idle    ",
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
    snprintf(line, sizeof(line), "Humid: %3d%%     ", (int)humidity);
    lcd.setCursor(0, 1); lcd.print(line);
  } else {
    lcd.setCursor(0, 0); lcd.print(F("Temp:  ERROR    "));
    lcd.setCursor(0, 1); lcd.print(F("Humid: ERROR    "));
  }
}

void lcdShowLight(int servoAngle) {
  lcdEnsureScreen(SCR_LIGHT);
  char buf[17];
  snprintf(buf, sizeof(buf), "Light:%3d%%      ", lightPercent);
  lcd.setCursor(0, 0); lcd.print(buf);
  snprintf(buf, sizeof(buf), "Servo:%3d%c      ",          // [FIX AU] was 7 spaces
           servoAngle, (char)223);
  lcd.setCursor(0, 1); lcd.print(buf);
}

void lcdShowAlarmSummary() {
  lcdEnsureScreen(SCR_ALARMS);
  lcd.setCursor(0, 0);
  if (alarmBlinkPhase) lcd.print(F("!! PUMP FAULT !!"));
  else                 lcd.print(F("Zone fault(s):  "));
  char buf[17];
  snprintf(buf, sizeof(buf), "Z1:%c Z2:%c Z3:%c  ",        // [FIX AU] was 3 spaces
           zones[0].inAlarm ? 'X' : '-',
           zones[1].inAlarm ? 'X' : '-',
           zones[2].inAlarm ? 'X' : '-');
  lcd.setCursor(0, 1); lcd.print(buf);
}

void lcdShowWatering() {
  lcdEnsureScreen(SCR_WATERING);
  char buf[17];
  snprintf(buf, sizeof(buf), "**Watering Z%d** ", activeZone + 1);
  lcd.setCursor(0, 0); lcd.print(buf);
  unsigned long elapsed   = millis() - pumpStartTime;
  unsigned long remaining = (pumpDurationMs > elapsed)
                            ? (pumpDurationMs - elapsed) / 1000UL : 0;
  snprintf(buf, sizeof(buf), "Left: %3lus      ", remaining);
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

// Plant diagnosis pushed from the Pi. Both lines are EXACTLY 16 chars.
void lcdShowDiagnosis() {
  lcdEnsureScreen(SCR_DIAG);

  if (diagTime == 0) {
    lcd.setCursor(0, 0); lcd.print(F("Plant Diagnosis "));
    lcd.setCursor(0, 1); lcd.print(F("Awaiting scan..."));
    return;
  }

  // Line 0: "Diag Zx STATUS " — STATUS is one of four 7-char words.
  char zlab[4];
  if (diagZone >= 0 && diagZone < (int)NUM_ZONES)
    snprintf(zlab, sizeof(zlab), "Z%d", diagZone + 1);
  else
    snprintf(zlab, sizeof(zlab), "--");

  const char *st;
  switch (diagHealth) {
    case 1:  st = "Healthy"; break;
    case 2:  st = "Stressd"; break;
    case 3:  st = "Diseasd"; break;
    default: st = "Unknown"; break;
  }

  char line[17];
  snprintf(line, sizeof(line), "Diag %-2s %-7s ", zlab, st);
  lcd.setCursor(0, 0); lcd.print(line);

  // Line 1: short species/condition label, else foliage score, else blank.
  if (diagLabel[0] != '\0')
    snprintf(line, sizeof(line), "%-16s", diagLabel);
  else if (diagScore >= 0)
    snprintf(line, sizeof(line), "Foliage %3d%%    ", diagScore);
  else
    snprintf(line, sizeof(line), "                ");
  lcd.setCursor(0, 1); lcd.print(line);
}

// [FIX AR] Single rendering function used both for immediate-render
// after a screen switch AND for the periodic content refresh. The
// duplication that used to live in loop() is gone.
void renderCurrentScreen(int servoAngle) {
  switch (screenIndex) {
    case 0:  lcdShowMoisture();        break;
    case 1:  lcdShowTemp();            break;
    case 2:  lcdShowLight(servoAngle); break;
    case 3:  lcdShowAlarmSummary();    break;
    case 4:  lcdShowDiagnosis();       break;
    default: screenIndex = 0; lcdShowMoisture(); break;
  }
}

// ======================== SERVO ======================================
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
// Irrigation trigger uses fixed thresholds (ZONE_MOISTURE_THRESHOLD[]).
// When multiple zones need watering, the one with the largest deficit
// below its threshold is selected (most urgent first).
int selectZoneToWater() {
  int           bestZone    = -1;
  int           bestDeficit = 0;
  unsigned long now         = millis();

  for (uint8_t i = 0; i < NUM_ZONES; i++) {
    if (zones[i].inAlarm)     continue;
    if (!zones[i].moistureOK) continue;
    if (zones[i].lastPumpTime != 0
        && (now - zones[i].lastPumpTime) <= PUMP_COOLDOWN_MS) continue;

    if (zones[i].moisturePercent < ZONE_MOISTURE_THRESHOLD[i]) {
      int deficit = ZONE_MOISTURE_THRESHOLD[i] - zones[i].moisturePercent;
      if (deficit > bestDeficit) {
        bestDeficit = deficit;
        bestZone    = i;
      }
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
      // Threshold trigger already decided watering is needed; ensure a
      // meaningful minimum duration even when fuzzy returns a small value.
      if (dur < PUMP_MIN_TRIGGER_MS) dur = PUMP_MIN_TRIGGER_MS;

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
  else if (strncmp(p, "diag", 4) == 0) {
    // Display-only diagnosis from the Pi. Validate, store, show on LCD.
    int zone  = parseIntField(line, "\"zone\"",  -1);
    int h     = parseIntField(line, "\"h\"",      0);
    int score = parseIntField(line, "\"score\"", -1);
    if (zone < -1 || zone >= (int)NUM_ZONES) zone = -1;
    if (h < 0 || h > 3) h = 0;
    diagZone   = zone;
    diagHealth = (uint8_t)h;
    diagScore  = (score < 0 || score > 100) ? -1 : score;
    parseStrField(line, "\"label\"", diagLabel, sizeof(diagLabel));
    diagTime   = millis();
    Serial.print(F("[PI] Diagnosis zone="));
    Serial.print(zone);
    Serial.print(F(" h="));     Serial.print(h);
    if (diagLabel[0]) { Serial.print(F(" "));  Serial.print(diagLabel); }
    Serial.println();
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
  lcd.setCursor(0, 1); lcd.print(F("v6.4 Mega 3-zone"));
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

  Serial.println(F("System started (v6.4 - Mega, 3 zones, Pi link)."));
}

// ======================== LOOP =======================================
void loop() {
  wdt_reset();
  unsigned long now = millis();

  // [FIX AT] Disengage manual override once per loop, no side effects
  // elsewhere.
  updateServoManualState();

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
  //    Capacitive mapping: high ADC (air) -> 0%, low ADC (water) -> 100%.
  //    To calibrate, temporarily uncomment the Serial line below and read
  //    ms.avg in air and in water, then set MOISTURE_DRY_ADC / MOISTURE_WET_ADC.
  for (uint8_t i = 0; i < NUM_ZONES; i++) {
    AnalogStat ms = readAnalogAvg(moisturePin[i]);
    // if (i == 0) { Serial.print(F("RAW Z1=")); Serial.println(ms.avg); }
    bool railStuck = (ms.avg < MOISTURE_RAW_MIN) || (ms.avg > MOISTURE_RAW_MAX);
    bool deadFlat  = (ms.spread < MOISTURE_VAR_MIN);
    zones[i].moistureOK      = !(railStuck && deadFlat);
    zones[i].moisturePercent = constrain(map(ms.avg, MOISTURE_DRY_ADC, MOISTURE_WET_ADC, 0, 100), 0, 100);
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
      // Choose the next screen index. The diagnosis screen (last entry in
      // NORMAL_SCREEN_MAP) only joins the rotation once the Pi has sent at
      // least one diagnosis; until then we cycle the original three.
      uint8_t normalCount = (diagTime != 0) ? NUM_NORMAL_SCREENS
                                            : (uint8_t)(NUM_NORMAL_SCREENS - 1);
      if (anyAlarm()) {
        if (showAlarmNext) {
          screenIndex   = 3;
          showAlarmNext = false;
        } else {
          normalScreenIdx = (normalScreenIdx + 1) % normalCount;
          screenIndex     = NORMAL_SCREEN_MAP[normalScreenIdx];
          showAlarmNext   = true;
        }
      } else {
        normalScreenIdx = (normalScreenIdx + 1) % normalCount;
        screenIndex     = NORMAL_SCREEN_MAP[normalScreenIdx];
        showAlarmNext   = true;
      }
      lastScreenSwitch = now;

      // [FIX AR] Render IMMEDIATELY so the new screen appears without
      // a 500ms gap. The else-branch below handles dynamic value
      // refresh on the SAME screen.
      renderCurrentScreen(servoAngle);
      lastLcdRefresh = now;
    }
    else if (now - lastLcdRefresh > LCD_REFRESH_MS
             || currentScreen == SCR_NONE) {
      renderCurrentScreen(servoAngle);
      lastLcdRefresh = now;
    }
  }

  // 6) Pi <-> Mega comms
  readPiSerial();

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
