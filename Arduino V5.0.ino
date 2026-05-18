// =====================================================================
//   SMART PLANT CARE ROBOT  -  v5.0 (MEGA + 3-ZONE + PI SUPERVISOR)
//   Target board : Arduino Mega 2560  (ATmega2560, 256KB flash, 8KB RAM)
// =====================================================================
//
//  Hardware:
//    A0, A1, A2 - Soil moisture sensors (zone 1, 2, 3)
//    A3         - LDR light sensor
//    D4         - AM2302 / DHT22 (temperature & humidity)
//    D7         - Pump MOSFET module D4184 (active-HIGH)
//    D22        - Zone 1 solenoid valve MOSFET (active-HIGH)
//    D24        - Zone 2 solenoid valve MOSFET (active-HIGH)
//    D26        - Zone 3 solenoid valve MOSFET (active-HIGH)
//    D8         - Buzzer
//    D9         - Servo (polarized shading sheets)
//    D20 (SDA)  - LCD I2C @ 0x27
//    D21 (SCL)  - LCD I2C @ 0x27
//    USB        - Serial @ 115200 to Raspberry Pi 4 (also powers Mega)
//
//  Hydraulic order (CRITICAL):
//    Reservoir -> Pump -> Manifold (T) -> [Valve1, Valve2, Valve3]
//                                      -> Drip line for each zone.
//
//    The pump MUST be upstream of the valves because the diaphragm
//    valves need >=0.02 MPa to open properly. Pump provides that
//    pressure. The product description "not suitable for gravity-fed
//    systems" comes from this same minimum pressure requirement.
//
//  Power architecture:
//    External 12V 10A PSU -> pump + 3 valves (through MOSFET modules)
//    Pi 4 -> Mega via USB (5V, common GND with PSU)
//
//    Worst-case current: pump (~2-3A) + 1 valve (~1A) = ~4A.
//    Only one valve is ever open at a time (single shared pump).
//    Each valve needs a flyback diode (1N5408 / SS54), cathode to +12V.
//
//  Architecture / control philosophy:
//    The Mega is the AUTONOMOUS low-level controller. It runs the
//    fuzzy logic, state machine, and all actuators. EEPROM persistence
//    and an 8-second watchdog keep it safe. If the Pi crashes or its
//    SD card dies, the Mega keeps watering the plants.
//
//    The Pi is the SUPERVISOR: camera vision, dashboard, data logging,
//    optional command override. The Pi parses JSON status lines from
//    /dev/ttyACM0 and sends JSON commands back. The Mega is the
//    single source of truth for actuator state.
//
//  Pi <-> Mega protocol (Serial @ 115200, line-based):
//    Outbound (every 5s and on state change):
//      {"t":12345,"state":"RUNNING","zone":1,
//       "m":[42,67,31],"l":68,"temp":24.5,"hum":58,
//       "fails":[0,0,2],"alarms":[0,0,1]}
//    Inbound (Pi -> Mega, one JSON per line, ends with \n):
//      {"cmd":"water","zone":0,"ms":3000}      // manual water
//      {"cmd":"clear_alarm","zone":2}          // reset failures
//      {"cmd":"set_servo","angle":90}          // manual shade
//    Non-JSON lines on Serial are human-readable debug logs; the Pi
//    can filter by checking if the first non-whitespace char is '{'.
//
//  v5.0 changes vs v4.3 (each marked [FIX J..R]):
//   [J] Target board changed to Arduino Mega 2560 (pin map expanded).
//   [K] 3 zones: per-zone Zone struct holds sensor reading, baseline,
//       failure counter, alarm flag, and cooldown timer.
//   [L] State machine adds VALVE_OPENING / PUMP_STOPPING / VALVE_CLOSING
//       transitions so we never run the pump against a closed system
//       and never close a valve while the pump is still pushing.
//   [M] Per-zone alarms: a single jammed tube no longer halts the
//       other zones. Only the affected zone is locked out.
//   [N] EEPROM layout: 1 magic byte + 3 failure counters (4 bytes).
//   [O] LCD: rotating screens (moisture trio, temp/hum, light/servo,
//       and an alarm summary that appears only when needed).
//   [P] Hydraulic safety: VALVE_OPEN_DELAY (200ms) before pump ON,
//       VALVE_CLOSE_DELAY (250ms) after pump OFF before valve OFF.
//   [Q] JSON status over USB Serial @ 115200 for the Pi supervisor.
//       Minimal hand-rolled parser for "cmd"/"zone"/"ms"/"angle".
//   [R] wdt_disable() at boot to avoid Mega bootloader reset loops
//       if the watchdog ever triggers while in setup().
// =====================================================================

#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <Servo.h>
#include <AM2302-Sensor.h>
#include <EEPROM.h>
#include <avr/wdt.h>
#include <stdio.h>
#include <string.h>

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

// ======================== POLARITY (one place to flip if HW changes)
const uint8_t PUMP_ON   = HIGH;
const uint8_t PUMP_OFF  = LOW;
const uint8_t VALVE_ON  = HIGH;
const uint8_t VALVE_OFF = LOW;

// ======================== TUNABLE CONSTANTS ==========================
// Pump pulse lengths (fuzzy outputs)
const int PUMP_SHORT_MS              = 1500;
const int PUMP_MED_MS                = 3000;
const int PUMP_LONG_MS               = 5000;
const int PUMP_MIN_TRIGGER_MS        = 500;
const int PUMP_MAX_SAFETY_MS         = 8000;     // hard ceiling
const unsigned long PUMP_COOLDOWN_MS = 30000UL;  // per-zone

// Valve sequencing [FIX P]
const unsigned long VALVE_OPEN_DELAY_MS  = 200;  // valve open -> pump on
const unsigned long VALVE_CLOSE_DELAY_MS = 250;  // pump off  -> valve off
const unsigned long VERIFY_PRE_DELAY_MS  = 200;  // valve off -> verify

// Failure detection
const unsigned long VERIFY_TIMEOUT_MS = 10000UL;
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
const unsigned long SERVO_DETACH_DELAY_MS  = 500UL;
const unsigned long PI_REPORT_MS           = 5000UL;

// Disconnection detection
const int MOISTURE_RAW_MIN  = 5;
const int MOISTURE_RAW_MAX  = 1020;
const int MOISTURE_VAR_MIN  = 1;

// Humidity factors
const float H_FACTOR_DRY    = 1.3f;
const float H_FACTOR_NORMAL = 1.0f;
const float H_FACTOR_HUMID  = 0.5f;

// EEPROM layout [FIX N]
const int  EE_ADDR_MAGIC  = 0;
const int  EE_ADDR_FAILS0 = 1;       // bytes 1,2,3 = zone 0,1,2 fail counts
const byte EE_MAGIC       = 0xB5;    // new magic for v5 (so v4 EEPROM resets)

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

int  activeZone = -1;   // currently being watered (-1 = none)
Zone zones[NUM_ZONES];

unsigned long stateStartTime  = 0;
unsigned long pumpStartTime   = 0;
unsigned long pumpFinishTime  = 0;
unsigned long pumpDurationMs  = 0;
unsigned long lastEepromSave  = 0;

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

unsigned long servoMoveTime   = 0;
bool          servoIsAttached = false;

enum Screen { SCR_NONE, SCR_MOIST, SCR_TEMP, SCR_LIGHT, SCR_ALARMS,
              SCR_WATERING, SCR_VERIFY };
Screen currentScreen   = SCR_NONE;
bool   alarmBlinkPhase = false;

// Pi serial RX line buffer
char    piBuf[96];
uint8_t piBufLen = 0;

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

// ======================== FUZZY MEMBERSHIP ===========================
float triangle(float x, float a, float b, float c) {
  if (x <= a || x >= c) return 0;
  if (x < b)  return (x - a) / (b - a);
  return (c - x) / (c - b);
}

// [FIX A] Plateau check FIRST so x==a==b (and x==c==d) -> 1, not 0.
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

int fuzzyServoAngle(int light) {
  float l_low  = trapezoid(light, 0, 0, 20, 40);
  float l_med  = triangle (light, 30, 50, 70);
  float l_high = trapezoid(light, 60, 80, 100, 100);

  float den = l_low + l_med + l_high;
  if (den < 0.001f) return 0;
  return (int)((l_med * 45 + l_high * 90) / den);
}

// ======================== EEPROM =====================================
void eepromLoad() {
  byte magic = EEPROM.read(EE_ADDR_MAGIC);
  if (magic == EE_MAGIC) {
    for (uint8_t i = 0; i < NUM_ZONES; i++) {
      int f = EEPROM.read(EE_ADDR_FAILS0 + i);
      if (f > MAX_CONSECUTIVE_FAILURES + 5) f = 0;
      zones[i].consecutiveFailures   = f;
      zones[i].inAlarm               = (f >= MAX_CONSECUTIVE_FAILURES);
      zones[i].alarmBaselineNeedsInit = zones[i].inAlarm;  // [FIX B]
    }
  } else {
    EEPROM.update(EE_ADDR_MAGIC, EE_MAGIC);
    for (uint8_t i = 0; i < NUM_ZONES; i++) {
      EEPROM.update(EE_ADDR_FAILS0 + i, 0);
      zones[i].consecutiveFailures   = 0;
      zones[i].inAlarm               = false;
      zones[i].alarmBaselineNeedsInit = false;
    }
  }
}

void eepromSaveThrottled() {
  unsigned long now = millis();
  if (now - lastEepromSave < EEPROM_THROTTLE_MS) return;
  for (uint8_t i = 0; i < NUM_ZONES; i++)
    EEPROM.update(EE_ADDR_FAILS0 + i, (byte)zones[i].consecutiveFailures);
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

  // Line 1: Z1 + Z2
  if (zones[0].moistureOK && zones[1].moistureOK) {
    snprintf(buf, sizeof(buf), "Z1:%3d%% Z2:%3d%%",
             zones[0].moisturePercent, zones[1].moisturePercent);
  } else {
    snprintf(buf, sizeof(buf), "Z1:%s Z2:%s    ",
             zones[0].moistureOK ? "OK " : "ERR",
             zones[1].moistureOK ? "OK " : "ERR");
  }
  lcd.setCursor(0, 0); lcd.print(buf);

  // Line 2: Z3 + activity indicator
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
    char buf[17];
    int tInt  = (int)(temperature * 10);            // x.x
    snprintf(buf, sizeof(buf), "Temp: %d.%d C     ",
             tInt / 10, abs(tInt % 10));
    lcd.setCursor(0, 0); lcd.print(buf);
    snprintf(buf, sizeof(buf), "Humid: %3d%%      ", (int)humidity);
    lcd.setCursor(0, 1); lcd.print(buf);
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
  snprintf(buf, sizeof(buf), "Servo:%3d%c       ",
           servoAngle, (char)223);
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
void updateServo(int angle) {
  if (lastServoAngle < 0 || abs(angle - lastServoAngle) >= 2) {
    if (!servoIsAttached) {
      myServo.attach(servoPin);
      servoIsAttached = true;
    }
    myServo.write(angle);
    lastServoAngle = angle;
    servoMoveTime  = millis();
  }
  if (servoIsAttached && (millis() - servoMoveTime >= SERVO_DETACH_DELAY_MS)) {
    myServo.detach();
    servoIsAttached = false;
  }
}

// ======================== ZONE SELECTION =============================
// Pick the zone with the longest fuzzy pump duration (driest first),
// respecting per-zone cooldown and alarm state.
int selectZoneToWater() {
  if (!dhtOK) return -1;
  int           bestZone     = -1;
  int           bestDuration = PUMP_MIN_TRIGGER_MS - 1;
  unsigned long now          = millis();

  for (uint8_t i = 0; i < NUM_ZONES; i++) {
    if (zones[i].inAlarm)     continue;
    if (!zones[i].moistureOK) continue;
    if (zones[i].lastPumpTime != 0
        && (now - zones[i].lastPumpTime) <= PUMP_COOLDOWN_MS) continue;

    int dur = fuzzyPumpDuration(zones[i].moisturePercent, temperature, humidity);
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

      int dur = fuzzyPumpDuration(zones[z].moisturePercent, temperature, humidity);
      activeZone                  = z;
      zones[z].moistureBeforePump = zones[z].moisturePercent;
      pumpDurationMs              = dur;

      // [FIX P] Open valve FIRST; pump still off.
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
        Serial.println(F("[PUMP] OFF -> draining"));
      }
      break;
    }

    case PUMP_STOPPING: {
      // [FIX P] Let the pump coast / pressure relax before closing valve
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
          // [FIX M] only THIS zone enters alarm
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

    // [FIX B] Deferred baseline init (boot-time alarm from EEPROM)
    if (zones[i].alarmBaselineNeedsInit && zones[i].moistureOK) {
      zones[i].moistureBeforePump     = zones[i].moisturePercent;
      zones[i].alarmBaselineTime      = now;
      zones[i].alarmBaselineNeedsInit = false;
    }

    // Track downward drift so a slow drying soil doesn't auto-clear
    if (!zones[i].alarmBaselineNeedsInit && zones[i].moistureOK
        && (now - zones[i].alarmBaselineTime > ALARM_BASELINE_REFRESH)) {
      if (zones[i].moisturePercent < zones[i].moistureBeforePump) {
        zones[i].moistureBeforePump = zones[i].moisturePercent;
      }
      zones[i].alarmBaselineTime = now;
    }

    // Auto-clear if moisture rises a lot (user manually watered)
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

  // Buzzer blinks while ANY zone is in alarm
  static unsigned long lastBlink = 0;
  if (any) {
    if (now - lastBlink > 500) {
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

// Emit a single-line JSON status. The Pi reads it from /dev/ttyACM0.
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
  if (dhtOK) Serial.print(temperature, 1);
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
  Serial.println(F("]}"));
}

// Tiny "JSON-ish" parser for {"cmd":"...","zone":N,"ms":N,"angle":N}.
// Anything unrecognized is silently ignored (safe default).
void handlePiCommand(const char* line) {
  const char* p = strstr(line, "\"cmd\"");
  if (!p) return;
  p = strchr(p, ':'); if (!p) return; p++;
  while (*p == ' ' || *p == '"') p++;

  if (strncmp(p, "clear_alarm", 11) == 0) {
    const char* zptr = strstr(line, "\"zone\"");
    if (!zptr) return;
    zptr = strchr(zptr, ':'); if (!zptr) return;
    int zone = atoi(zptr + 1);
    if (zone >= 0 && zone < (int)NUM_ZONES) {
      zones[zone].inAlarm                = false;
      zones[zone].consecutiveFailures    = 0;
      zones[zone].alarmBaselineNeedsInit = false;
      eepromSaveThrottled();
      Serial.print(F("[PI] Cleared alarm Z")); Serial.println(zone + 1);
    }
  }
  else if (strncmp(p, "water", 5) == 0) {
    const char* zptr = strstr(line, "\"zone\"");
    const char* mptr = strstr(line, "\"ms\"");
    if (!zptr) return;
    zptr = strchr(zptr, ':'); if (!zptr) return;
    int zone = atoi(zptr + 1);
    int ms   = 2000;
    if (mptr) { mptr = strchr(mptr, ':'); if (mptr) ms = atoi(mptr + 1); }
    if (zone >= 0 && zone < (int)NUM_ZONES
        && pumpState == PUMP_IDLE
        && !zones[zone].inAlarm) {
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
  }
  else if (strncmp(p, "set_servo", 9) == 0) {
    const char* aptr = strstr(line, "\"angle\"");
    if (!aptr) return;
    aptr = strchr(aptr, ':'); if (!aptr) return;
    int ang = constrain(atoi(aptr + 1), 0, 180);
    if (!servoIsAttached) { myServo.attach(servoPin); servoIsAttached = true; }
    myServo.write(ang);
    lastServoAngle = ang;
    servoMoveTime  = millis();
    Serial.print(F("[PI] Servo -> ")); Serial.println(ang);
  }
}

void readPiSerial() {
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\n' || c == '\r') {
      if (piBufLen > 0) {
        piBuf[piBufLen] = '\0';
        if (piBuf[0] == '{') handlePiCommand(piBuf);  // only treat JSON
        piBufLen = 0;
      }
    } else if (piBufLen < sizeof(piBuf) - 1) {
      piBuf[piBufLen++] = c;
    } else {
      piBufLen = 0;  // overflow guard
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
    Serial.print(F("Temp  : ")); Serial.print(temperature); Serial.println(F(" C"));
    Serial.print(F("Humid : ")); Serial.print(humidity);    Serial.println('%');
  } else {
    Serial.println(F("DHT   : ERROR"));
  }
  Serial.print(F("Servo : ")); Serial.print(servoAngle); Serial.println(F(" deg"));
  Serial.print(F("State : ")); Serial.print(stateName(pumpState));
  if (activeZone >= 0) {
    Serial.print(F(" (Z")); Serial.print(activeZone + 1); Serial.print(')');
  }
  Serial.println();
}

// ======================== SETUP ======================================
void setup() {
  // [FIX R] Disable watchdog early - some Mega bootloaders fail to
  //         clear WDT after a reset, causing a boot-loop.
  MCUSR  = 0;
  wdt_disable();

  Serial.begin(115200);

  // Drive all MOSFETs LOW BEFORE making them outputs so the gate
  // stays at the OFF level the instant the pin becomes an output.
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

  // Servo init + detach to save power
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
  lcd.setCursor(0, 1); lcd.print(F("v5.0 Mega 3-zone"));
  delay(2000);
  lcd.clear();
  currentScreen = SCR_NONE;

  am2302.begin();

  // Init zones BEFORE loading EEPROM (eepromLoad fills failure fields)
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

  wdt_enable(WDTO_8S);

  Serial.println(F("System started (v5.0 - Mega, 3 zones, Pi link)."));
}

// ======================== LOOP =======================================
void loop() {
  wdt_reset();
  unsigned long now = millis();

  // 1) DHT every 2.5 s
  if (now - lastDhtRead >= DHT_INTERVAL_MS) {
    int8_t st   = am2302.read();
    temperature = am2302.get_Temperature();
    humidity    = am2302.get_Humidity();
    dhtOK = (st == 0)
            && !isnan(temperature) && !isnan(humidity)
            && temperature > -40 && temperature < 80
            && humidity    >= 0  && humidity    <= 100;
    lastDhtRead = now;
  }

  // 2) Moisture (3 zones) + light
  for (uint8_t i = 0; i < NUM_ZONES; i++) {
    AnalogStat ms = readAnalogAvg(moisturePin[i]);
    bool railStuck = (ms.avg < MOISTURE_RAW_MIN) || (ms.avg > MOISTURE_RAW_MAX);
    bool deadFlat  = (ms.spread < MOISTURE_VAR_MIN);
    zones[i].moistureOK      = !(railStuck && deadFlat);
    zones[i].moisturePercent = constrain(map(ms.avg, 1023, 0, 0, 100), 0, 100);
  }
  AnalogStat ls = readAnalogAvg(ldrPin);
  lightPercent = constrain(map(ls.avg, 0, 1023, 100, 0), 0, 100);

  // 3) Servo
  int servoAngle = fuzzyServoAngle(lightPercent);
  updateServo(servoAngle);

  // 4) State machine + per-zone alarms
  runStateMachine();
  updateAlarms();

  // 5) LCD - only when the state machine is not drawing its own screen
  if (pumpState == PUMP_IDLE) {
    if (now - lastScreenSwitch > SCREEN_INTERVAL_MS) {
      // Cycle screens. Include the alarm screen only if needed.
      uint8_t maxScreen = anyAlarm() ? 4 : 3;
      screenIndex      = (screenIndex + 1) % maxScreen;
      lastScreenSwitch = now;
      lastLcdRefresh   = now;
    }
    if (now - lastLcdRefresh > LCD_REFRESH_MS || currentScreen == SCR_NONE) {
      switch (screenIndex) {
        case 0: lcdShowMoisture();        break;
        case 1: lcdShowTemp();            break;
        case 2: lcdShowLight(servoAngle); break;
        case 3: lcdShowAlarmSummary();    break;
      }
      lastLcdRefresh = now;
    }
  }

  // 6) Pi <-> Mega comms
  readPiSerial();
  if (now - lastPiReport > PI_REPORT_MS) {
    reportToPi();
    lastPiReport = now;
  }

  // 7) Human-readable debug
  if (now - lastDebugPrint > DEBUG_INTERVAL_MS) {
    printDebug(servoAngle);
    lastDebugPrint = now;
  }
}
