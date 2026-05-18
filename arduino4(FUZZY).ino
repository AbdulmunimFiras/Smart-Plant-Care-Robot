// SMART PLANT CARE ROBOT - WITH AI FUZZY LOGIC CONTROL + FAULT DETECTION
//
// Components:
// 1- Moisture sensor (A0)
// 2- LDR light sensor (A1)
// 3- Servo motor - controls polarized sheets for shading (pin 9)
// 4- Relay - water pump (pin 7)
// 5- AM2302 Temperature & Humidity sensor (pin 4)
// 6- Buzzer - alarm if pump fails (pin 8)
//
// AI: Fuzzy Logic Control
//   - Pump duration: based on (moisture + temperature)
//   - Servo angle:   smooth 0-90 based on light level
//   - Fault detect:  if pump runs but moisture doesn't rise -> buzzer

#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <Servo.h>
#include <AM2302-Sensor.h>

// ===== Pin Definitions =====
const int moisturePin     = A0;
const int ldrPin          = A1;
const int relayPin        = 7;
const int buzzerPin       = 8;         
const int servoPin        = 9;
constexpr uint8_t dhtPin  = 4;

// ===== Fault Detection Settings =====
const int MOISTURE_RISE_THRESHOLD = 3;  // % rise we expect after pumping
const int MAX_FAILURES            = 2;  // strikes before alarm
int  pumpFailureCount = 0;
bool pumpFault        = false;

// ===== Objects =====
AM2302::AM2302_Sensor am2302(dhtPin);
Servo myServo;
LiquidCrystal_I2C lcd(0x27, 16, 2);

// ===== Globals =====
int ldrValue        = 0;
int moistureValue   = 0;
bool showTempScreen = false;

// =====================================================
//             FUZZY الدوال الرياضية
// =====================================================
float triangle(float x, float a, float b, float c) {
  if (x <= a || x >= c) return 0;
  if (x == b) return 1;
  if (x < b)  return (x - a) / (b - a);
  return (c - x) / (c - b);
}

float trapezoid(float x, float a, float b, float c, float d) {
  if (x <= a || x >= d) return 0;
  if (x >= b && x <= c) return 1;
  if (x < b)  return (x - a) / (b - a);
  return (d - x) / (d - c);
}

// =====================================================
//   FUZZY PUMP CONTROL (moisture + temperature)
// =====================================================
int fuzzyPumpDuration(int moisture, float temp) {
  float m_dry    = trapezoid(moisture, 0, 0, 20, 40);
  float m_normal = triangle(moisture, 30, 50, 70);
  float m_wet    = trapezoid(moisture, 60, 80, 100, 100);

  float t_cold = trapezoid(temp, 0, 0, 18, 25);
  float t_warm = triangle(temp, 22, 28, 34);
  float t_hot  = trapezoid(temp, 30, 35, 50, 50);

  float off    = m_wet;
  float shortP = min(m_normal, t_cold);
  float medP   = max(min(m_dry, t_cold), min(m_normal, t_warm));
  float longP  = max(min(m_dry, t_warm), min(m_dry, t_hot));

  float num = off*0 + shortP*1500 + medP*3000 + longP*5000;
  float den = off + shortP + medP + longP;

  if (den == 0) return 0;
  return (int)(num / den);
}

// =====================================================
//   FUZZY SERVO ANGLE (light -> 0-90 deg)
// =====================================================
int fuzzyServoAngle(int light) {
  float l_low  = trapezoid(light, 0, 0, 20, 40);
  float l_med  = triangle(light, 30, 50, 70);
  float l_high = trapezoid(light, 60, 80, 100, 100);

  float num = l_low*0 + l_med*45 + l_high*90;
  float den = l_low + l_med + l_high;

  if (den == 0) return 0;
  return (int)(num / den);
}

// =====================================================
//                       SETUP
// =====================================================
void setup() {
  Serial.begin(9600);

  myServo.attach(servoPin);
  myServo.write(0);
  delay(1000);

  lcd.init();
  lcd.backlight();
  lcd.setCursor(0, 0);
  lcd.print("Smart Plant Care");
  lcd.setCursor(0, 1);
  lcd.print("AI Fuzzy Ctrl");
  delay(2000);
  lcd.clear();

  pinMode(relayPin, OUTPUT);
  digitalWrite(relayPin, HIGH);     // pump OFF

  pinMode(buzzerPin, OUTPUT);       // NEW
  digitalWrite(buzzerPin, LOW);     // buzzer OFF

  am2302.begin();
}

// =====================================================
//                       LOOP
// =====================================================
void loop() {

  // ── 0. ALARM MODE ──
  // If pump fault was confirmed, stop everything and just beep.
  if (pumpFault) {
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("!! PUMP FAULT !!");
    lcd.setCursor(0, 1);
    lcd.print("Check pump/tube ");

    digitalWrite(buzzerPin, HIGH);
    delay(500);
    digitalWrite(buzzerPin, LOW);
    delay(500);
    return;     // skip the rest of the loop
  }

  // 1. Moisture
  moistureValue = analogRead(moisturePin);
  int moisturePercent = map(moistureValue, 1023, 0, 0, 100);
  Serial.print("Moisture: "); Serial.print(moisturePercent); Serial.println("%");

  // 2. Light
  ldrValue = analogRead(ldrPin);
  int lightPercent = map(ldrValue, 0, 1023, 100, 0);
  Serial.print("Light: "); Serial.print(lightPercent); Serial.println("%");

  // 3. Temperature & Humidity
  am2302.read();
  float temperature = am2302.get_Temperature();
  float humidity    = am2302.get_Humidity();
  bool sensorOK = !isnan(temperature) && !isnan(humidity);

  if (!sensorOK) {
    Serial.println("AM2302 ERROR: Check wiring on pin 4!");
  } else {
    Serial.print("Temp: "); Serial.print(temperature); Serial.println(" C");
    Serial.print("Humid: "); Serial.print(humidity); Serial.println("%");
  }

  // 4. FUZZY SERVO
  int servoAngle = fuzzyServoAngle(lightPercent);
  myServo.write(servoAngle);
  Serial.print(">> Fuzzy Servo Angle: "); Serial.print(servoAngle); Serial.println(" deg");

  // 5. LCD - Alternating Screens
  lcd.clear();
  if (!showTempScreen) {
    lcd.setCursor(0, 0);
    lcd.print("M:"); lcd.print(moisturePercent);
    lcd.print("% L:"); lcd.print(lightPercent); lcd.print("%");
    lcd.setCursor(0, 1);
    lcd.print("Servo:"); lcd.print(servoAngle); lcd.print((char)223);
  } else {
    lcd.setCursor(0, 0);
    if (sensorOK) { lcd.print("Temp: "); lcd.print(temperature, 1); lcd.print(" C"); }
    else          { lcd.print("Temp:  ERROR"); }
    lcd.setCursor(0, 1);
    if (sensorOK) { lcd.print("Humid:"); lcd.print((int)humidity); lcd.print("%"); }
    else          { lcd.print("Humid: ERROR"); }
  }
  showTempScreen = !showTempScreen;
  Serial.println("-----------------");

  // 6. FUZZY PUMP CONTROL + FAULT DETECTION
  if (sensorOK) {
    int pumpTime = fuzzyPumpDuration(moisturePercent, temperature);
    Serial.print(">> Fuzzy Pump Time: "); Serial.print(pumpTime); Serial.println(" ms");

    if (pumpTime > 500) {

      // Step 1: Snapshot moisture BEFORE pumping
      int moistureBefore = moisturePercent;
      Serial.print("Moisture before: ");
      Serial.print(moistureBefore); Serial.println("%");

      // Step 2: Run the pump
      digitalWrite(relayPin, LOW);
      Serial.println("Pump: ON");
      delay(pumpTime);
      digitalWrite(relayPin, HIGH);
      Serial.println("Pump: OFF");

      // Step 3: Wait for water to soak in
      Serial.println("Waiting for soak...");
      delay(4000);

      // Step 4: Re-read moisture
      int rawAfter = analogRead(moisturePin);
      int moistureAfter = map(rawAfter, 1023, 0, 0, 100);
      int rise = moistureAfter - moistureBefore;

      Serial.print("Moisture after: ");
      Serial.print(moistureAfter); Serial.println("%");
      Serial.print("Rise: ");
      Serial.print(rise); Serial.println("%");

      // Step 5: Decide if this counts as a failure
      if (rise < MOISTURE_RISE_THRESHOLD) {
        pumpFailureCount++;
        Serial.print("!! Suspected pump fault. Strike ");
        Serial.print(pumpFailureCount);
        Serial.print("/");
        Serial.println(MAX_FAILURES);

        if (pumpFailureCount >= MAX_FAILURES) {
          pumpFault = true;
          Serial.println("!!! PUMP FAULT CONFIRMED !!!");
        }
      } else {
        pumpFailureCount = 0;
        Serial.println("Watering OK!");
      }
    }
  }

  delay(2000);
}