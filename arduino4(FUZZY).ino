// SMART PLANT CARE ROBOT - WITH AI FUZZY LOGIC CONTROL
// 
//
// Components:
// 1- Moisture sensor (A0)
// 2- LDR light sensor (A1)
// 3- Servo motor - controls polarized sheets for shading (pin 9)
// 4- Relay - water pump (pin 7)
// 5- AM2302 Temperature & Humidity sensor (pin 4)
//
// AI: Fuzzy Logic Control
//   - Pump duration: based on (moisture + temperature)
//   - Servo angle:   smooth 0-90 based on light level

#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <Servo.h>
#include <AM2302-Sensor.h>

// ===== Pin Definitions =====
const int moisturePin     = A0;
const int ldrPin          = A1;
const int relayPin        = 7;
const int servoPin        = 9;
constexpr uint8_t dhtPin  = 4;

// ===== Objects =====
AM2302::AM2302_Sensor am2302(dhtPin);
Servo myServo;
LiquidCrystal_I2C lcd(0x27, 16, 2);

// ===== Globals =====
int ldrValue        = 0;
int moistureValue   = 0;
bool showTempScreen = false;

// =====================================================
//             FUZZYالقليل من ال
//              هذي تعريف الدوال الرياضية 
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
//   FUZZY هسة التحكم بالمضخة مال (moisture + temperature)
//   Output: pump duration in milliseconds
// =====================================================
int fuzzyPumpDuration(int moisture, float temp) {
  // Fuzzify moisture (%)
  float m_dry    = trapezoid(moisture, 0, 0, 20, 40);
  float m_normal = triangle(moisture, 30, 50, 70);
  float m_wet    = trapezoid(moisture, 60, 80, 100, 100);

  // Fuzzify temperature (Celsius)
  float t_cold = trapezoid(temp, 0, 0, 18, 25);
  float t_warm = triangle(temp, 22, 28, 34);
  float t_hot  = trapezoid(temp, 30, 35, 50, 50);

  // Fuzzy rules
  float off    = m_wet;                                       // wet -> no water
  float shortP = min(m_normal, t_cold);                       // ok soil + cold
  float medP   = max(min(m_dry, t_cold), min(m_normal, t_warm));
  float longP  = max(min(m_dry, t_warm), min(m_dry, t_hot));  // dry + hot -> long

  // Defuzzify (weighted average) -> milliseconds
  float num = off*0 + shortP*1500 + medP*3000 + longP*5000;
  float den = off + shortP + medP + longP;

  if (den == 0) return 0;
  return (int)(num / den);
}

// =====================================================
//   FUZZY SERVO ANGLE (replaces fixed sweep)  
//   Smooth angle 0-90 based on light intensity  //السيرفو هسة ما يتحرك بوقت معين، يستقر على زاوية تتناسب مع شدة الضوء
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
  lcd.print("Smart Plant Care"); // ن 
  lcd.setCursor(0, 1);
  lcd.print("AI Fuzzy Ctrl");
  delay(2000);
  lcd.clear();

  pinMode(relayPin, OUTPUT);
  digitalWrite(relayPin, HIGH);   // pump OFF

  am2302.begin();
}

// =====================================================
//                       LOOP
// =====================================================
void loop() {
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

  // 4. FUZZY SERVO (replaces the old sweep) // نفس ما كلنا فوك
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
    lcd.print("Servo:"); lcd.print(servoAngle); lcd.print((char)223); // degree symbol
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

  // 6. FUZZY PUMP CONTROL
  if (sensorOK) {
    int pumpTime = fuzzyPumpDuration(moisturePercent, temperature);
    Serial.print(">> Fuzzy Pump Time: "); Serial.print(pumpTime); Serial.println(" ms");

    if (pumpTime > 500) {                 // ignore tiny activations
      digitalWrite(relayPin, LOW);
      Serial.println("Pump: ON");
      delay(pumpTime);
      digitalWrite(relayPin, HIGH);
      Serial.println("Pump: OFF");
    }
  }

  delay(2000);
}
