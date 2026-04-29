// THIS CODE IS FOR WEDNESDAY EXPLICITLY SO THE PEOPLE CAN UNDERSTAND WHATS HAPPENING!!!

// 1- Moisture sensor
// 2- LDR light sensor
// 3- Servo motor
// 4- Relay (water pump)
// 5- AM2302 Temperature & Humidity sensor

#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <Servo.h>
#include <AM2302-Sensor.h>

// ─── Pin Definitions ───────────────────────────────────────────────
const int moisturePin     = A0;
const int ldrPin          = A1;
const int relayPin        = 7;
const int servoPin        = 9;
constexpr uint8_t dhtPin  = 4;      // AM2302 DATA pin → pin 2

// ─── Object Declarations ───────────────────────────────────────────
AM2302::AM2302_Sensor am2302(dhtPin);

int ldrValue        = 0;
int moistureValue   = 0;
bool showTempScreen = false;

Servo myServo;
LiquidCrystal_I2C lcd(0x27, 16, 2);

void setup() {
  Serial.begin(9600);

  // Servo
  myServo.attach(servoPin);
  myServo.write(0);
  delay(1000);

  // LCD
  lcd.init();
  lcd.backlight();
  lcd.setCursor(0, 0);
  lcd.print("  Smart PLant Care Robot  ");
  lcd.setCursor(0, 1);
  lcd.print("  Starting...   ");
  delay(2000);
  lcd.clear();

  // Relay
  pinMode(relayPin, OUTPUT);
  digitalWrite(relayPin, HIGH);      // Pump OFF at start

  // AM2302
  am2302.begin();
}

void loop() {

  // ── 1. Read Moisture ─────────────────────────────────────────────
  moistureValue = analogRead(moisturePin);
  int moisturePercent = map(moistureValue, 1023, 0, 0, 100);

  Serial.print("Moisture: ");
  Serial.print(moisturePercent);
  Serial.println("%");

  // ── 2. Read Light ────────────────────────────────────────────────
  ldrValue = analogRead(ldrPin);
  int lightPercent = map(ldrValue, 0, 1023, 100, 0);

  Serial.print("Light: ");
  Serial.print(lightPercent);
  Serial.println("%");

  if      (lightPercent < 30) Serial.println("Status: LOW light");
  else if (lightPercent < 70) Serial.println("Status: MODERATE light");
  else                        Serial.println("Status: HIGH light");

  // ── 3. Read Temp & Humidity (AM2302) ─────────────────────────────
  am2302.read();                                   // ← just call read(), ignore status enum

  float temperature = am2302.get_Temperature();
  float humidity    = am2302.get_Humidity();

  bool sensorOK = !isnan(temperature) && !isnan(humidity);  // ← check values directly

  if (!sensorOK) {
    Serial.println("AM2302 ERROR: Check wiring on pin 4!");
  } else {
    Serial.print("Temperature: "); Serial.print(temperature); Serial.println(" C");
    Serial.print("Humidity:    "); Serial.print(humidity);    Serial.println("%");
  }

  Serial.println("-----------------");

  // ── 4. LCD Alternating Screens ───────────────────────────────────
  lcd.clear();

  if (!showTempScreen) {
    // Screen 1 — Moisture & Light
    lcd.setCursor(0, 0);
    lcd.print("Moist: ");
    lcd.print(moisturePercent);
    lcd.print("%   ");

    lcd.setCursor(0, 1);
    lcd.print("Light: ");
    lcd.print(lightPercent);
    lcd.print("%   ");

  } else {
    // Screen 2 — Temp & Humidity
    lcd.setCursor(0, 0);
    if (sensorOK) {
      lcd.print("Temp:  ");
      lcd.print(temperature, 1);
      lcd.print(" C  ");
    } else {
      lcd.print("Temp:  ERROR    ");
    }

    lcd.setCursor(0, 1);
    if (sensorOK) {
      lcd.print("Humid: ");
      lcd.print((int)humidity);
      lcd.print("%   ");
    } else {
      lcd.print("Humid: ERROR    ");
    }
  }

  showTempScreen = !showTempScreen;
  delay(2000);

  // ── 5. Servo Sweep ───────────────────────────────────────────────
  myServo.write(90);
  delay(3000);
  myServo.write(0);
  delay(3000);

  // ── 6. Pump Control ──────────────────────────────────────────────
  if (moisturePercent < 40) {
    digitalWrite(relayPin, LOW);
    Serial.println("Pump: ON");
    delay(3000);
    digitalWrite(relayPin, HIGH);
    Serial.println("Pump: OFF");
  }

  delay(1000);
}
