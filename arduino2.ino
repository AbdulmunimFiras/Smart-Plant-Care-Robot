// THIS CODE IS FOR WEDNESDAY EXPLICITLY SO THE PEOPLE CAN UNDERSTAND WHATS HAPPENING!!!

// 1- Moisture sensor
const int moisturePin = A0;  // sensor connected to analog pin A0
int moistureValue = 0;       // variable to store the reading

void setup() {
  Serial.begin(9600);        // start serial communication at 9600 baud
}

void loop() {
  moistureValue = analogRead(moisturePin);  // read sensor (0–1023)
  
  int moisturePercent = map(moistureValue, 1023, 0, 0, 100); // convert to %
  
  Serial.print("Moisture: ");
  Serial.print(moisturePercent);
  Serial.println("%");
  
  delay(1000);  // wait 1 second before next reading
}

// 2 - LDR (LIght sensor)
const int ldrPin = A1;      // LDR connected to analog pin A1
int ldrValue = 0;           // variable to store the reading

void setup() {
  Serial.begin(9600);       // start serial communication
}

void loop() {
  ldrValue = analogRead(ldrPin);  // read LDR (0–1023)
  
  int lightPercent = map(ldrValue, 0, 1023, 0, 100);  // convert to %
  
  Serial.print("Light: ");
  Serial.print(lightPercent);
  Serial.println("%");
  
  if (lightPercent < 30) {
    Serial.println("Status: LOW light");
  } else if (lightPercent < 70) {
    Serial.println("Status: MODERATE light");
  } else {
    Serial.println("Status: HIGH light");
  }

  delay(1000);
}

// 3- Servo Motors

#include <Servo.h>          // include the servo library

Servo myServo;              // create a servo object

void setup() {
  myServo.attach(9);        // servo signal wire connected to pin 9
  myServo.write(0);         // start at 0 degrees
  delay(1000);              // wait for it to reach 0 before moving
}

void loop() {
  myServo.write(90);        // turn to 90 degrees
  delay(1000);              // wait 1 second at 90°
  
  myServo.write(0);         // return back to 0 degrees
  delay(1000);              // wait 1 second at 0°
}

// 4- LCD SCreen

#include <Wire.h>                    // I2C communication library
#include <LiquidCrystal_I2C.h>      // I2C LCD library

LiquidCrystal_I2C lcd(0x27, 16, 2); // address 0x27, 16 columns, 2 rows

void setup() {
  lcd.init();                        // initialize the LCD
  lcd.backlight();                   // turn on the backlight
  
  lcd.setCursor(0, 0);              // column 0, row 0 (top left)
  lcd.print("Moisture: ");          // print text on row 1
  
  lcd.setCursor(0, 1);              // column 0, row 1 (bottom left)
  lcd.print("Light: ");             // print text on row 2
}

void loop() {
  int moisturePercent = 75;         // replace with your actual sensor value
  int lightPercent = 40;            // replace with your actual sensor value
  
  lcd.setCursor(10, 0);             // move to column 10, row 0
  lcd.print(moisturePercent);       // print moisture value
  lcd.print("%   ");                // % sign + spaces to clear old digits
  
  lcd.setCursor(7, 1);              // move to column 7, row 1
  lcd.print(lightPercent);          // print light value
  lcd.print("%   ");                // % sign + spaces to clear old digits
  
  delay(1000);
}

// 5- Relay module

const int relayPin = 7;       // relay IN connected to digital pin 7

void setup() {
  pinMode(relayPin, OUTPUT);  // set relay pin as output
  digitalWrite(relayPin, HIGH); // start with pump OFF 
}

void loop() {
  int moisturePercent = 30;   // replace your actual sensor reading !!!!!!
  
  if (moisturePercent < 40) {         // if soil is too dry
    digitalWrite(relayPin, LOW);      // turn pump ON
    Serial.println("Pump: ON");
    delay(3000);                      // run pump for 3 seconds
    digitalWrite(relayPin, HIGH);     // turn pump OFF
    Serial.println("Pump: OFF");
  }
  
  delay(1000);                        // check moisture every second
}

// TEEEEEEEEST