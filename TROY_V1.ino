// ============================================================
//  نظام التظليل الذكي للنبات - Fuzzy Logic Control
//  Arduino Uno + LDR Sensor + 3 Servo Motors (Shading Layers)
// ============================================================
//
//  الدوائر الكهربائية / Wiring:
//  - LDR: بين A0 و GND مع مقاومة 10kΩ إلى 5V
//  - Servo 1 (طبقة 1): Pin 9
//  - Servo 2 (طبقة 2): Pin 10
//  - Servo 3 (طبقة 3): Pin 11
//  - LED أخضر (نبات آمن): Pin 4
//  - LED أصفر (تحذير): Pin 5
//  - LED أحمر (خطر): Pin 6
// ============================================================

#include <Servo.h>

// --- تعريف الأجهزة ---
Servo layer1, layer2, layer3;

const int LDR_PIN   = A0;
const int LED_GREEN = 4;
const int LED_YELLOW= 5;
const int LED_RED   = 6;

// زوايا المحرك
const int OPEN   = 0;    // طبقة مفتوحة (لا تظليل)
const int CLOSED = 90;   // طبقة مغلقة (تظليل كامل)

// --- متغيرات الحالة ---
float lightRaw    = 0;
float lightNorm   = 0;   // 0.0 إلى 1.0
float shadingOut  = 0;   // 0.0 إلى 3.0 (عدد الطبقات الفعّالة)

// ============================================================
//  دوال عضوية Fuzzy - Membership Functions
// ============================================================

// مثلث: trapezoid membership
float trapMF(float x, float a, float b, float c, float d) {
  if (x <= a || x >= d) return 0.0;
  if (x >= b && x <= c) return 1.0;
  if (x < b)  return (x - a) / (b - a);
  return (d - x) / (d - c);
}

float triMF(float x, float a, float b, float c) {
  if (x <= a || x >= c) return 0.0;
  if (x <= b) return (x - a) / (b - a);
  return (c - x) / (c - b);
}

// --- مجموعات Fuzzy للإدخال (شدة الضوء 0→1) ---
float veryLow (float x) { return trapMF(x, 0.0, 0.0,  0.15, 0.30); }
float low     (float x) { return triMF (x, 0.15, 0.30, 0.50); }
float medium  (float x) { return triMF (x, 0.35, 0.50, 0.70); }
float high    (float x) { return triMF (x, 0.55, 0.70, 0.85); }
float veryHigh(float x) { return trapMF(x, 0.75, 0.85, 1.0,  1.0); }

// ============================================================
//  محرك Fuzzy Inference (Mamdani - Centroid defuzz)
// ============================================================
float fuzzyInference(float light) {
  // درجات الانتماء
  float mVL = veryLow (light);
  float mL  = low     (light);
  float mM  = medium  (light);
  float mH  = high    (light);
  float mVH = veryHigh(light);

  // القواعد → إخراج مرجّح
  // الإخراج: 0=لا تظليل, 1=طبقة1, 2=طبقتان, 3=ثلاث طبقات
  // نستخدم centroid بسيط على نقاط مركز
  float num = mVL * 0.0
            + mL  * 1.0
            + mM  * 2.0
            + mH  * 2.5
            + mVH * 3.0;

  float den = mVL + mL + mM + mH + mVH;

  if (den < 0.001) return 0.0;
  return num / den;
}

// ============================================================
//  تطبيق قرار الطبقات
// ============================================================
void applyShading(float shade) {
  // shade: 0.0 → 3.0

  if (shade < 0.8) {
    // لا تظليل
    layer1.write(OPEN); layer2.write(OPEN); layer3.write(OPEN);
    digitalWrite(LED_GREEN, HIGH);
    digitalWrite(LED_YELLOW, LOW);
    digitalWrite(LED_RED, LOW);
  }
  else if (shade < 1.6) {
    // طبقة واحدة
    layer1.write(CLOSED); layer2.write(OPEN); layer3.write(OPEN);
    digitalWrite(LED_GREEN, LOW);
    digitalWrite(LED_YELLOW, HIGH);
    digitalWrite(LED_RED, LOW);
  }
  else if (shade < 2.4) {
    // طبقتان
    layer1.write(CLOSED); layer2.write(CLOSED); layer3.write(OPEN);
    digitalWrite(LED_GREEN, LOW);
    digitalWrite(LED_YELLOW, HIGH);
    digitalWrite(LED_RED, HIGH);
  }
  else {
    // ثلاث طبقات
    layer1.write(CLOSED); layer2.write(CLOSED); layer3.write(CLOSED);
    digitalWrite(LED_GREEN, LOW);
    digitalWrite(LED_YELLOW, LOW);
    digitalWrite(LED_RED, HIGH);
  }
}

// ============================================================
//  Setup
// ============================================================
void setup() {
  Serial.begin(9600);

  layer1.attach(9);
  layer2.attach(10);
  layer3.attach(11);

  pinMode(LED_GREEN,  OUTPUT);
  pinMode(LED_YELLOW, OUTPUT);
  pinMode(LED_RED,    OUTPUT);

  // موضع أولي: مفتوح كلياً
  layer1.write(OPEN);
  layer2.write(OPEN);
  layer3.write(OPEN);

  Serial.println(F("=== نظام التظليل الذكي - Fuzzy Plant Shading ==="));
  Serial.println(F("Raw_ADC | Light(%) | Fuzzy_Out | Layers"));
  Serial.println(F("--------|----------|-----------|-------"));
}

// ============================================================
//  Loop
// ============================================================
void loop() {
  // قراءة متوسط 10 قراءات لتقليل الضجيج
  long sum = 0;
  for (int i = 0; i < 10; i++) {
    sum += analogRead(LDR_PIN);
    delay(10);
  }
  lightRaw  = sum / 10.0;

  // تطبيع: LDR في الضوء الساطع → قيمة عالية
  lightNorm = lightRaw / 1023.0;

  // Fuzzy inference
  shadingOut = fuzzyInference(lightNorm);

  // تطبيق التظليل
  applyShading(shadingOut);

  // طباعة للـ Serial Monitor
  int layers = (shadingOut < 0.8) ? 0
             : (shadingOut < 1.6) ? 1
             : (shadingOut < 2.4) ? 2 : 3;

  Serial.print((int)lightRaw);
  Serial.print(F("\t| "));
  Serial.print(lightNorm * 100.0, 1);
  Serial.print(F("%\t| "));
  Serial.print(shadingOut, 2);
  Serial.print(F("\t   | "));
  Serial.println(layers);

  delay(500);
}
