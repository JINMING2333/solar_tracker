#include <Arduino.h>
#include <AccelStepper.h>
#include <Adafruit_NeoPixel.h>

// 步进电机配置
AccelStepper stepper(AccelStepper::FULL4WIRE, 8, 10, 9, 11);
const int STEPS_PER_REVOLUTION = 2048;

// 电位器设置
const int POT_PIN = A5;
const int DEADZONE = 10;
const unsigned long UPDATE_INTERVAL = 100;
unsigned long lastUpdate = 0;
int lastPotValue = -1;

// 直流电机设置
const int PWMA = 5;
const int AIN1 = 7;
const int AIN2 = 4;
const int STBY = 6;

// 编码器设置
const int ENC_A = 2;
const int ENC_B = 3;
volatile long encoderCount = 0;
volatile unsigned long lastEncTime = 0;
const float PULSES_PER_DEG = 4.4;
const int PWM_DUTY = 200;
const int ENC_TOL = 1;
const unsigned long ENC_DEBOUNCE_US = 200;

// 状态变量
int targetDC_deg = 0;
bool dcActive = false;
bool dcDir = true; // true=正转，false=反转
unsigned long dcStartTime = 0;
const int DC_DELAY = 1000;

// LDR 和模拟太阳
const int LDR_LEFT_PIN = A0;
const int LDR_RIGHT_PIN = A1;
const int LDR_FIXED_PIN = A2;
const int SUN_MOSFET_PIN = 24;

int ldrLeft = 0;
int ldrRight = 0;
int ldrFixed = 0;

// 船型开关
const int MODE2_SWITCH_PIN = 13;
bool mode2Enabled = false;

// LED 灯条控制（使用 Adafruit NeoPixel）
#define NUMPIXELS 16
const int LED_FIXED_PIN = 50;
const int LED_SINGLE_PIN = 51;
Adafruit_NeoPixel ledFixed(NUMPIXELS, LED_FIXED_PIN, NEO_GRB + NEO_KHZ800);
Adafruit_NeoPixel ledSingle(NUMPIXELS, LED_SINGLE_PIN, NEO_GRB + NEO_KHZ800);

// 中断函数
void onEncoderRise() {
  unsigned long t = micros();
  if (t - lastEncTime < ENC_DEBOUNCE_US) return;
  lastEncTime = t;
  if (digitalRead(ENC_B) == LOW) encoderCount++;
  else                            encoderCount--;
}

void setup() {
  Serial.begin(115200);

  stepper.setMaxSpeed(500);
  stepper.setAcceleration(200);
  stepper.moveTo(0);
  stepper.enableOutputs();

  pinMode(PWMA, OUTPUT);
  pinMode(AIN1, OUTPUT);
  pinMode(AIN2, OUTPUT);
  pinMode(STBY, OUTPUT);
  digitalWrite(STBY, HIGH);

  pinMode(ENC_A, INPUT_PULLUP);
  pinMode(ENC_B, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ENC_A), onEncoderRise, RISING);

  pinMode(POT_PIN, INPUT);
  pinMode(LDR_LEFT_PIN, INPUT);
  pinMode(LDR_RIGHT_PIN, INPUT);
  pinMode(LDR_FIXED_PIN, INPUT);
  pinMode(SUN_MOSFET_PIN, OUTPUT);

  pinMode(MODE2_SWITCH_PIN, INPUT_PULLUP);

  ledFixed.begin();
  ledFixed.show();
  ledSingle.begin();
  ledSingle.show();

  Serial.println("系统初始化完成");
}

void loop() {
  mode2Enabled = digitalRead(MODE2_SWITCH_PIN) == LOW; // LOW 表示开关接通
  digitalWrite(SUN_MOSFET_PIN, mode2Enabled ? HIGH : LOW);

  if (!mode2Enabled) {
    stopMotor();
    stepper.moveTo(0);
    stepper.disableOutputs();
    
    ledFixed.clear();
    ledSingle.clear();
    ledFixed.show();
    ledSingle.show();
    return;
  } else {
    stepper.enableOutputs();
  }

  unsigned long now = millis();

  if (now - lastUpdate >= UPDATE_INTERVAL) {
    lastUpdate = now;

    int pot = analogRead(POT_PIN);
    if (abs(pot - lastPotValue) > DEADZONE) {
      lastPotValue = pot;

      long targetStepper = map(pot, 0, 1023, 0, STEPS_PER_REVOLUTION / 2) * -1;
      stepper.moveTo(targetStepper);

      targetDC_deg = map(pot, 0, 1023, 0, 180);
      dcStartTime = now;
      dcActive = false;

      Serial.println("——————————————");
      Serial.print("[POT] "); Serial.print(pot);
      Serial.print(" | Stepper目标: "); Serial.print(targetStepper);
      Serial.print(" | 当前位置: "); Serial.println(stepper.currentPosition());
      Serial.print("[DC] 目标角度: "); Serial.print(targetDC_deg);
      Serial.print(" | 当前角度: ");
      Serial.println(encoderCount / PULSES_PER_DEG);
    }

    ldrLeft = analogRead(LDR_LEFT_PIN);
    ldrRight = analogRead(LDR_RIGHT_PIN);
    ldrFixed = analogRead(LDR_FIXED_PIN);
    Serial.print("[LDR] 左: "); Serial.print(ldrLeft);
    Serial.print(" | 右: "); Serial.print(ldrRight);
    Serial.print(" | 固定: "); Serial.println(ldrFixed);

    int ldrMax = max(ldrLeft, ldrRight);
    int ledCountFixed = map(ldrFixed, 0, 1023, 0, NUMPIXELS);
    int ledCountSingle = map(ldrMax, 0, 1023, 0, NUMPIXELS);

    for (int i = 0; i < NUMPIXELS; i++) {
      ledFixed.setPixelColor(i, i < ledCountFixed ? ledFixed.Color(255, 255, 0) : 0);
      ledSingle.setPixelColor(i, i < ledCountSingle ? ledSingle.Color(255, 255, 0) : 0);
    }
    ledFixed.show();
    ledSingle.show();
  }

  stepper.run();

  if (!dcActive && millis() - dcStartTime >= DC_DELAY) {
    long targetPulses = lround(targetDC_deg * PULSES_PER_DEG);
    long diff = targetPulses - encoderCount;

    if (abs(diff) > ENC_TOL) {
      dcDir = diff > 0;
      if (dcDir) {
        digitalWrite(AIN1, HIGH);
        digitalWrite(AIN2, LOW);
      } else {
        digitalWrite(AIN1, LOW);
        digitalWrite(AIN2, HIGH);
      }
      analogWrite(PWMA, PWM_DUTY);
      dcActive = true;
    } else {
      stopMotor();
    }
  }

  if (dcActive) {
    long targetPulses = lround(targetDC_deg * PULSES_PER_DEG);
    long diff = targetPulses - encoderCount;

    Serial.print("[DC] 当前角度: ");
    Serial.println(encoderCount / PULSES_PER_DEG);

    if (abs(diff) <= ENC_TOL) {
      stopMotor();
      dcActive = false;
      Serial.print("→ [DC] 到达目标角度: ");
      Serial.println(targetDC_deg);
    }
  }
}

void stopMotor() {
  analogWrite(PWMA, 0);
  digitalWrite(AIN1, LOW);
  digitalWrite(AIN2, LOW);
}
