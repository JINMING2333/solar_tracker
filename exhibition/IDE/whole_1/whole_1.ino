// === 模式一 + 模式二 集成版本（共享电机）===
// 修复版：使用用户验证可运行的模式二代码
//   • D13 控制模式一（带编码器步进+直流）运行一次
//   • D12 控制模式二（电位器控制角度 + LDR + LED）运行/关闭
//   • 两个模式使用相同的步进与直流电机（共用引脚）

#include <Arduino.h>
#include <AccelStepper.h>
#include <Adafruit_NeoPixel.h>

// 步进电机配置
AccelStepper stepper(AccelStepper::FULL4WIRE, 8, 10, 9, 11);
const int STEPS_PER_REV = 2048;

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
bool dcDir = true;
unsigned long dcStartTime = 0;
const int DC_DELAY = 1000;

// LDR 和模拟太阳
const int LDR_LEFT_PIN = A0;
const int LDR_RIGHT_PIN = A1;
const int LDR_FIXED_PIN = A2;
const int SUN_MOSFET_PIN = 24;

// 船型开关
const int BTN1_PIN = 25;
const int BTN2_PIN = 13;
bool mode1Active = false;
bool mode2Enabled = false;

// LED 灯条控制
#define NUMPIXELS 16
const int LED_FIXED_PIN = 50;
const int LED_SINGLE_PIN = 51;
Adafruit_NeoPixel ledFixed(NUMPIXELS, LED_FIXED_PIN, NEO_GRB + NEO_KHZ800);
Adafruit_NeoPixel ledSingle(NUMPIXELS, LED_SINGLE_PIN, NEO_GRB + NEO_KHZ800);

// 模式一状态
bool stepFwd = true;
bool stepReturning = false;
unsigned long tStepStart = 0, tHold = 0;
const unsigned long HOLD_MS = 3000;
const unsigned long DC_START_DELAY = 2000;
const unsigned long DWELL_MS = 1650;
const float DC_STEP_DEG = 18.0;
const byte DC_STEP_NUM = 10;
byte dcIdx = 0;
bool dcWait = false, dcBack = false, dcEnabled = false;

void stopMotor() {
  analogWrite(PWMA, 0);
  digitalWrite(AIN1, LOW);
  digitalWrite(AIN2, LOW);
}

void onEncoderRise() {
  unsigned long t = micros();
  if (t - lastEncTime < ENC_DEBOUNCE_US) return;
  lastEncTime = t;
  encoderCount += digitalRead(ENC_B) == LOW ? 1 : -1;
}

void setup() {
  Serial.begin(115200);
  pinMode(PWMA, OUTPUT); pinMode(AIN1, OUTPUT); pinMode(AIN2, OUTPUT);
  pinMode(STBY, OUTPUT); digitalWrite(STBY, HIGH);
  pinMode(ENC_A, INPUT_PULLUP); pinMode(ENC_B, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ENC_A), onEncoderRise, RISING);

  stepper.setMaxSpeed(500);
  stepper.setAcceleration(200);
  stepper.setCurrentPosition(0);
  stepper.moveTo(0);
  stepper.enableOutputs();

  pinMode(POT_PIN, INPUT);
  pinMode(LDR_LEFT_PIN, INPUT);
  pinMode(LDR_RIGHT_PIN, INPUT);
  pinMode(LDR_FIXED_PIN, INPUT);
  pinMode(SUN_MOSFET_PIN, OUTPUT);

  pinMode(BTN1_PIN, INPUT_PULLUP);
  pinMode(BTN2_PIN, INPUT_PULLUP);

  ledFixed.begin(); ledFixed.show();
  ledSingle.begin(); ledSingle.show();

  Serial.println("系统初始化完成");
}

void loop() {
  static bool prevBtn1 = HIGH;
  bool btn1 = digitalRead(BTN1_PIN);
  bool btn2 = digitalRead(BTN2_PIN);

  if (prevBtn1 == HIGH && btn1 == LOW && !mode1Active && !mode2Enabled) {
    mode1Active = true;
    stepFwd = true; stepReturning = false;
    dcIdx = 0; dcWait = false; dcBack = false; dcEnabled = false;
    encoderCount = 0;
    stepper.setMaxSpeed(50);
    stepper.moveTo(-STEPS_PER_REV / 2);
    tStepStart = millis();
    Serial.println("→ 模式一启动");
  }
  prevBtn1 = btn1;

  mode2Enabled = (btn2 == LOW);
  digitalWrite(SUN_MOSFET_PIN, mode2Enabled ? HIGH : LOW);

  if (mode1Active) {
    stepper.run();
    if (!dcEnabled && millis() - tStepStart >= DC_START_DELAY) {
      dcEnabled = true; dcStartTime = millis();
    }
    if (dcEnabled && !dcBack && dcIdx < DC_STEP_NUM) {
      float tgtDeg = (dcIdx + 1) * DC_STEP_DEG;
      long tgtPulse = lround(tgtDeg * PULSES_PER_DEG);
      if (!dcWait) {
        if (abs(encoderCount - tgtPulse) > ENC_TOL) {
          digitalWrite(AIN1, HIGH); digitalWrite(AIN2, LOW);
          analogWrite(PWMA, PWM_DUTY);
        } else {
          stopMotor(); dcIdx++; dcWait = true; dcStartTime = millis();
        }
      } else if (millis() - dcStartTime >= DWELL_MS) {
        dcWait = false; dcStartTime = millis();
      }
    }
    if (stepFwd && stepper.distanceToGo() == 0) {
      stepFwd = false; tHold = millis();
    }
    if (!stepFwd && !stepReturning && (millis() - tHold >= HOLD_MS)) {
      stepReturning = true; stepper.setMaxSpeed(600); stepper.moveTo(0);
    }
    if (stepReturning && stepper.distanceToGo() == 0) {
      stepReturning = false; dcBack = true;
    }
    if (dcBack) {
      if (abs(encoderCount) > ENC_TOL) {
        digitalWrite(AIN1, LOW); digitalWrite(AIN2, HIGH);
        analogWrite(PWMA, PWM_DUTY);
      } else {
        stopMotor(); dcBack = false; mode1Active = false;
        stepper.stop(); for (int p=8; p<=11; ++p) digitalWrite(p, LOW);
        Serial.println("✔ 模式一完成");
      }
    }
    return;
  }

  if (mode2Enabled) {
    stepper.enableOutputs();
    unsigned long now = millis();
    if (now - lastUpdate >= UPDATE_INTERVAL) {
      lastUpdate = now;
      int pot = analogRead(POT_PIN);
      if (abs(pot - lastPotValue) > DEADZONE) {
        lastPotValue = pot;

        long targetStepper = map(pot, 0, 1023, 0, STEPS_PER_REV / 2) * -1;
        if (stepper.distanceToGo() != targetStepper) {
          stepper.moveTo(targetStepper);
        }

        targetDC_deg = map(pot, 0, 1023, 0, 180);
        dcStartTime = now;
        dcActive = false;
      }

      int ldrLeft = analogRead(LDR_LEFT_PIN);
      int ldrRight = analogRead(LDR_RIGHT_PIN);
      int ldrFixed = analogRead(LDR_FIXED_PIN);
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
          digitalWrite(AIN1, HIGH); digitalWrite(AIN2, LOW);
        } else {
          digitalWrite(AIN1, LOW); digitalWrite(AIN2, HIGH);
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
      if (abs(diff) <= ENC_TOL) {
        stopMotor();
        dcActive = false;
      }
    }
  } else {
    stopMotor();
    stepper.moveTo(0);
    ledFixed.clear();
    ledSingle.clear();
    ledFixed.show();
    ledSingle.show();
  }
}
