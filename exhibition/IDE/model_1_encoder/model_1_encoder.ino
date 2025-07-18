// === 模式一（单独版）非阻塞：步进 + 编码器直流电机 ===
// 仅按钮从 OFF → ON（LOW）时运行 1 次完整周期，期间拨回 OFF 不影响

#include <Arduino.h>
#include <AccelStepper.h>

/* ───── 0. 参数设置 ───── */
const float  PULSE_PER_DEG  = 4.4;
const int    ENC_TOL        = 2;
const int    PWM_DUTY       = 150;
const unsigned long DC_START_DELAY = 2000;
const unsigned long DWELL_MS = 1650;
const unsigned long HOLD_MS  = 3000;
const float  DC_STEP_DEG = 18.0;
const byte   DC_STEP_NUM = 10;

/* ───── 1. 步进配置 ───── */
const int STEPS_PER_REV = 2048;
AccelStepper stepper(AccelStepper::FULL4WIRE, 8, 10, 9, 11);

/* ───── 2. 直流 & 编码器 ───── */
const int PWMA = 5, AIN1 = 7, AIN2 = 4, STBY = 6;
const int ENC_A = 2, ENC_B = 3;
volatile long encCnt = 0;  volatile unsigned long encT = 0;
const unsigned long ENC_DEB_US = 200;

/* ───── 3. 开关输入 ───── */
const byte BTN_PIN = 25;

/* ───── 4. 状态变量 ───── */
bool cycleActive = false, stepFwd = true, stepReturning = false;
unsigned long tStepStart = 0, tHold = 0;
byte dcIdx = 0;
bool dcWait = false, dcBack = false, dcEnabled = false;
unsigned long tDcMark = 0;

/* ───── 5. 中断处理 ───── */
void onEncRise() {
  unsigned long t = micros();
  if (t - encT < ENC_DEB_US) return;
  encT = t;
  encCnt += digitalRead(ENC_B) ? -1 : +1;
}

inline void dcStop() {
  analogWrite(PWMA, 0);
  digitalWrite(AIN1, LOW);
  digitalWrite(AIN2, LOW);
}

/* ───── 6. 初始化 ───── */
void setup() {
  Serial.begin(115200);

  pinMode(PWMA, OUTPUT); pinMode(AIN1, OUTPUT); pinMode(AIN2, OUTPUT);
  pinMode(STBY, OUTPUT); digitalWrite(STBY, HIGH);
  pinMode(ENC_A, INPUT_PULLUP); pinMode(ENC_B, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ENC_A), onEncRise, RISING);
  pinMode(BTN_PIN, INPUT_PULLUP);

  stepper.setMaxSpeed(50);
  stepper.setAcceleration(200);
  stepper.moveTo(0);

  Serial.println(F("等待按钮从 OFF → ON 启动周期..."));
}

/* ───── 7. 主循环 ───── */
void loop() {
  static bool prevBtn = HIGH;
  bool nowBtn = digitalRead(BTN_PIN);

  if (prevBtn == HIGH && nowBtn == LOW && !cycleActive) {
    cycleActive = true;
    stepFwd = true; stepReturning = false;
    dcIdx = 0; dcWait = false; dcBack = false; dcEnabled = false;
    encCnt = 0;

    stepper.setMaxSpeed(50);
    stepper.moveTo(-STEPS_PER_REV / 2);
    tStepStart = millis();
    Serial.println(F("→ 周期开始：步进 0→180°，直流等待 3 s"));
  }
  prevBtn = nowBtn;

  if (!cycleActive) return;
  stepper.run();

  if (!dcEnabled && millis() - tStepStart >= DC_START_DELAY) {
    dcEnabled = true; tDcMark = millis();
    Serial.println(F("→ 直流开始 0→180° 步进"));
  }

  if (dcEnabled && !dcBack && dcIdx < DC_STEP_NUM) {
    float tgtDeg = (dcIdx + 1) * DC_STEP_DEG;
    long  tgtPulse = lround(tgtDeg * PULSE_PER_DEG);
    if (!dcWait) {
      if (abs(encCnt - tgtPulse) > ENC_TOL) {
        digitalWrite(AIN1, HIGH); digitalWrite(AIN2, LOW);
        analogWrite(PWMA, PWM_DUTY);
      } else {
        dcStop();
        unsigned long used = millis() - tDcMark;
        Serial.print(F("   ↳ DC 步")); Serial.print(dcIdx + 1);
        Serial.print(' '); Serial.print(tgtDeg, 0);
        Serial.print(F("° 用时 ")); Serial.print(used); Serial.println(F(" ms"));
        dcIdx++; dcWait = true; tDcMark = millis();
      }
    } else if (millis() - tDcMark >= DWELL_MS) {
      dcWait = false; tDcMark = millis();
    }
  }

  if (stepFwd && stepper.distanceToGo() == 0) {
    stepFwd = false; tHold = millis();
    Serial.println(F("⏸ 步进已达 180°，等待 3 s 顶点"));
  }
  if (!stepFwd && !stepReturning && (millis() - tHold >= HOLD_MS)) {
    stepReturning = true;
    stepper.setMaxSpeed(600); stepper.moveTo(0);
    Serial.println(F("← 步进回零..."));
  }
  if (stepReturning && stepper.distanceToGo() == 0) {
    stepReturning = false; dcBack = true;
    Serial.println(F("→ 步进已到 0°，直流归位"));
  }
  if (dcBack) {
    if (abs(encCnt) > ENC_TOL) {
      digitalWrite(AIN1, LOW); digitalWrite(AIN2, HIGH);
      analogWrite(PWMA, PWM_DUTY);
    } else {
      dcStop(); dcBack = false; cycleActive = false;
      stepper.stop(); for (int p = 8; p <= 11; ++p) digitalWrite(p, LOW);
      Serial.println(F("✔ 周期完成，等待按钮重新拨到 OFF 再启动"));
    }
  }
}
