#include <Arduino.h>
#include <AccelStepper.h>

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
const float PULSES_PER_DEG = 4.6;
const int PWM_DUTY = 200;
const int ENC_TOL = 1;
const unsigned long ENC_DEBOUNCE_US = 200;

// 状态变量
int targetDC_deg = 0;
bool dcActive = false;
bool dcDir = true; // true=正转，false=反转
unsigned long dcStartTime = 0;
const int DC_DELAY = 1000;

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

  // 步进初始化
  stepper.setMaxSpeed(500);
  stepper.setAcceleration(200);
  stepper.moveTo(0);

  // 直流驱动初始化
  pinMode(PWMA, OUTPUT);
  pinMode(AIN1, OUTPUT);
  pinMode(AIN2, OUTPUT);
  pinMode(STBY, OUTPUT);
  digitalWrite(STBY, HIGH);

  // 编码器初始化
  pinMode(ENC_A, INPUT_PULLUP);
  pinMode(ENC_B, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ENC_A), onEncoderRise, RISING);

  // 电位器初始化
  pinMode(POT_PIN, INPUT);

  Serial.println("系统初始化完成");
}

void loop() {
  unsigned long now = millis();

  // —— 读取电位器并设置目标 ——
  if (now - lastUpdate >= UPDATE_INTERVAL) {
    lastUpdate = now;

    int pot = analogRead(POT_PIN);
    if (abs(pot - lastPotValue) > DEADZONE) {
      lastPotValue = pot;

      // 步进电机目标
      long targetStepper = map(pot, 0, 1023, 0, STEPS_PER_REVOLUTION / 2) * -1;
      stepper.moveTo(targetStepper);

      // 直流电机目标
      targetDC_deg = map(pot, 0, 1023, 0, 180);
      dcStartTime = now;
      dcActive = false;  // 重置直流标志

      Serial.println("——————————————");
      Serial.print("[POT] "); Serial.print(pot);
      Serial.print(" | Stepper目标: "); Serial.print(targetStepper);
      Serial.print(" | 当前位置: "); Serial.println(stepper.currentPosition());
      Serial.print("[DC] 目标角度: "); Serial.print(targetDC_deg);
      Serial.print(" | 当前角度: ");
      Serial.println(encoderCount / PULSES_PER_DEG);
    }
  }

  // 步进电机运行
  stepper.run();

  // 启动直流电机延迟后运行
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

  // —— 若直流电机正在运行，判断是否到位 ——
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
