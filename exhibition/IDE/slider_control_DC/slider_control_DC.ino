#include <Arduino.h>

// —— 1. 驱动引脚 ——
const int PWMA = 5;   // TB6612 PWM 控速
const int AIN1 = 7;
const int AIN2 = 4;
const int STBY = 6;   // 使能

// —— 2. 编码器引脚 ——
const int ENC_A = 2;  // 编码器 A 相
const int ENC_B = 3;  // 编码器 B 相

// —— 3. 电位器（滑动变位器）——
const int POT_PIN = A5; // 连接至滑动电位器输出

// —— 4. 参数 ——
const float PULSES_PER_DEG = 2.436;     // 每度脉冲数
const int PWM_DUTY        = 120;       // 电机供电占空比
const int ENC_TOL         = 1;         // 容差脉冲数
const unsigned long ENC_DEBOUNCE_US = 200; // 编码器去抖间隔

// —— 5. 全局变量 ——
volatile long encoderCount = 0;
volatile unsigned long lastEncTime = 0;

// —— 6. 编码器中断 ——
void onEncoderRise() {
  unsigned long t = micros();
  if (t - lastEncTime < ENC_DEBOUNCE_US) return;
  lastEncTime = t;
  if (digitalRead(ENC_B) == LOW) encoderCount++;
  else                            encoderCount--;
}

void setup() {
  Serial.begin(115200);

  // 驱动使能
  pinMode(STBY, OUTPUT);
  digitalWrite(STBY, HIGH);
  pinMode(PWMA, OUTPUT);
  pinMode(AIN1, OUTPUT);
  pinMode(AIN2, OUTPUT);

  // 编码器输入
  pinMode(ENC_A, INPUT_PULLUP);
  pinMode(ENC_B, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ENC_A), onEncoderRise, RISING);

  // 滑动变位器
  pinMode(POT_PIN, INPUT);

  Serial.println("Ready: slider -> DC motor position");
}

void loop() {
  // 读取电位器，映射到 0~180 度
  int pot = analogRead(POT_PIN);
  int targetDeg = map(pot, 0, 1023, 0, 180);
  Serial.print("Pot: "); Serial.print(pot);
  Serial.print(" -> Target: "); Serial.print(targetDeg);
  Serial.print("°, Encoder: "); Serial.println(encoderCount);

  // 驱动至目标角度
  rotateToAngle(targetDeg);

  delay(100); // 每 100ms 更新一次
}

// —— 根据目标角度驱动电机 ——
void rotateToAngle(int angleDeg) {
  long targetPulses = lround(angleDeg * PULSES_PER_DEG);
  long diff = targetPulses - encoderCount;
  if (abs(diff) <= ENC_TOL) {
    stopMotor();
    return;
  }
  // 选择旋转方向
  if (diff > 0) {
    digitalWrite(AIN1, HIGH);
    digitalWrite(AIN2, LOW);
  } else {
    digitalWrite(AIN1, LOW);
    digitalWrite(AIN2, HIGH);
  }
  analogWrite(PWMA, PWM_DUTY);

  // 持续驱动直至到位
  while (abs(targetPulses - encoderCount) > ENC_TOL) {
    ;
  }
  stopMotor();
  Serial.print("Reached "); Serial.print(angleDeg); Serial.println("°");
}

void stopMotor() {
  analogWrite(PWMA, 0);
  digitalWrite(AIN1, LOW);
  digitalWrite(AIN2, LOW);
}