#include <Stepper.h>

// 步进电机参数
const int stepsPerRevolution = 2048;
Stepper myStepper(stepsPerRevolution, 8, 10, 9, 11);  // IN1,IN3,IN2,IN4

// TB6612 控制引脚
const int PWMA = 5;  // PWM口，必须是 ~5、~6、~9、~10、~11 等
const int AIN1 = 7;
const int AIN2 = 4;
const int STBY = 6;

// 按钮控制
const int ButtonPin1 = 2;
// bool mode1Active = false;
// bool lastButtonState = HIGH;

void setup() {
  Serial.begin(9600);

  // // 步进电机速度设置
  // myStepper.setSpeed(5);

  // 电机驱动引脚
  pinMode(PWMA, OUTPUT);
  pinMode(AIN1, OUTPUT);
  pinMode(AIN2, OUTPUT);
  pinMode(STBY, OUTPUT);

  digitalWrite(STBY, HIGH);  // 初始关闭直流电机驱动

  // 按钮设置
  pinMode(ButtonPin1, INPUT_PULLUP);

  Serial.println("等待模式一按钮...");
}

void loop() {
  // 检查按钮按下（下降沿检测）
  bool buttonPressed = digitalRead(ButtonPin1) == LOW;

  if (buttonPressed) {
    Serial.println("模式一: 启动");
    // digitalWrite(STBY, HIGH); // 启动直流电机驱动器
    delay(300);  // 消抖

    // 步进电机顺时针半圈
    myStepper.setSpeed(5);
    myStepper.step(stepsPerRevolution / 2);
    Serial.println("→ 步进电机顺转 180°");

    // 启动直流电机
    digitalWrite(AIN1, HIGH);
    digitalWrite(AIN2, LOW);
    analogWrite(PWMA, 80);  // 控制直流电机转速
    Serial.println("↑ 直流电机正转");

    delay(3000); // 展示时间

    // 步进电机逆时针回到原位
    myStepper.setSpeed(15);
    myStepper.step(-stepsPerRevolution / 2);
    Serial.println("← 步进电机反转 180°");

    // 直流电机反转
    digitalWrite(AIN1, LOW);
    digitalWrite(AIN2, HIGH);
    analogWrite(PWMA, 80);
    Serial.println("↓ 直流电机反转");

    delay(3000);

    // 停止直流电机
    analogWrite(PWMA, 0);
    Serial.println("直流电机停止");

    delay(6000);

  } else {
    // 没有按下按钮
    // digitalWrite(STBY, LOW); // 关闭电机驱动
  }

  delay(500);  // 每半秒检查一次按钮状态
}
