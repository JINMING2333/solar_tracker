#include <Stepper.h>

const int stepsPerRevolution = 2048;  // 28BYJ-48 的标准步数（半步模式）

// 初始化：IN1-IN4 分别接 D2~D5
Stepper myStepper(stepsPerRevolution, 2, 4, 3, 5);

void setup() {
  myStepper.setSpeed(10); // 设置转速 RPM
  Serial.begin(9600);
  Serial.println("Stepper Test Start");
}

void loop() {
  Serial.println("Clockwise");
  myStepper.step(stepsPerRevolution);  // 正转一圈
  delay(1000);

  Serial.println("Counter-Clockwise");
  myStepper.step(-stepsPerRevolution); // 反转一圈
  delay(1000);
}