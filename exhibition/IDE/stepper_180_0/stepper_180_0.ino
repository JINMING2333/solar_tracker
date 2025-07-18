#include <AccelStepper.h>

// 28BYJ-48 一圈 2048 步
const int stepsPerRevolution = 2048;

// AccelStepper 初始化：模式 4 = 4线单步电机
AccelStepper stepper(AccelStepper::FULL4WIRE, 8, 10, 9, 11);

void setup() {
  Serial.begin(9600);

  // 设置最大速度（步/秒）和加速度（步/秒^2）
  stepper.setMaxSpeed(1000);       // 最快运行速度
  stepper.setAcceleration(200);    // 加速/减速斜率

  Serial.println("准备开始...");
}

void loop() {
  // === 慢速从 0° 转到 180° ===
  stepper.setMaxSpeed(50);  // 慢速

  unsigned long startTime = millis();  // 开始时间
  stepper.moveTo(-stepsPerRevolution / 2);  // 目标：-1024

  while (stepper.distanceToGo() != 0) {
    stepper.run();
  }
  unsigned long endTime = millis();  // 结束时间
  Serial.print("缓慢转到 180° 耗时：");
  Serial.print((endTime - startTime) / 1000.0);
  Serial.println(" 秒");

  delay(3000);

  // === 快速返回 0° ===
  stepper.setMaxSpeed(600);  // 快速返回

  startTime = millis();
  stepper.moveTo(0);  // 回到原点

  while (stepper.distanceToGo() != 0) {
    stepper.run();
  }
  endTime = millis();
  Serial.print("快速返回 0° 耗时：");
  Serial.print((endTime - startTime) / 1000.0);
  Serial.println(" 秒");

  delay(5000);  // 等待后再次循环
}