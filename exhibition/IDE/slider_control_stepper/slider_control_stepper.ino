#include <AccelStepper.h>

// 步进电机配置：模式 4 = FULL4WIRE
AccelStepper stepper(AccelStepper::FULL4WIRE, 8, 10, 9, 11);

// 常量设置
const int potPin = A5;            // 滑动电位器连接 A0
const int stepsPerRev = 2048;     // 28BYJ-48 全步一圈
const int maxAngle = 180;         // 最大角度
const int deadZone = 10;           // 电位器变化阈值（死区）
const unsigned long updateInterval = 100; // 最快每 100ms 更新一次

// 状态变量
int lastPotValue = -1;
unsigned long lastUpdateTime = 0;

void setup() {
  Serial.begin(115200);

  stepper.setMaxSpeed(500);      // 最大速度（步/秒）
  stepper.setAcceleration(200);  // 加速度（步/秒^2）

  Serial.println("电位器控制步进电机初始化完成");
}

void loop() {
  // 当前时间
  unsigned long now = millis();

  // 满足更新间隔时，检查电位器是否有明显变化
  if (now - lastUpdateTime > updateInterval) {
    lastUpdateTime = now;

    int potValue = analogRead(potPin);
    if (abs(potValue - lastPotValue) > deadZone) {
      lastPotValue = potValue;

      // 将电位器值映射到步进电机角度范围（0° ~ 180° -> 50 ~ 1000）
      long targetSteps = map(potValue, 50, 1000, 0, stepsPerRev / 2);

      stepper.moveTo(targetSteps);

      Serial.print("电位器值：");
      Serial.print(potValue);
      Serial.print(" → 电机目标位置：");
      Serial.println(targetSteps);
    }
  }

  // 必须持续调用 run() 来保持运动
  stepper.run();
}
