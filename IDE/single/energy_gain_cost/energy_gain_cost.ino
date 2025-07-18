#include <Servo.h>

const int ldrLeftPin = A1;    // 左侧 LDR
const int ldrRightPin = A0;   // 右侧 LDR
const int servoPin = 6;

const int threshold = 50;     // 光照差异阈值（可调）
int servoAngle = 90;          // 初始舵机角度
const int stepAngle = 5;
const int minAngle = 0;
const int maxAngle = 180;

Servo myServo;

void setup() {
  Serial.begin(115200);
  myServo.attach(servoPin);
  myServo.write(servoAngle);
  delay(1000);
}

void loop() {
  int leftValue = analogRead(ldrLeftPin);
  int rightValue = analogRead(ldrRightPin);

  int diff = leftValue - rightValue;

  Serial.print("LDR Left: ");
  Serial.print(leftValue);
  Serial.print(" | LDR Right: ");
  Serial.print(rightValue);
  Serial.print(" | Diff: ");
  Serial.println(diff);

  // 光从右边来，转向右
  if (diff < -threshold && servoAngle < maxAngle) {
    servoAngle += stepAngle;
    myServo.write(servoAngle);
    Serial.println("→ Turn Right");
    delay(500);
  }
  // 光从左边来，转向左
  else if (diff > threshold && servoAngle > minAngle) {
    servoAngle -= stepAngle;
    myServo.write(servoAngle);
    Serial.println("← Turn Left");
    delay(500);
  }

  delay(1000);
}
