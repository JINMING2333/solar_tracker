#include <Servo.h>

Servo myServo;

void setup() {
  myServo.attach(6); // D6
  myServo.write(0);  // 设为起始角度
  delay(1000);
  myServo.write(0); // 转动至 90 度
}

void loop() {
}