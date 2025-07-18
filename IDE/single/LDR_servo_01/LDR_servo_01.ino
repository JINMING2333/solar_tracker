#include <Servo.h>
#include <Wire.h>
#include <Adafruit_INA219.h>

const int ldrLeftPin = A0;    // 左侧 LDR
const int ldrRightPin = A1;   // 右侧 LDR
const int servoPin = 6;

const int threshold = 100;     // 光照差异阈值（可调）
int servoAngle = 30;          // 初始舵机角度
const int stepAngle = 5;
const int minAngle = 0;
const int maxAngle = 180;

Servo myServo;

Adafruit_INA219 ina219;

unsigned long lastServoTime = 0;
unsigned long lastInaTime = 0;
const int servoInterval = 1000; // 每 1 秒判断一次是否转动舵机
const int inaInterval = 100;    // 每 200ms 读取一次功耗

float accumulatedEnergy_mWh = 0;

void setup() {
  Serial.begin(9600);
  Wire.begin();

  myServo.attach(servoPin);
  myServo.write(servoAngle);

  if (!ina219.begin()) {
    Serial.println("Failed to find INA219!");
    while (1);
  }

  delay(1000);
}

void loop() {
  unsigned long now = millis();

  // 舵机控制（基于光敏电阻差值）
  if (now - lastServoTime >= servoInterval) {
    lastServoTime = now;
    controlServoByLDR();
  }

  // 读取电压/电流/功率
  if (now - lastInaTime >= inaInterval) {
    lastInaTime = now;
    readPowerData();
  }
}

// 舵机控制函数
void controlServoByLDR() {
  int leftLDR = analogRead(ldrLeftPin);
  int rightLDR = analogRead(ldrRightPin);
  int diff = leftLDR - rightLDR;

  Serial.print("LDR Left: "); Serial.print(leftLDR);
  Serial.print(" | LDR Right: "); Serial.print(rightLDR);
  Serial.print(" | Diff: "); Serial.println(diff);

  if (diff > threshold && servoAngle > minAngle) {
    servoAngle -= stepAngle;
    myServo.write(servoAngle);
    Serial.print(" ← Turn Left");
  } else if (diff < -threshold && servoAngle < maxAngle) {
    servoAngle += stepAngle;
    myServo.write(servoAngle);
    Serial.print(" → Turn Right");
  } else {
    Serial.print(" ○ Stay");
  }

  Serial.print(" | Servo Angle: "); Serial.println(servoAngle);
}

// INA219 功耗读取函数
void readPowerData() {
  float voltage = ina219.getBusVoltage_V();
  float current = ina219.getCurrent_mA();
  float power = ina219.getPower_mW();
  accumulatedEnergy_mWh += power * inaInterval / 3600000.0; // 换算 mWh

  Serial.print("   [INA219] V: ");
  Serial.print(voltage, 2);
  Serial.print(" V | I: ");
  Serial.print(current, 2);
  Serial.print(" mA | P: ");
  Serial.print(power, 2);
  Serial.print(" mW");
  Serial.print(" | Total Energy: "); Serial.print(accumulatedEnergy_mWh); Serial.println(" mWh");
} 