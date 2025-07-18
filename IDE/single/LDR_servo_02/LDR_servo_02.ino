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

Adafruit_INA219 ina219Load(0x40);   // 默认地址
Adafruit_INA219 ina219Solar(0x41);  // 改了地址的模块

unsigned long lastServoTime = 0;
unsigned long lastInaTime = 0;
const int servoInterval = 1000; // 每 1 秒判断一次是否转动舵机
const int inaInterval = 50;    // 每 200ms 读取一次功耗

float accumulatedEnergyLoad_mWh = 0;
float accumulatedEnergySolar_mWh = 0;

void setup() {
  Serial.begin(115200);
  Wire.begin();

  myServo.attach(servoPin);
  myServo.write(servoAngle);

  if (!ina219Load.begin()) {
    //Serial.println("INA219 Load not found!");
    while (1);
  }
  if (!ina219Solar.begin()) {
    //Serial.println("INA219 Solar not found!");
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

  //Serial.print("LDR Left: "); Serial.print(leftLDR);
  //Serial.print(" | LDR Right: "); Serial.print(rightLDR);
  //Serial.print(" | Diff: "); Serial.println(diff);

  if (diff > threshold && servoAngle > minAngle) {
    servoAngle -= stepAngle;
    myServo.write(servoAngle);
    //Serial.print(" ← Turn Left");
  } else if (diff < -threshold && servoAngle < maxAngle) {
    servoAngle += stepAngle;
    myServo.write(servoAngle);
    //Serial.print(" → Turn Right");
  } else {
    //Serial.print(" ○ Stay");
  }

  //Serial.print(" | Servo Angle: "); Serial.println(servoAngle);
}

// INA219 功耗读取函数
void readPowerData() {
  float vLoad = ina219Load.getBusVoltage_V();
  float iLoad = ina219Load.getCurrent_mA();
  float pLoad = vLoad * iLoad;
  accumulatedEnergyLoad_mWh += pLoad * inaInterval / (1000.0 * 3600.0);

  float vSolar = ina219Solar.getBusVoltage_V();
  float iSolar = ina219Solar.getCurrent_mA();
  float pSolar = vSolar * iSolar;
  accumulatedEnergySolar_mWh += pSolar * inaInterval / (1000.0 * 3600.0);

  //Serial.println("--- Power Monitor ---");

  //Serial.print("[LOAD]  V: "); Serial.print(vLoad, 2);
  //Serial.print(" V | I: "); Serial.print(iLoad, 2);
  //Serial.print(" mA | P: "); Serial.print(pLoad, 2);
  //Serial.print(" mW | Total: "); Serial.print(accumulatedEnergyLoad_mWh); Serial.println(" mWh");
  //Serial.print("Power_mW: ");
  Serial.print(pLoad); 
  Serial.print("\t");
  Serial.println(pSolar); 

  //Serial.print("[SOLAR] V: "); Serial.print(vSolar, 2);
  //Serial.print(" V | I: "); Serial.print(iSolar, 2);
  //Serial.print(" mA | P: "); Serial.print(pSolar, 2);
  //Serial.print(" mW | Total: "); Serial.print(accumulatedEnergySolar_mWh); Serial.println(" mWh");
}