#include <Servo.h>
#include <Wire.h>
#include <Adafruit_INA219.h>

//////////////////// 用户可调参数 ////////////////////
const uint8_t SERVO_PIN   = 5;
const uint16_t STEP_DEG   = 12;
const uint8_t  NUM_STEPS  = 10;
const uint32_t DWELL_MS   = 30000;
const uint32_t FINAL_DWELL_MS = 60000;
const uint16_t START_DEG  = 30;
uint16_t currentAngle = START_DEG;  // 初始化为起始角度
//////////////////////////////////////////////////////

Servo myServo;
Adafruit_INA219 ina219;

float accum_mWh = 0.0;
uint8_t currentStep = 0;
unsigned long stepStart = 0;
bool movingPhase = true;
bool looping = false;
unsigned long finalStepDoneTime = 0;

// 采样记录
unsigned long lastSample = 0;

void setup() {
  Serial.begin(115200);
  Wire.begin();

  if (!ina219.begin()) {
    //Serial.println(F("INA219 not found - check wiring!"));
    while (1);
  }
  ina219.setCalibration_32V_2A();

  myServo.attach(SERVO_PIN);
  myServo.write(START_DEG);
  delay(500);

  // 等待 Python 发来的 "START" 命令
  //Serial.println("Waiting for START...");
  while (!Serial.available()) {}
  String cmd = Serial.readStringUntil('\n');
  if (cmd != "START") {
    //Serial.println("Invalid command");
    while (1);  // 错误命令则停机
  }
  //Serial.println("Starting...");

  stepStart = millis();
  movingPhase = false;
}

void loop() {
  static unsigned long lastPrint = 0;
  unsigned long now = millis();

  /************ 实时采样 INA219 ************/
  float busV = ina219.getBusVoltage_V();
  float cur  = ina219.getCurrent_mA();
  float p_mW = busV * cur;

  unsigned long delta_t = now - lastSample;
  if (delta_t > 0 && delta_t < 1000) {  // 避免 millis() 回绕误差
    accum_mWh += p_mW * delta_t / 3600000.0;
  }
  lastSample = now;

  uint16_t currentAngle = START_DEG + currentStep * STEP_DEG;

  if (now - lastPrint >= 50) {  // 限制为 20Hz 输出
    lastPrint = now;

    Serial.print(now);
    Serial.print(",");

    Serial.print(currentAngle);
    Serial.print(",");

    Serial.print(busV, 2);
    Serial.print(",");

    Serial.print(cur, 1);
    Serial.print(",");

    Serial.println(p_mW, 2);
  }
  //Serial.print(",");

  // Serial.print(accum_mWh, 4);
  // Serial.println(" mWh");

  /************ 舵机控制状态机 ************/
  if (looping) {
    if (now - finalStepDoneTime >= FINAL_DWELL_MS) {
      myServo.write(START_DEG);
      delay(500);
      currentStep = 0;
      stepStart = millis();
      movingPhase = false;
      looping = false;
      //Serial.println(F("---- 新一轮开始 ----"));
    }
    return;
  }

  if (!movingPhase && (now - stepStart >= DWELL_MS)) {
    currentStep++;
    if (currentStep <= NUM_STEPS) {
      uint16_t target = START_DEG + currentStep * STEP_DEG;
      myServo.write(target);
      movingPhase = true;
      stepStart = now;
    }
    if (currentStep == NUM_STEPS) {
      looping = true;
      finalStepDoneTime = now;
    }
  }

  if (movingPhase && (now - stepStart > 200)) {
    movingPhase = false;
    stepStart = now;
  }
}
