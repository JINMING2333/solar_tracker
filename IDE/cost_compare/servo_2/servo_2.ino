#include <Wire.h>
#include <Servo.h>
#include <Adafruit_INA219.h>

// 用户可调参数
const int SERVO_PIN       = 5;      // 接伺服的 PWM 引脚
const uint16_t STEP_DEG   = 12;     // 每步角度
const uint8_t  NUM_STEPS  = 10;     // 共 10 步
const uint32_t DWELL_MS   = 30000;  // 每步驻停 30 s
const uint32_t FINAL_DWELL_MS = 60000; // 末步驻停 60 s
const uint16_t START_DEG  = 30;     // 起始角度
const uint16_t MOVE_DELAY = 200;    // 运动后 200 ms 确保到位

// 采样参数
const uint32_t SAMPLE_MS  = 50;     // INA219 采样周期 20 Hz

Servo myServo;
Adafruit_INA219 ina219;

// 状态机变量
enum State {
  STATE_INITIAL_DWELL,   // 首次驻停
  STATE_MOVE,            // 伺服运动
  STATE_DWELL,           // 中间驻停
  STATE_FINAL_DWELL,     // 末步驻停
  STATE_RETURN_MOVE      // 回到起始
};
State state = STATE_INITIAL_DWELL;

uint8_t currentStep = 0;      // 已完成步数
unsigned long stateStart = 0; // 本阶段开始时间
unsigned long lastSample   = 0;
unsigned long lastPrint    = 0;
float accum_mWh = 0.0;

void setup() {
  Serial.begin(115200);
  Wire.begin();
  // INA219 初始化
  if (!ina219.begin()) while (1);
  ina219.setCalibration_32V_2A();

  // 伺服上电并写入起始角度
  myServo.attach(SERVO_PIN);
  myServo.write(START_DEG);
  delay(500);                      // 等舵机到位
  myServo.detach();                // 切电，进入初次驻停

  // 等待 START
  while (!Serial.available()) {}
  if (Serial.readStringUntil('\n') != "START") while (1);

  stateStart = millis();
}

void loop() {
  unsigned long now = millis();
  // —— 始终采样输出 INA219 ——  
  sampleAndPrint(now);

  // —— 状态机 ——  
  switch(state) {
    // 初始驻停（与中间驻停逻辑相同，仅运行一次）
    case STATE_INITIAL_DWELL:
      if (now - stateStart >= DWELL_MS) {
        currentStep = 0;
        transitionToMove();
      }
      break;

    // 执行单步运动
    case STATE_MOVE:
      // 等待伺服到位
      if (now - stateStart >= MOVE_DELAY) {
        myServo.detach();         // 到位后切电
        state = STATE_DWELL;
        stateStart = now;
      }
      break;

    // 驻停
    case STATE_DWELL:
      if (now - stateStart >= DWELL_MS) {
        if (currentStep < NUM_STEPS) {
          transitionToMove();
        } else {
          // 全部步完成 -> 末步驻停
          state = STATE_FINAL_DWELL;
          stateStart = now;
        }
      }
      break;

    // 末步驻停 60s
    case STATE_FINAL_DWELL:
      if (now - stateStart >= FINAL_DWELL_MS) {
        // 回到起始角度
        myServo.attach(SERVO_PIN);
        myServo.write(START_DEG);
        state = STATE_RETURN_MOVE;
        stateStart = now;
      }
      break;

    // 回转到起始后，等待 200 ms 到位，再切电准备下一循环
    case STATE_RETURN_MOVE:
      if (now - stateStart >= MOVE_DELAY) {
        myServo.detach();
        // 重置，重新开始循环
        state = STATE_INITIAL_DWELL;
        stateStart = now;
      }
      break;
  }
}

// 切换到“运动”阶段
void transitionToMove() {
  currentStep++;
  float target = START_DEG + STEP_DEG * currentStep;
  myServo.attach(SERVO_PIN);
  myServo.write(target);
  state = STATE_MOVE;
  stateStart = millis();
}

// INA219 采样+串口输出
void sampleAndPrint(unsigned long now) {
  // 累积能量计算
  if (now - lastSample >= SAMPLE_MS) {
    unsigned long dt = now - lastSample;
    lastSample = now;
    float V = ina219.getBusVoltage_V();
    float I = ina219.getCurrent_mA();
    float P = V * I;
    accum_mWh += P * dt / 3600000.0;
  }
  // 串口打印（20 Hz）
  if (now - lastPrint >= SAMPLE_MS) {
    lastPrint = now;
    // 当前角度
    float angle = (state==STATE_DWELL || state==STATE_FINAL_DWELL) ?
                   (START_DEG + STEP_DEG*currentStep) : 
                   ((state==STATE_RETURN_MOVE) ? START_DEG : START_DEG + STEP_DEG*currentStep);
    float V = ina219.getBusVoltage_V();
    float I = ina219.getCurrent_mA();
    float P = V * I;
    Serial.print(now);      Serial.print(',');
    Serial.print(angle);    Serial.print(',');
    Serial.print(V,2);      Serial.print(',');
    Serial.print(I,1);      Serial.print(',');
    Serial.println(P,2);
  }
}
