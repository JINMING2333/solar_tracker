#include <Wire.h>
#include <Adafruit_INA219.h>
#include <Stepper.h>

//////////////////// 用户可调参数 ////////////////////
const int STEPS_PER_REV     = 2048;        // 半步驱动全圈步数
const int STEP_PER_12DEG    = 68;          // 对应 12°
const int MOVES_TOTAL       = 10;          // 10 × 12° = 120°
const unsigned long SAMPLE_MS       = 50;      // 50ms 采样周期（20Hz）
const unsigned long DWELL_MS        = 30000;  // 每步驻停 30s
const unsigned long FINAL_DWELL_MS  = 60000;  // 顶部驻停 60s
const int STEPPER_RPM        = 10;          // 10 RPM ≈ 34 步/秒
//////////////////////////////////////////////////////

// 硬件对象
Stepper myStepper(STEPS_PER_REV, 2, 4, 3, 5);  // IN1,IN3,IN2,IN4
Adafruit_INA219 ina219;

// 状态变量
int   movesDone       = 0;
bool  forwardPhase    = true;
bool  inFinalDwell    = false;
bool  returningPhase  = false;
float currentAngleDeg = 0.0;  // 单位：度

// 采样计时
unsigned long lastSample  = 0;
unsigned long lastPrint   = 0;
unsigned long phaseStart  = 0;

float accum_mWh = 0.0;

void setup() {
  Serial.begin(115200);
  Wire.begin();

  // INA219 初始化
  if (!ina219.begin()) {
    while (1);  // 找不到 INA219
  }
  ina219.setCalibration_32V_2A();

  // 步进电机初始速度
  myStepper.setSpeed(STEPPER_RPM);

  // 等待 START
  while (!Serial.available()) {}
  if (Serial.readStringUntil('\n') != "START") while (1);

  phaseStart = millis();
}

void loop() {
  unsigned long now = millis();

  // —— 实时采样 & 输出 ——  
  updateINA219(now);

  // —— 控制步进电机周期 ——  
  if (forwardPhase) {
    // 正向每步
    if (!inFinalDwell && !returningPhase && now - phaseStart >= DWELL_MS) {
      // 执行一步
      movesDone++;
      myStepper.step(STEP_PER_12DEG);
      currentAngleDeg = movesDone * 12.0;
      phaseStart = now;
    }
    // 到达 120°
    if (movesDone == MOVES_TOTAL && !inFinalDwell) {
      inFinalDwell = true;
      phaseStart   = now;
    }
    // 顶部驻停 60s
    if (inFinalDwell && now - phaseStart >= FINAL_DWELL_MS) {
      // 反向一次性回到 0°
      myStepper.step(-STEP_PER_12DEG * MOVES_TOTAL);
      currentAngleDeg = 0.0;
      returningPhase  = true;
      inFinalDwell    = false;
      phaseStart      = now;
    }
    // 顶部反转完成后
    if (returningPhase && now - phaseStart >= DWELL_MS) {
      forwardPhase    = false;  // 进入下个阶段（可循环）
      returningPhase  = false;
      movesDone       = 0;
      phaseStart      = now;
    }
  } else {
    // 循环重置：你也可以改为只运行一次
    forwardPhase    = true;
    movesDone       = 0;
    phaseStart      = now;
  }
}

// —— 封装：采样 INA219 并串口输出 ——  
void updateINA219(unsigned long now) {
  if (now - lastSample >= SAMPLE_MS) {
    lastSample = now;

    float V = ina219.getBusVoltage_V();
    float I = ina219.getCurrent_mA();
    float P = V * I;
    accum_mWh += P * SAMPLE_MS / 3600000.0;

    if (now - lastPrint >= SAMPLE_MS) {
      lastPrint = now;
      Serial.print(now);               Serial.print(',');
      Serial.print(currentAngleDeg,1); Serial.print(',');
      Serial.print(V,2);               Serial.print(',');
      Serial.print(I,1);               Serial.print(',');
      Serial.println(P,2);             // ← 用 println，确保这一行结束
    }
  }
}
