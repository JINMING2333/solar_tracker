#include <Wire.h>
#include <Adafruit_INA219.h>

////////////////////用户可调参数 ////////////////////
const int IN1  = 3;    // AIN1
const int IN2  = 4;    // AIN2
const int PWM  = 5;    // PWMA
const int STBY = 2;    // STBY

const int speedPWM     = 60;       // 转速 PWM 值
const int stepDelayMs  = 130;      // 12 度转动时间
const int dwellMs      = 30000;    // 每步驻停时间
const int finalDwellMs = 60000;    // 最后驻停时间
const int NUM_STEPS    = 10;       // 每次12° * 10 步
const int SAMPLE_MS    = 10;       // 50 ms 采样周期（20 Hz）
//////////////////////////////////////////////////////

Adafruit_INA219 ina219;
float accum_mWh = 0.0;

int    currentStep      = 0;
bool   movingPhase      = true;
bool   returning        = false;
bool   inFinalDwell     = false;
unsigned long stepStart       = 0;
unsigned long finalDwellStart = 0;
unsigned long lastSample      = 0;
unsigned long lastPrint       = 0;

// 当前角度，单位度
float angleDeg = 0.0;

void setup() {
  Serial.begin(115200);
  Wire.begin();
  if (!ina219.begin()) while (1);
  ina219.setCalibration_32V_2A();

  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(PWM, OUTPUT);
  pinMode(STBY, OUTPUT);
  digitalWrite(STBY, HIGH);

  // 等待 Python 发来 START
  while (!Serial.available()) {}
  if (Serial.readStringUntil('\n') != "START") while (1);

  delay(1000);
  stepStart = millis();
  movingPhase = false;
}

void loop() {
  unsigned long now = millis();

  // —— 始终采样并输出 INA219 ——  
  updateINA219(now);

  // —— 电机控制状态机 ——  
  if (!returning) {
    // 正向步进
    if (!movingPhase && !inFinalDwell && now - stepStart >= dwellMs) {
      currentStep++;
      if (currentStep <= NUM_STEPS) {
        angleDeg = currentStep * 12.0;
        rotateDCMotorNonBlocking(true, stepDelayMs);
        movingPhase = true;
        stepStart = now;
      }
      if (currentStep == NUM_STEPS) {
        inFinalDwell    = true;
        finalDwellStart = now;
      }
    }
    // Final 驻停
    if (inFinalDwell && now - finalDwellStart >= finalDwellMs) {
      // 反转回零
      rotateDCMotorNonBlocking(false, stepDelayMs * NUM_STEPS);
      angleDeg      = 0.0;
      returning     = true;
      inFinalDwell  = false;
      stepStart     = now;
    }
    // 切回驻停完成后
    if (movingPhase && now - stepStart >= 200 && !inFinalDwell) {
      movingPhase = false;
      stepStart   = now;
    }
  } else {
    // 完成一轮，重置
    currentStep  = 0;
    returning    = false;
    movingPhase  = false;
    stepStart    = now;
  }
}

// —— 用于在通电期间非阻塞采样的转动函数 ——  
void rotateDCMotorNonBlocking(bool forward, int durationMs) {
  digitalWrite(IN1, forward ? HIGH : LOW);
  digitalWrite(IN2, forward ? LOW  : HIGH);
  analogWrite(PWM, speedPWM);

  unsigned long t0 = millis();
  while (millis() - t0 < (unsigned long)durationMs) {
    // 在这里不断地采样、打印
    unsigned long now = millis();
    updateINA219(now);
    // 为了避免 tight-loop 过快，稍微给点空隙
    delay(1);
  }

  // 切断电机
  analogWrite(PWM, 0);
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
}

// —— 采样 & 串口输出 ——  
void updateINA219(unsigned long now) {
  // 采样
  if (now - lastSample >= SAMPLE_MS) {
    lastSample = now;
    float busV = ina219.getBusVoltage_V();
    float cur  = ina219.getCurrent_mA();
    float p_mW = busV * cur;
    accum_mWh += p_mW * (now - lastSample) / 3600000.0;
  }

  // 打印（20 Hz）
  if (now - lastPrint >= SAMPLE_MS) {
    lastPrint = now;
    Serial.print(now);      Serial.print(",");
    Serial.print(angleDeg); Serial.print(",");
    Serial.print(ina219.getBusVoltage_V(), 2); Serial.print(",");
    Serial.print(ina219.getCurrent_mA(), 1);    Serial.print(",");
    Serial.println(ina219.getBusVoltage_V() * ina219.getCurrent_mA(), 2);
  }
}

