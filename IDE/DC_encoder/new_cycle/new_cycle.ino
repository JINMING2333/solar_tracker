#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_INA219.h>

/* ----------- 可按需修改的参数 ----------- */
const byte IN1   = 3;
const byte IN2   = 4;
const byte PWM   = 5;
const byte STBY  = 2;
const byte ENC_A = 6;
const byte ENC_B = 7;

const uint8_t  STEP_DEG        = 12;         // 每步角度
const uint8_t  STEP_COUNT      = 10;         // 0→120° 共 10 步
const uint16_t PWM_DUTY         = 120;       // 0-255，占空调节
const unsigned long T_DWELL_S   = 10000;  // 步间停留 30 s
const unsigned long T_DWELL_L   = 30000;  // 120° 停留 60 s
const float    PULSE_PER_DEG   = 2.436;      // 编码器比例

/* ---------- INA219 ---------- */
Adafruit_INA219 motorINA(0x41);     // 电机
Adafruit_INA219 logicINA(0x40);     // 逻辑
Adafruit_INA219 solarINA(0x44);     // 光伏（暂时可不接）
const unsigned long SAMPLE_MS = 50;      // 50 ms 周期
unsigned long tLastSample = 0;

/* ----------- 全局变量 / ISR ----------- */
volatile long encCount = 0;
void isrEnc() { encCount += digitalRead(ENC_B) ? -1 : +1; }

/* ----------- 电机控制 ----------- */
inline void motorRun(bool fwd, uint8_t duty)
{
  digitalWrite(IN1, fwd ? HIGH : LOW);
  digitalWrite(IN2, fwd ? LOW  : HIGH);
  analogWrite (PWM, duty);
}
inline void motorBrake()
{
  analogWrite(PWM, 0);
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
}

/* ----------- INA219 定时采样 ----------- */
void serviceINA219(){
  unsigned long now = millis();
  if (now - tLastSample >= SAMPLE_MS){
    tLastSample = now;
    float v = motorINA.getBusVoltage_V();
    float i = motorINA.getCurrent_mA();
    float p = v * i;
    float ang = encCount / PULSE_PER_DEG;
    Serial.print(now); Serial.print(',');
    // Serial.print(encCount);  Serial.print(',');
    Serial.print(ang, 1);    Serial.print(',');
    Serial.print(v, 2);      Serial.print(',');
    Serial.print(i, 1);      Serial.print(',');
    Serial.println(p, 2);
  }
}

/* ----------- 状态机 ----------- */
enum State { S_INIT, S_FWD, S_DWELL_S, S_DWELL_L, S_BACK, S_DONE };
State state          = S_INIT;
bool  moving         = false;
bool  dirForward     = true;
long  targetPulse    = 0;
uint8_t stepIndex    = 0;
unsigned long tState = 0;               // 当前状态开始时间
long   baseCount     = 0;

/* ----------- 帮助函数：开始一次移动 ----------- */
void startMove(long tgt)
{
  targetPulse = tgt;
  dirForward  = (tgt > encCount);
  motorRun(dirForward, PWM_DUTY);
  moving      = true;
}

/* ======================== SETUP ======================== */
void setup()
{
  Serial.begin(115200);

  pinMode(IN1,  OUTPUT);
  pinMode(IN2,  OUTPUT);
  pinMode(PWM,  OUTPUT);
  pinMode(STBY, OUTPUT);
  digitalWrite(STBY, HIGH);         // 使能 H-bridge

  pinMode(ENC_A, INPUT_PULLUP);
  pinMode(ENC_B, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ENC_A), isrEnc, RISING);

  Wire.begin();
  Wire.setClock(400000);                   // 400 kHz I²C
  motorINA.begin();
  motorINA.setCalibration_32V_2A();        // 根据分流电阻/量程调整

  /* ----------- wait for “START” ---------- */
  while (!Serial.available()) {}
  if (Serial.readStringUntil('\n') != "START") while (1);

  Serial.println(F("ms,encCnt,angle,V,I,P"));   // CSV 头
}

/* ======================== LOOP  ======================== */
void loop(){
  serviceINA219();
  /* ---------- 状态机 ---------- */
  switch (state)
  {
  case S_INIT:                            // 记录零位
    baseCount = encCount;
    stepIndex = 0;
    state     = S_FWD;
    break;

  /* ---------- 正向 12° ×10 步 ---------- */
  case S_FWD:
    if (!moving) {                        // 进入移动
      ++stepIndex;
      long tgt = baseCount + lround(stepIndex * STEP_DEG * PULSE_PER_DEG);
      startMove(tgt);
      Serial.print(F("FWD step ")); Serial.println(stepIndex);
    }
    else                                  // 正在移动
    {
      if ((dirForward && encCount >= targetPulse) ||
          (!dirForward && encCount <= targetPulse))
      {
        motorBrake();  moving = false;
        tState = millis();
        state  = (stepIndex >= STEP_COUNT) ? S_DWELL_L : S_DWELL_S;
      }
    }
    break;

  /* ---------- 步间 30 s ---------- */
  case S_DWELL_S:
    if (millis() - tState >= T_DWELL_S) state = S_FWD;
    break;

  /* ---------- 120° 停 60 s ---------- */
  case S_DWELL_L:
    if (millis() - tState >= T_DWELL_L) {
      startMove(baseCount);               // 一次性回零
      //Serial.println(F("BACK to 0"));
      state = S_BACK;
    }
    break;

  /* ---------- 回零完成 ---------- */
  case S_BACK:
    if ((dirForward && encCount >= targetPulse) ||
        (!dirForward && encCount <= targetPulse))
    {
      motorBrake();  moving = false;
      //Serial.println(F("== CYCLE DONE =="));
      state = S_DONE;
    }
    break;

    case S_DONE:  // 保持停止 + 继续采样一段时间
      motorBrake();
      static unsigned long tDoneStart = 0;
      if (tDoneStart == 0) {
        tDoneStart = millis();
        Serial.println(F("== CYCLE DONE =="));  // 加提示
      }
      if (millis() - tDoneStart > 10000) {
        while (true) {}  // 停止程序运行
      }
      break;

  }
}