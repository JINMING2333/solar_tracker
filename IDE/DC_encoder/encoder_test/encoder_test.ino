#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_INA219.h>

/* ---------- pin map ---------- */
const byte IN1   = 7;
const byte IN2   = 4;
const byte PWM   = 5;
const byte STBY  = 6;
const byte ENC_A = 2;
const byte ENC_B = 3;

/* ---------- motor test参数 ---------- */
const uint16_t PWM_DUTY = 120;      // ≈50 % 占空
const uint32_t RUN_MS   = 3000;     // 正/反各 3 s
const uint32_t BRAKE_MS = 3000;     // 中间停 1 s

/* ---------- INA219 ---------- */
Adafruit_INA219 motorINA(0x41);     // 电机
Adafruit_INA219 logicINA(0x40);     // 逻辑
Adafruit_INA219 solarINA(0x44);     // 光伏（暂时可不接）
const uint32_t  SAMPLE_MS = 500;     // 采样周期
uint32_t lastSample = 0;

/* ---------- encoder ---------- */
volatile long encCount = 0;
void isrEncA() { encCount += digitalRead(ENC_B) ? +1 : -1; }

/* ---------- motor helpers ---------- */
inline void motorRun(bool fwd, uint8_t duty) {
  digitalWrite(IN1, fwd ? HIGH : LOW);
  digitalWrite(IN2, fwd ? LOW  : HIGH);
  analogWrite (PWM, duty);
}
inline void motorBrake() {
  analogWrite(PWM, 0);
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
}

/* ---------- INA non-blocking sampler ---------- */
void serviceINA219()
{
  uint32_t now = millis();
  if (now - lastSample < SAMPLE_MS) return;
  lastSample = now;

  // 读取三路（如暂时未接，也只是读到 0 或固定值）
  float Vm = motorINA.getBusVoltage_V();
  float Im = motorINA.getCurrent_mA();

  float Vl = logicINA.getBusVoltage_V();
  float Il = logicINA.getCurrent_mA();

  float Vs = solarINA.getBusVoltage_V();
  float Is = solarINA.getCurrent_mA();

  // Serial.print(now);              Serial.print(',');
  Serial.println(encCount);         //Serial.print(',');
  // Serial.print(Vm,2);             Serial.print(',');
  // Serial.print(Im,0);             Serial.print(',');
  // Serial.print(Vl,2);             Serial.print(',');
  // Serial.print(Il,0);             Serial.print(',');
  // Serial.print(Vs,2);             Serial.print(',');
  // Serial.println(Is,0);
}

/* ---------- SETUP ---------- */
void setup()
{
  Serial.begin(115200);

  pinMode(IN1,  OUTPUT);
  pinMode(IN2,  OUTPUT);
  pinMode(PWM,  OUTPUT);
  pinMode(STBY, OUTPUT);
  digitalWrite(STBY, HIGH);

  pinMode(ENC_A, INPUT_PULLUP);
  pinMode(ENC_B, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ENC_A), isrEncA, RISING);

  Wire.begin();
  Wire.setClock(400000);             // 保守使用 400 kHz

  motorINA.begin();
  logicINA.begin();
  solarINA.begin();                  // 如果没连，依旧安全

  motorINA.setCalibration_16V_400mA();
  logicINA.setCalibration_16V_400mA();
  solarINA.setCalibration_16V_400mA();

  Serial.println(F("t,enc,Vm,Im,Vl,Il,Vs,Is"));
}

/* ---------- LOOP ---------- */
void loop()
{
  /* ---------- forward ---------- */
  encCount = 0;
  motorRun(true, PWM_DUTY);
  uint32_t t0 = millis();
  while (millis() - t0 < RUN_MS) serviceINA219();
  motorBrake();

  uint32_t t1 = millis();
  while (millis() - t1 < BRAKE_MS) serviceINA219();

  /* ---------- backward ---------- */
  encCount = 0;
  motorRun(false, PWM_DUTY);
  t0 = millis();
  while (millis() - t0 < RUN_MS) serviceINA219();
  motorBrake();

  t1 = millis();
  while (millis() - t1 < BRAKE_MS) serviceINA219();

  /* ---------- 循环 ---------- */
}
