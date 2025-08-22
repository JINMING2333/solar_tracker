// === 依赖库 ===
#include <Wire.h>
#include <MKRWAN.h>
#include <RTClib.h>
#include <SunPosition.h>
#include <ArduinoLowPower.h>
#include <math.h>
#include <Adafruit_INA219.h>

// === 位置信息 ===
double LAT = 51.538593;
double LON = -0.009006;
int timeZone = 1;  // BST（伦敦夏令时）
RTC_DS3231 rtc;

// === LoRa 配置 ===
LoRaModem modem;
const char *appEui = "0000000000000000";
const char *appKey = "5DEF1848C136307477DC8E930E463F8F";
uint32_t lastJoinAttempt = 0;

// === 电机引脚与编码器 ===
const byte IN1 = 3, IN2 = 4, PWM = 5, STBY = 2;
const byte ENC_A = 6, ENC_B = 7;
const uint8_t PWM_DUTY = 180;
volatile long encCount = 0;
const float PULSE_PER_DEG = 4.7;
const int16_t EAST_LIMIT = 90;
const int16_t WEST_LIMIT = 90;
bool motorMoving = false;

// === LDR 参数 ===
const int LDR_EAST_PIN = A1, LDR_WEST_PIN = A0;
const bool LDR_LOW_IS_BRIGHT = true, LDR_FLIP_DIR = false;
const uint8_t LDR_SAMPLES = 4;
const float LDR_STEP_DEG = 2.0;
const uint16_t LDR_SETTLE_MS = 200;
const int LDR_EXIT_THRESH = 20;
const uint32_t TRACK_TIME_BUDGET_MS = 5000;

// === 日夜与唤醒间隔 ===
const float EL_NIGHT = 0.0, EL_DAY = 2.0;
const uint16_t DAY_INTERVAL_MIN = 10, NIGHT_INTERVAL_MIN = 30;
bool nightMode = false, homedEastTonight = false;

// === INA219 模块 ===
Adafruit_INA219 solarINA(0x40), logicINA(0x44), motorINA(0x41);

// === 电压电流能量统计 ===
float solar_energy_mWh = 0, logic_energy_mWh = 0;
float motor_energy_run_mWh = 0, motor_energy_idle_mWh = 0;
float solar_sumV = 0, solar_sumI_mA = 0;
float logic_sumV = 0, logic_sumI_mA = 0;
float motor_sumV_run = 0, motor_sumI_run = 0;
float motor_sumV_idle = 0, motor_sumI_idle = 0;
uint32_t solar_samples = 0, logic_samples = 0;
uint32_t motor_samples_run = 0, motor_samples_idle = 0;

// === 采样控制 ===
uint32_t lastSampleMs = 0;
const uint16_t SAMPLE_INTERVAL_MS = 50;
uint32_t wakeStartMs = 0;
uint16_t wakeDuration_s = 0;

// === 中断/角度相关 ===
void isrEnc() { encCount += digitalRead(ENC_B) ? +1 : -1; }
float getAngle() { return encCount / PULSE_PER_DEG; }

// === 电机控制函数 ===
void motorRun(bool fwd, uint8_t duty) {
  motorMoving = true;
  digitalWrite(IN1, fwd ? LOW : HIGH);
  digitalWrite(IN2, fwd ? HIGH : LOW);
  analogWrite(PWM, duty);
}

void motorBrake() {
  analogWrite(PWM, 0);
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  motorMoving = false;
}

// === 角度移动控制（含采样）===
bool moveToAngle(float tgtDeg, uint32_t timeout = 8000) {
  tgtDeg = constrain(tgtDeg, -90, 90);
  long tgtPulse = lround(tgtDeg * PULSE_PER_DEG);
  bool fwd = (tgtPulse > encCount);
  motorRun(fwd, PWM_DUTY);
  uint32_t t0 = millis();
  long prev = encCount;

  while (fwd ? encCount < tgtPulse : encCount > tgtPulse) {
    uint32_t nowMs = millis();
    sampleSensors(nowMs);
    if (nowMs - t0 > timeout || (nowMs - t0 > 5000 && encCount == prev)) {
      motorBrake();
      return false;
    }
    prev = encCount;
  }

  motorBrake();
  encCount = constrain(encCount, -90 * PULSE_PER_DEG, +90 * PULSE_PER_DEG);
  return true;
}

void autoHoming() {
  Serial.println("🚩 Entering autoHoming()");
  const uint32_t timeout = 10000;
  const uint8_t duty = 100;
  long lastCount = encCount;
  uint32_t start = millis();
  
  motorMoving = true;  // ✅ 开始动作前，标记为运动
  motorRun(false, duty);  // 向东归零

  while (millis() - start < timeout) {
    delay(200);
    if (encCount == lastCount) break;  // 到机械限位
    lastCount = encCount;
  }

  motorBrake();
  motorMoving = false;  // ✅ 动作完成后，标记为静止
  encCount = -90 * PULSE_PER_DEG;
  Serial.println("✅ Homing complete.");
}

// === LDR 比较建议方向 ===
int readLdrAvgRaw(int pin, uint8_t n=LDR_SAMPLES) {
  long s = 0;
  for (uint8_t i = 0; i < n; i++) {
    s += analogRead(pin);
    delay(2);
  }
  return s / n;
}

int ldrDeltaSuggest(bool &goWest, int &eRaw, int &wRaw) {
  eRaw = readLdrAvgRaw(LDR_EAST_PIN);
  wRaw = readLdrAvgRaw(LDR_WEST_PIN);
  int eB = LDR_LOW_IS_BRIGHT ? (1023 - eRaw) : eRaw;
  int wB = LDR_LOW_IS_BRIGHT ? (1023 - wRaw) : wRaw;
  goWest = (wB > eB);                              //Decide on the direction of fine-tuning
  if (LDR_FLIP_DIR) goWest = !goWest;
  return abs(wB - eB);                             //Returns the absolute value of the difference
}

// === Linear interpolation to calculate the target angle ===
float computeLinearTgt(SunPosition& sunNow, const DateTime& nowTime,
                       int eastLimit, int westLimit) {
  float sunEl = sunNow.altitude();
  uint16_t sr = sunNow.sunrise();
  uint16_t ss = sunNow.sunset();
  uint16_t nowMin = nowTime.hour()*60 + nowTime.minute();             //get real time
  if (sunEl <= EL_NIGHT || ss <= sr || nowMin < sr || nowMin > ss)    //Return east at night
    return -eastLimit;
  float ratio = (float)(nowMin - sr) / (float)(ss - sr);              //Linear interpolation
  ratio = constrain(ratio, 0.0f, 1.0f);
  return -90.0f + ratio * 180.0f;
}

// === sampling ===
void sampleSensors(uint32_t nowMs) {
  if (lastSampleMs != 0 && nowMs - lastSampleMs < SAMPLE_INTERVAL_MS) return;
  lastSampleMs = nowMs;
  float vS = solarINA.getBusVoltage_V(), iS = solarINA.getCurrent_mA();
  float vL = logicINA.getBusVoltage_V(), iL = logicINA.getCurrent_mA();
  float vM = motorINA.getBusVoltage_V(), iM = motorINA.getCurrent_mA();

  if (vS >= 0 && iS >= 0) {
    solar_sumV += vS; solar_sumI_mA += iS;
    solar_samples++;
    solar_energy_mWh += (vS * iS) * (SAMPLE_INTERVAL_MS / 3600000.0f);
  }

  if (vL >= 0 && iL >= 0) {
    logic_sumV += vL; logic_sumI_mA += iL;
    logic_samples++;
    logic_energy_mWh += (vL * iL) * (SAMPLE_INTERVAL_MS / 3600000.0f);
  }

  if (vM >= 0 && iM >= 0) {
    if (motorMoving) {
      motor_sumV_run += vM; motor_sumI_run += iM;
      motor_samples_run++;
      motor_energy_run_mWh += (vM * iM) * (SAMPLE_INTERVAL_MS / 3600000.0f);
    } else {
      motor_sumV_idle += vM; motor_sumI_idle += iM;
      motor_samples_idle++;
      motor_energy_idle_mWh += (vM * iM) * (SAMPLE_INTERVAL_MS / 3600000.0f);
    }
  }
}


void beforeSleep(uint32_t sleepMs) {
  wakeDuration_s = (millis() - wakeStartMs) / 1000UL;
  motorBrake();                                       //Motor stall
  pinMode(STBY, OUTPUT); digitalWrite(STBY, LOW);     //Disable the motor driver board
  modem.sleep(sleepMs / 1000);                        //LoRa module sleep
  LowPower.deepSleep(sleepMs);                        //Enter low-power sleep mode
}

void afterWake() {
  Wire.begin();                                                   //Restart I2C
  digitalWrite(STBY, HIGH);                                       //Re-enable the motor driver board
  attachInterrupt(digitalPinToInterrupt(ENC_A), isrEnc, RISING);
  lastSampleMs = 0;
  wakeStartMs = millis();                                         //Record wake-up time
  solar_energy_mWh = logic_energy_mWh = 0;                        // Reset sampling statistics
  motor_energy_run_mWh = motor_energy_idle_mWh = 0;
  solar_sumV = solar_sumI_mA = logic_sumV = logic_sumI_mA = 0;
  motor_sumV_run = motor_sumI_run = motor_sumV_idle = motor_sumI_idle = 0;
  solar_samples = logic_samples = 0;
  motor_samples_run = motor_samples_idle = 0;
}

// === 启动 ===
void setup() {
  Serial.begin(115200); delay(300);
  pinMode(IN1, OUTPUT); pinMode(IN2, OUTPUT); pinMode(PWM, OUTPUT); pinMode(STBY, OUTPUT);
  pinMode(ENC_A, INPUT_PULLUP); pinMode(ENC_B, INPUT_PULLUP);
  digitalWrite(STBY, HIGH);
  attachInterrupt(digitalPinToInterrupt(ENC_A), isrEnc, RISING);

  Wire.begin(); rtc.begin();
  solarINA.begin(); logicINA.begin(); motorINA.begin();

  if (!modem.begin(EU868)) { Serial.println("❌ LoRa init failed"); while(1); }
  modem.setADR(true); modem.setPort(1);
  if (modem.joinOTAA(appEui, appKey)) Serial.println("✅ LoRa joined");
  else Serial.println("⚠️ LoRa join failed");

  autoHoming();
  afterWake();
}

// === 主循环 ===
void loop() {
  DateTime now = rtc.now();
  char buf[32];
  snprintf(buf, sizeof(buf), "⏰ RTC Time: %04d-%02d-%02d %02d:%02d:%02d",
          now.year(), now.month(), now.day(),
          now.hour(), now.minute(), now.second());
  Serial.println(buf);

  uint32_t tUTC = now.unixtime() - 3600UL;  // BST to UTC
  SunPosition sun(LAT, LON, tUTC, timeZone);
  float sunEl = sun.altitude();

  if (!nightMode && sunEl <= EL_NIGHT) {
    if (!homedEastTonight && getAngle() > -EAST_LIMIT + 0.5f)
      moveToAngle(-EAST_LIMIT);
    nightMode = true; homedEastTonight = true;
  }

  if (nightMode && sunEl >= EL_DAY) {
    nightMode = false; homedEastTonight = false;
  }

  float tgt = -EAST_LIMIT;
  if (!nightMode) {
    tgt = computeLinearTgt(sun, now, EAST_LIMIT, WEST_LIMIT);
    moveToAngle(tgt);

    uint32_t t0 = millis();
    float cur = getAngle();
    while (millis() - t0 < TRACK_TIME_BUDGET_MS) {                 //Fine-tune within the set time
      bool goW; int er, wr;
      if (ldrDeltaSuggest(goW, er, wr) <= LDR_EXIT_THRESH) break;  //Call ldrDeltaSuggest()
      float next = cur + (goW ? +LDR_STEP_DEG : -LDR_STEP_DEG);    
      next = constrain(next, -EAST_LIMIT, WEST_LIMIT);             //Calculate a new target angle
      if (!moveToAngle(next)) break;                               //The motor turns to the new angle
      delay(LDR_SETTLE_MS);
      cur = getAngle();
    }
  }

  uint32_t holdStart = millis();
  uint16_t holdMs = nightMode ? 500 : 2000;
  while (millis() - holdStart < holdMs) {
    sampleSensors(millis());
    delay(5);
  }

  // 构建并发送 payload（略，为节省长度，可继续补全）
  if (!modem.connected()) {
    if (millis() - lastJoinAttempt > 300000UL) {
      if (modem.joinOTAA(appEui, appKey)) Serial.println("✅ Rejoined");
      else Serial.println("❌ Rejoin failed");
      lastJoinAttempt = millis();
    }
  } else {
    int16_t cur_x10 = (int16_t)lround(getAngle() * 10.0f);
    int16_t tgt_x10 = (int16_t)lround(tgt * 10.0f);
    uint16_t ldrEast = (uint16_t)readLdrAvgRaw(LDR_EAST_PIN);
    uint16_t ldrWest = (uint16_t)readLdrAvgRaw(LDR_WEST_PIN);
    uint16_t wake_s = (millis() - wakeStartMs) / 1000UL;

    float Vm_logic = logic_samples ? (logic_sumV / logic_samples) : 0.0f;
    float Im_logic = logic_samples ? (logic_sumI_mA / logic_samples) : 0.0f;
    float Vm_motor_idle = motor_samples_idle ? (motor_sumV_idle / motor_samples_idle) : 0.0f;
    float Im_motor_idle = motor_samples_idle ? (motor_sumI_idle / motor_samples_idle) : 0.0f;
    float Vm_motor_run = motor_samples_run ? (motor_sumV_run / motor_samples_run) : 0.0f;
    float Im_motor_run = motor_samples_run ? (motor_sumI_run / motor_samples_run) : 0.0f;
    float Vm_solar = solar_samples ? (solar_sumV / solar_samples) : 0.0f;
    float Im_solar = solar_samples ? (solar_sumI_mA / solar_samples) : 0.0f;

    uint16_t E_logic_x100 = (uint16_t)lround(max(0.0f, logic_energy_mWh) * 100.0f);
    uint16_t E_motor_idle_x100 = (uint16_t)lround(max(0.0f, motor_energy_idle_mWh) * 100.0f);
    uint16_t E_motor_run_x100 = (uint16_t)lround(max(0.0f, motor_energy_run_mWh) * 100.0f);
    uint16_t E_solar_x100 = (uint16_t)lround(max(0.0f, solar_energy_mWh) * 100.0f);
    uint16_t Vm_logic_x100 = (uint16_t)lround(max(0.0f, Vm_logic) * 100.0f);
    uint16_t Im_logic_x100 = (uint16_t)lround(max(0.0f, Im_logic) * 100.0f);
    uint16_t Vm_motor_idle_x100 = (uint16_t)lround(max(0.0f, Vm_motor_idle) * 100.0f);
    uint16_t Im_motor_idle_x100 = (uint16_t)lround(max(0.0f, Im_motor_idle) * 100.0f);
    uint16_t Vm_motor_run_x100 = (uint16_t)lround(max(0.0f, Vm_motor_run) * 100.0f);
    uint16_t Im_motor_run_x100 = (uint16_t)lround(max(0.0f, Im_motor_run) * 100.0f);
    uint16_t Vm_solar_x100 = (uint16_t)lround(max(0.0f, Vm_solar) * 100.0f);
    uint16_t Im_solar_x100 = (uint16_t)lround(max(0.0f, Im_solar) * 100.0f);

    uint8_t payload[36]; int i = 0;
    auto put16 = [&](uint16_t v) { payload[i++] = v & 0xFF; payload[i++] = v >> 8; };

    put16(cur_x10);           // 当前角度
    put16(tgt_x10);           // 目标角度
    put16(ldrEast);           // LDR东
    put16(ldrWest);           // LDR西
    put16(wake_s);            // 唤醒时间（秒）
    put16(E_logic_x100);      // 逻辑电路消耗
    put16(E_motor_idle_x100); // 电机静止消耗
    put16(E_motor_run_x100);  // 电机运动消耗
    put16(E_solar_x100);      // 太阳能吸收
    put16(Vm_logic_x100);     // 逻辑电压
    put16(Im_logic_x100);     // 逻辑电流
    put16(Vm_motor_idle_x100);// 电机静止电压
    put16(Im_motor_idle_x100);// 电机静止电流
    put16(Vm_motor_run_x100); // 电机运动电压
    put16(Im_motor_run_x100); // 电机运动电流
    put16(Vm_solar_x100);     // 太阳能电压
    put16(Im_solar_x100);     // 太阳能电流

    modem.beginPacket();
    modem.write(payload, sizeof(payload));
    int res = modem.endPacket(false);
    Serial.println(res == 0 ? "📡 LoRa OK" : "📡 LoRa FAIL");
  }

  uint32_t sleepMs = (nightMode ? NIGHT_INTERVAL_MIN : DAY_INTERVAL_MIN) * 60000UL;
  beforeSleep(sleepMs);
  afterWake();
}
