#include <Wire.h>
#include <Adafruit_INA219.h>
#include <MKRWAN.h>
#include <RTClib.h>            // RTC DS3231
#include <SunPosition.h>       // Sun position calculation library
#include <ArduinoLowPower.h>
#include <math.h>

// ───── LoRaWAN 初始化 ─────
LoRaModem modem;
const char *appEui = "0000000000000000";
const char *appKey = "5DEF1848C136307477DC8E930E463F8F";

// ───── SunPosition ─────
double LAT = 51.538593;      // London
double LON = -0.009006;
// 如果 RTC 存 BST(夏令时)，设为 1；如果 RTC 存 UTC，设为 0（推荐）
int timeZone = 0;

SunPosition sunPos;
RTC_DS3231 rtc;

// 电机控制引脚
const byte IN1 = 3, IN2 = 4, PWM = 5, STBY = 2;
const uint8_t PWM_DUTY = 150;

// 编码器
const byte ENC_A = 6, ENC_B = 7;
volatile long encCount = 0;
const float PULSE_PER_DEG = 4.7;

// 角度限制
const int16_t EAST_LIMIT = 90;
const int16_t WEST_LIMIT = 90;
const float   ROTATE_THRESHOLD = 5.0;

// 电流传感器
Adafruit_INA219 motorINA(0x41);
Adafruit_INA219 logicINA(0x44);
Adafruit_INA219 solarINA(0x40);

// 能耗累计值
float energyActive_mWh = 0.0;
float energyIdle_mWh   = 0.0;
float energyLogic_mWh  = 0.0;
float energySolar_mWh  = 0.0;

// 每分钟对比值
float lastEnergyActive_mWh = 0.0;
float lastEnergyIdle_mWh   = 0.0;
float lastEnergyLogic_mWh  = 0.0;
float lastEnergySolar_mWh  = 0.0;

// INA 平均值统计
float sumMotorV_active = 0, sumMotorI_active = 0;
float sumLogicV = 0, sumLogicI = 0;
float sumSolarV = 0, sumSolarI = 0;
int sampleCountLogic = 0;
int sampleCountSolar = 0;
int sampleCountMotorActive = 0;
int sampleCountMotorIdle = 0;

// LDR
const int LDR_EAST_PIN = A1;
const int LDR_WEST_PIN = A0;
int ldrEastVal = 0;
int ldrWestVal = 0;

// 追踪逻辑参数（新增）
const int LDR_ENTER_THRESH = 60;        // 进入“需要动作”的差值阈值
const int LDR_EXIT_THRESH  = 30;        // 退出/认为对准的阈值（迟滞）
const float COARSE_MIN_MOVE_DEG = 3.0;  // 小于此差值就不做粗调
const uint32_t TRACK_TIME_BUDGET_MS = 2500; // 本次唤醒最多追踪时长
const float SAFETY_MAX_JUMP_DEG = 120.0f;   // 异常兜底（RTC/编码器错误时保护）

// 定时器
uint32_t lastIdleSample = 0;
uint32_t lastLogicSample = 0;
uint32_t lastSolarSample = 0;
uint32_t lastUpload = 0;
const uint32_t SAMPLE_IDLE_MS = 500;
const uint32_t SAMPLE_OTHER_MS = 500;

// ── 先声明一下要用到的函数（为了解决顺序依赖）──
void motorBrake();
void isrEnc();
uint32_t msToNextMinute();

// ───── 外设省电（睡前）/恢复（醒后） ─────
void beforeSleep(uint32_t sleepMs) {
  motorBrake();
  pinMode(STBY, OUTPUT);
  digitalWrite(STBY, LOW);          // 关闭电机驱动（省电）
  // INA219 进省电
  motorINA.powerSave(true);
  logicINA.powerSave(true);
  solarINA.powerSave(true);
  // LoRa 模块（库支持的话）
  modem.sleep(sleepMs / 1000);      // 不支持也能编译通过
  // 进入 deepSleep（SAMD21: Standby，SRAM 保持）
  LowPower.deepSleep(sleepMs);
}

void afterWake() {
  // 唤醒后 I2C 重新 init 更稳
  Wire.begin();
  motorINA.begin();  motorINA.setCalibration_32V_2A();
  logicINA.begin();  logicINA.setCalibration_32V_2A();
  solarINA.begin();  solarINA.setCalibration_32V_2A();
  motorINA.powerSave(false);
  logicINA.powerSave(false);
  solarINA.powerSave(false);
  // 重新使能电机驱动（即使本轮不转）
  pinMode(STBY, OUTPUT);
  digitalWrite(STBY, HIGH);
  // 保险起见：重新挂中断（大多情况下不必，但这么做更稳）
  detachInterrupt(digitalPinToInterrupt(ENC_A));
  attachInterrupt(digitalPinToInterrupt(ENC_A), isrEnc, RISING);
}

// ───── 工具函数 ─────
float computeSunAzimuth() {
  DateTime now = rtc.now();
  SunPosition sun(LAT, LON, now.unixtime(), timeZone);
  return sun.azimuth();
}

void isrEnc() {
  encCount += digitalRead(ENC_B) ? +1 : -1;
}

float getAngle() {
  return encCount / PULSE_PER_DEG;
}

void motorRun(bool fwd, uint8_t duty) {
  digitalWrite(IN1, fwd ? LOW : HIGH);
  digitalWrite(IN2, fwd ? HIGH : LOW);
  analogWrite(PWM, duty);
}

void motorBrake() {
  analogWrite(PWM, 0);
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
}

float printPowerLog(long encNow, long tgtPulse, long errNow, uint8_t duty) {
  float busVoltage = motorINA.getBusVoltage_V();
  float current_mA = motorINA.getCurrent_mA();
  float power_mW = busVoltage * current_mA;
  return power_mW;
}

bool moveToAngle(float tgtDeg, uint32_t timeout = 8000) {
  tgtDeg = constrain(tgtDeg, -90, 90);
  long tgtPulse = lround(tgtDeg * PULSE_PER_DEG);
  bool fwd = tgtPulse > encCount;
  motorRun(fwd, PWM_DUTY);
  uint32_t t0 = millis();
  long prev = encCount;
  uint32_t lastPrint = 0;

  while (fwd ? encCount < tgtPulse : encCount > tgtPulse) {
    if (millis() - lastPrint > 20) {
      float voltage = motorINA.getBusVoltage_V();
      float current = motorINA.getCurrent_mA();
      float power = voltage * current;
      printPowerLog(encCount, tgtPulse, tgtPulse - encCount, PWM_DUTY);
      energyActive_mWh += power * 0.02 / 3600.0;
      if (current > 0) {
        sumMotorV_active += voltage;
        sumMotorI_active += current;
        sampleCountMotorActive++;
      }
      lastPrint = millis();
    }
    if (millis() - t0 > timeout || (millis() - t0 > 5000 && encCount == prev)) {
      motorBrake();
      return false;
    }
  }
  motorBrake();
  encCount = constrain(encCount, -90 * PULSE_PER_DEG, 90 * PULSE_PER_DEG);
  return true;
}

void autoHoming() {
  const uint32_t timeout = 10000; // 最多找 10 秒
  const uint8_t duty = 100;       // 慢速转动
  long lastCount = encCount;
  uint32_t start = millis();

  Serial.println("🚩 自动归零中...");
  motorRun(false, duty);  // 向东（负方向）转动

  while (millis() - start < timeout) {
    delay(200);  // 每200ms检查一次
    if (encCount == lastCount) {
      break;     // 编码器不再变化，说明触到底限
    }
    lastCount = encCount;
  }
  motorBrake();
  encCount = -90 * PULSE_PER_DEG;
  Serial.println("✅ 归零完成");
}

// LDR 辅助（本阶段不转电机，先保留）
int readLdrAvg(int pin, uint8_t n = LDR_SAMPLES) {
  long s = 0;
  for (uint8_t i = 0; i < n; i++) { s += analogRead(pin); delay(2); }
  return (int)(s / n);
}

int ldrDeltaSuggest(bool &goWest, int &eRaw, int &wRaw) {
  eRaw = readLdrAvg(LDR_EAST_PIN);
  wRaw = readLdrAvg(LDR_WEST_PIN);
  int eB = LDR_LOW_IS_BRIGHT ? (1023 - eRaw) : eRaw;
  int wB = LDR_LOW_IS_BRIGHT ? (1023 - wRaw) : wRaw;
  goWest = (wB > eB);                // 西侧更亮→往西
  return abs(wB - eB);
}

// 到下一个整分钟的睡眠时长（避免 <1.2s 的短睡）
uint32_t msToNextMinute() {
  DateTime t = rtc.now();
  uint32_t ms = (60 - t.second()) * 1000UL;
  if (ms < 1200) ms += 60000UL;
  return ms;
}

void setup() {
  Serial.begin(115200);
  delay(500);

  pinMode(IN1, OUTPUT); pinMode(IN2, OUTPUT); pinMode(PWM, OUTPUT); pinMode(STBY, OUTPUT);
  digitalWrite(STBY, HIGH);

  pinMode(ENC_A, INPUT_PULLUP); pinMode(ENC_B, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ENC_A), isrEnc, RISING);

  Wire.begin();
  rtc.begin();
  // if (rtc.lostPower()) { rtc.adjust(DateTime(F(__DATE__), F(__TIME__))); }

  motorINA.begin(); motorINA.setCalibration_32V_2A();
  logicINA.begin(); logicINA.setCalibration_32V_2A();
  solarINA.begin(); solarINA.setCalibration_32V_2A();

  // LoRa 初始化
  if (!modem.begin(EU868)) {
    Serial.println("❌ LoRa init failed");
    while (1);
  }
  modem.setADR(true);
  modem.setPort(1);
  bool joined = modem.joinOTAA(appEui, appKey);
  if (!joined) {
    Serial.println("⚠️ LoRa Join failed, will retry later");
  } else {
    Serial.println("✅ LoRaWAN joined");
  }

  autoHoming();  // 开机归零一次，之后不再每分钟归零

  // ✅ 第一次启动后，先对齐到整分再开始循环
  uint32_t sleepMs = msToNextMinute();
  beforeSleep(sleepMs);   // deepSleep 到整分
  afterWake();            // 唤醒后恢复外设
}

void loop() {
  // —— 一次 tick：算目标角、读传感器、上传（不转电机）——
  DateTime nowTime = rtc.now();
  SunPosition sunNow(LAT, LON, nowTime.unixtime(), timeZone);
  float sunEl = sunNow.altitude();

  uint16_t sr = sunNow.sunrise();                    // 分钟
  uint16_t ss = sunNow.sunset();                     // 分钟
  uint16_t nowMin = nowTime.hour()*60 + nowTime.minute();

  float tgt;
  if (sunEl > 5.0 && ss > sr) {
    float ratio = (float)(nowMin - sr) / (float)(ss - sr);
    ratio = constrain(ratio, 0.0f, 1.0f);
    tgt = -90.0f + ratio * 180.0f;                   // -90→+90
  } else {
    tgt = -EAST_LIMIT;                               // 夜里保持东端
  }

  // 传感器快照（一次即可）
  ldrEastVal = analogRead(LDR_EAST_PIN);
  ldrWestVal = analogRead(LDR_WEST_PIN);
  float logicV = logicINA.getBusVoltage_V();
  float logicI = logicINA.getCurrent_mA();
  float solarV = solarINA.getBusVoltage_V();
  float solarI = solarINA.getCurrent_mA();
  float curAngle = getAngle();                       // 仅报告，不动作

  // LoRa：连接着就发；不强制重入网（省电）
  if (modem.connected()) {
    float deltaActive = energyActive_mWh - lastEnergyActive_mWh;
    float deltaIdle   = energyIdle_mWh   - lastEnergyIdle_mWh;
    float deltaLogic  = energyLogic_mWh  - lastEnergyLogic_mWh;
    float deltaSolar  = energySolar_mWh  - lastEnergySolar_mWh;

    float avgMotorV_active = sumMotorV_active / max(1, sampleCountMotorActive);
    float avgMotorI_active = sumMotorI_active / max(1, sampleCountMotorActive);
    float avgLogicV = sumLogicV / max(1, sampleCountLogic);
    float avgLogicI = sumLogicI / max(1, sampleCountLogic);
    float avgSolarV = sumSolarV / max(1, sampleCountSolar);
    float avgSolarI = sumSolarI / max(1, sampleCountSolar);

    uint8_t payload[28];
    int i = 0;
    auto put16  = [&](uint16_t v){ payload[i++]= v & 0xFF; payload[i++]= v >> 8; };
    auto putS16 = [&](int16_t v){ put16((uint16_t)v); };

    putS16(lround(curAngle * 10));
    putS16(lround(tgt * 10));
    put16((uint16_t)(deltaActive * 1000));
    put16((uint16_t)(deltaIdle   * 1000));
    put16((uint16_t)(deltaLogic  * 100));
    put16((uint16_t)(deltaSolar  * 100));
    put16(ldrEastVal);
    put16(ldrWestVal);
    put16((uint16_t)(avgMotorV_active * 100));
    put16((uint16_t)(avgMotorI_active * 100));
    put16((uint16_t)(avgLogicV * 100));
    put16((uint16_t)(avgLogicI * 100));
    put16((uint16_t)(avgSolarV * 100));
    put16((uint16_t)(avgSolarI * 100));

    modem.beginPacket();
    modem.write(payload, sizeof(payload));
    int res = modem.endPacket(false);
    Serial.println(res == 0 ? "📡 LoRa OK" : "📡 LoRa FAIL");

    // 复位本轮统计（保持你原逻辑）
    lastEnergyActive_mWh = energyActive_mWh;
    lastEnergyIdle_mWh   = energyIdle_mWh;
    lastEnergyLogic_mWh  = energyLogic_mWh;
    lastEnergySolar_mWh  = energySolar_mWh;
    sumMotorV_active = sumMotorI_active = 0;
    sumLogicV = sumLogicI = sumSolarV = sumSolarI = 0;
    sampleCountMotorActive = sampleCountMotorIdle = sampleCountLogic = sampleCountSolar = 0;
  } else {
    Serial.println("⚠️ LoRa not connected, skip this minute.");
  }

  // 少量日志
  Serial.print("tick ");
  Serial.print(nowTime.hour()); Serial.print(":"); Serial.print(nowTime.minute());
  Serial.print("  tgt=");  Serial.print(tgt,1);
  Serial.print("  cur=");  Serial.print(curAngle,1);
  Serial.print("  LDR=");  Serial.print(ldrEastVal); Serial.print("/"); Serial.println(ldrWestVal);

  // —— 睡到下一整分钟（≈60s） —— //
  uint32_t sleepMs = msToNextMinute();   // 或者固定 60000UL
  beforeSleep(sleepMs);
  afterWake();                           // 醒来恢复外设，继续下一轮
}
