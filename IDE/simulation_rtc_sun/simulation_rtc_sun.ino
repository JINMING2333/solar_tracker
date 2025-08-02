#include <Wire.h>
#include <Adafruit_INA219.h>
#include <MKRWAN.h>
#include <RTClib.h>            // RTC DS3231
#include <SunPosition.h>       // Sun position calculation library

// ───── LoRaWAN 初始化 ─────
LoRaModem modem;
const char *appEui = "0000000000000000";
const char *appKey = "5DEF1848C136307477DC8E930E463F8F";

// ───── SunPosition ─────
double LAT = 51.538593;      // London
double LON = -0.009006;
int timeZone = 0;             // GMT+1 for British Summer Time (BST)
SunPosition sunPos;
RTC_DS3231 rtc;

// 电机控制引脚
const byte IN1 = 3, IN2 = 4, PWM = 5, STBY = 2;
const uint8_t PWM_DUTY = 150;

// 编码器
const byte ENC_A = 6, ENC_B = 7;
volatile long encCount = 0;
const float PULSE_PER_DEG = 4.5;

const int16_t EAST_LIMIT = 90;
const int16_t WEST_LIMIT = 90;
const float   ROTATE_THRESHOLD = 5.0;

// ───── Replace mockAz calculation ─────
float computeSunAzimuth() {
  DateTime now = rtc.now();
  SunPosition sun(LAT, LON, now.unixtime(), timeZone);  // 创建并计算
  return sun.azimuth();
}

// 电流传感器
Adafruit_INA219 motorINA(0x41);
Adafruit_INA219 logicINA(0x44);
Adafruit_INA219 solarINA(0x40);

// 能耗累计值
float energyActive_mWh = 0.0;
float energyIdle_mWh = 0.0;
float energyLogic_mWh = 0.0;
float energySolar_mWh = 0.0;

// 每分钟对比值
float lastEnergyActive_mWh = 0.0;
float lastEnergyIdle_mWh = 0.0;
float lastEnergyLogic_mWh = 0.0;
float lastEnergySolar_mWh = 0.0;

// INA 平均值统计
float sumMotorV_active = 0, sumMotorI_active = 0;
// float sumMotorV_idle = 0, sumMotorI_idle = 0;
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

// 定时器
uint32_t lastIdleSample = 0;
uint32_t lastLogicSample = 0;
uint32_t lastSolarSample = 0;
uint32_t lastUpload = 0;
const uint32_t SAMPLE_IDLE_MS = 500;
const uint32_t SAMPLE_OTHER_MS = 500;
const uint32_t UPLOAD_INTERVAL_MS = 1 * 60 * 1000;

uint32_t lastJoinAttempt = 0;

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

  // Serial.print("enc="); Serial.print(encNow);
  // Serial.print("  tgt="); Serial.print(tgtPulse);
  // Serial.print("  err="); Serial.print(errNow);
  // Serial.print("  duty="); Serial.print(duty);
  // Serial.print("  Vm="); Serial.print(busVoltage, 2);
  // Serial.print(" V  Im="); Serial.print(current_mA, 1);
  // Serial.print(" mA  Pm="); Serial.print(power_mW, 1); Serial.println(" mW");

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
    if (millis() - t0 > timeout || (millis() - t0 > 1500 && encCount == prev)) {
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
  const uint8_t duty = 100;        // 慢速转动
  long lastCount = encCount;
  uint32_t start = millis();

  Serial.println("🚩 自动归零中...");

  motorRun(false, duty);  // 向东（负方向）转动

  while (millis() - start < timeout) {
    delay(200);  // 每200ms检查一次
    if (encCount == lastCount) {
      // 编码器不再变化，说明触到底限
      break;
    }
    lastCount = encCount;
  }

  motorBrake();
  encCount = -90 * PULSE_PER_DEG;
  Serial.println("✅ 归零完成");
}

void setup() {
  Serial.begin(115200);
  delay(500);
  pinMode(IN1, OUTPUT); pinMode(IN2, OUTPUT); pinMode(PWM, OUTPUT); pinMode(STBY, OUTPUT);
  digitalWrite(STBY, HIGH);

  pinMode(ENC_A, INPUT_PULLUP); pinMode(ENC_B, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ENC_A), isrEnc, RISING);

  // ⬇️ setup() 中增加 RTC 初始化：
  Wire.begin();
  rtc.begin();
  // if (rtc.lostPower()) {
  //   Serial.println("⚠️ RTC lost power, set time manually!");
  //   rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));  // 编译时时间作为默认值
  // }

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
  }
  Serial.println("✅ LoRaWAN joined");

  lastJoinAttempt = millis();

  autoHoming();  // 自动归零！
  // encCount = 0;
  // encCount = -90 * PULSE_PER_DEG;
}

void loop() {

  static uint32_t lastRTCPrint = 0;
  if (millis() - lastRTCPrint >= 10000) {  // 每10秒打印一次
    DateTime nowTime = rtc.now();
    Serial.print("🕒 RTC：");
    Serial.print(nowTime.year()); Serial.print("-");
    Serial.print(nowTime.month()); Serial.print("-");
    Serial.print(nowTime.day()); Serial.print(" ");
    Serial.print(nowTime.hour()); Serial.print(":");
    Serial.print(nowTime.minute()); Serial.print(":");
    Serial.println(nowTime.second());
    lastRTCPrint = millis();
  }

  uint32_t now = millis();
  float curAngle = getAngle();

  static bool hasReturnedEast = false;
  DateTime nowTime = rtc.now();
  SunPosition sunNow(LAT, LON, nowTime.unixtime(), timeZone);
  float sunAz = sunNow.azimuth();    // ✅ 函数调用
  float sunEl = sunNow.altitude();  // ✅ 函数调用
  float tgt;

  // if (sunEl > 5.0) {
  //   // ☀️ 白天：开始追踪太阳
  //   tgt = constrain(sunAz - 180.0, -EAST_LIMIT, WEST_LIMIT);
  //   hasReturnedEast = false;  // 白天取消归东标记，准备晚上再回东
  // } else {
  //   // 🌙 夜晚：若尚未归东，则归东一次
  //   if (!hasReturnedEast) {
  //     Serial.println("🌙 太阳落山，执行回东");
  //     bool ok = moveToAngle(-EAST_LIMIT);
  //     if (ok) {
  //       Serial.println("✅ 夜间归东完成");
  //     } else {
  //       Serial.println("⚠️ 夜间归东失败");
  //     }
  //     hasReturnedEast = true;
  //   }
  //   tgt = -EAST_LIMIT;  // 夜间不动或保持东边等待日出
  // }

  uint32_t sunrise = sunNow.sunrise();
  uint32_t sunset  = sunNow.sunset();
  uint32_t nowUnix = nowTime.unixtime();

  float tgt;
  if (sunEl > 5.0) {
    float ratio = constrain((float)(nowUnix - sunrise) / (sunset - sunrise), 0.0, 1.0);
    tgt = -90.0 + ratio * 180.0;  // -90° 到 +90°
    hasReturnedEast = false;
  } else {
    if (!hasReturnedEast) {
      Serial.println("🌙 太阳落山，执行回东");
      bool ok = moveToAngle(-EAST_LIMIT);
      if (ok) {
        Serial.println("✅ 夜间归东完成");
      } else {
        Serial.println("⚠️ 夜间归东失败");
      }
      hasReturnedEast = true;
    }
    tgt = -EAST_LIMIT;
  }

  // Serial.print("🌞 mockAz="); Serial.print(mockAz);
  // Serial.print("° → 🎯 tgt="); Serial.print(tgt, 1);
  // Serial.print("° → 📍 cur="); Serial.print(curAngle, 1);
  // Serial.print("° → Δ="); Serial.println(fabs(tgt - curAngle), 1);

  ldrEastVal = analogRead(LDR_EAST_PIN);
  ldrWestVal = analogRead(LDR_WEST_PIN);
  // Serial.print("🌅 LDR_EAST="); Serial.print(ldrEastVal);
  // Serial.print("  🌇 LDR_WEST="); Serial.println(ldrWestVal);

  if (fabs(tgt - curAngle) >= ROTATE_THRESHOLD) {
    // Serial.println("🔄 Moving...");
    bool ok = moveToAngle(tgt);
    if (!ok) Serial.println("⚠️ Move failed!");
  } else {
    // Serial.println("✅ No move needed.");
    if (now - lastIdleSample >= SAMPLE_IDLE_MS) {
      float voltage = motorINA.getBusVoltage_V();
      float current = motorINA.getCurrent_mA();
      float power_mW = voltage * current;
      printPowerLog(encCount, lround(tgt * PULSE_PER_DEG), 0, 0);
      power_mW = max(0.0, power_mW);
      energyIdle_mWh += power_mW * SAMPLE_IDLE_MS / 1000.0 / 3600.0;

      // ✅ 累加电机电压电流
      // if (current > 0) {
      //   sumMotorV_idle += voltage;
      //   sumMotorI_idle += current;
      //   sampleCountMotorIdle++;
      // }

      lastIdleSample = now;
    }
  }

  if (now - lastLogicSample >= SAMPLE_OTHER_MS) {
    float logicV = logicINA.getBusVoltage_V();
    float logicI = logicINA.getCurrent_mA();
    float logicP = logicV * logicI;
    energyLogic_mWh += logicP * SAMPLE_OTHER_MS / 1000.0 / 3600.0;
    sumLogicV += logicV;
    sumLogicI += logicI;
    sampleCountLogic++;
    // Serial.print("  Vmcu="); Serial.print(logicV, 2);
    // Serial.print(" V  Imcu="); Serial.print(logicI, 2);
    // Serial.print(" mA  📟 MCU P="); Serial.print(logicP, 1); Serial.println(" mW");
    lastLogicSample = now;
  }

  if (now - lastSolarSample >= SAMPLE_OTHER_MS) {
    float solarV = solarINA.getBusVoltage_V();
    float solarI = solarINA.getCurrent_mA();
    float solarP = solarV * solarI;
    energySolar_mWh += solarP * SAMPLE_OTHER_MS / 1000.0 / 3600.0;
    sumSolarV += solarV;
    sumSolarI += solarI;
    sampleCountSolar++;
    // Serial.print("  Vsolar="); Serial.print(solarV, 2);
    // Serial.print(" V  Isolar="); Serial.print(solarI, 2);
    // Serial.print(" mA  🔆 Solar P="); Serial.print(solarP, 1); Serial.println(" mW");
    lastSolarSample = now;
  }

  if (now - lastUpload >= UPLOAD_INTERVAL_MS) {

    // 🔁 LoRa 掉线检测与自动重连
    if (!modem.connected()) {
      if (millis() - lastJoinAttempt > 300000) {  // 每5分钟尝试一次
        Serial.println("🔁 Attempting LoRa rejoin...");
        // modem.begin(EU868);  // ⚠️ 必须重新初始化 modem
        modem.setPort(1);
        modem.setADR(true);

        if (modem.joinOTAA(appEui, appKey)) {
          Serial.println("✅ Rejoined LoRa");
        } else {
          Serial.println("❌ Rejoin failed");
        }

        lastJoinAttempt = now;
      }
      return;  // ❗提前 return，保留当前角度，⚠️ 不要动 encCount
    }

    float deltaActive = energyActive_mWh - lastEnergyActive_mWh;
    float deltaIdle = energyIdle_mWh - lastEnergyIdle_mWh;
    float deltaLogic = energyLogic_mWh - lastEnergyLogic_mWh;
    float deltaSolar = energySolar_mWh - lastEnergySolar_mWh;

    float avgMotorV_active = sumMotorV_active / max(1, sampleCountMotorActive);
    float avgMotorI_active = sumMotorI_active / max(1, sampleCountMotorActive);
    // float avgMotorV_idle = sumMotorV_idle / max(1, sampleCountMotorIdle);
    // float avgMotorI_idle = sumMotorI_idle / max(1, sampleCountMotorIdle);
    float avgLogicV = sumLogicV / max(1, sampleCountLogic);
    float avgLogicI = sumLogicI / max(1, sampleCountLogic);
    float avgSolarV = sumSolarV / max(1, sampleCountSolar);
    float avgSolarI = sumSolarI / max(1, sampleCountSolar);

    // Serial.println("📡 ⬆️ 模拟上传数据:");
    // Serial.print("sun="); Serial.print(mockAz, 1);
    // Serial.print(" cur="); Serial.print(curAngle, 1);
    // Serial.print(" Ea_motor="); Serial.print(deltaActive, 3);
    // Serial.print(" Ei_motor="); Serial.print(deltaIdle, 3);
    // Serial.print(" Elogial="); Serial.print(deltaLogic, 3);
    // Serial.print(" Esolar="); Serial.print(deltaSolar, 3);
    // Serial.print(" VmA="); Serial.print(avgMotorV_active, 2);
    // Serial.print(" ImA="); Serial.print(avgMotorI_active, 2);
    // // Serial.print(" VmI="); Serial.print(avgMotorV_idle, 2);
    // // Serial.print(" ImI="); Serial.print(avgMotorI_idle, 2);
    // Serial.print(" Vl="); Serial.print(avgLogicV, 2);
    // Serial.print(" Il="); Serial.print(avgLogicI, 2);
    // Serial.print(" Vs="); Serial.print(avgSolarV, 2);
    // Serial.print(" Is="); Serial.print(avgSolarI, 2);
    // Serial.print(" LDR_E="); Serial.print(ldrEastVal);
    // Serial.print(" LDR_W="); Serial.println(ldrWestVal);

    // ⬇️ LoRa 打包并上传（末尾追加）
    uint8_t payload[28];
    int i = 0;
    auto put16 = [&](uint16_t v) { payload[i++] = v & 0xFF; payload[i++] = v >> 8; };
    auto putS16 = [&](int16_t v) { put16((uint16_t)v); };

    putS16(lround(getAngle() * 10));
    putS16(lround(tgt * 10));
    put16((uint16_t)(deltaActive * 1000));
    put16((uint16_t)(deltaIdle * 1000));
    put16((uint16_t)(deltaLogic * 100));
    put16((uint16_t)(deltaSolar * 100));
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

    lastEnergyActive_mWh = energyActive_mWh;
    lastEnergyIdle_mWh = energyIdle_mWh;
    lastEnergyLogic_mWh = energyLogic_mWh;
    lastEnergySolar_mWh = energySolar_mWh;
    sumMotorV_active = sumMotorI_active = 0;
    sumLogicV = sumLogicI = sumSolarV = sumSolarI = 0;
    sampleCountMotorActive = sampleCountMotorIdle = sampleCountLogic = sampleCountSolar = 0;
    lastUpload = now;
  }
}
