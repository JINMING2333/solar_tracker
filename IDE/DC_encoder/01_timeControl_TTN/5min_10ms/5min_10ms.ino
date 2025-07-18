/********************************************************************
 *  Solar‑Tracker  +  MKR WAN 1310
 *  1‑min LoRaWAN uplink — 仅上传「上一帧周期」的能量 (0.01 mWh) 与转动时长 (0.1 s)
 *  2025‑06‑26  修订：
 *    • INA219 mW·ms → 0.01 mWh，16‑bit 上限 655.35 mWh/帧
 *    • moveTime 累加 0.1 s 分辨率
 *    • uplink 采用 Unconfirmed；每帧结束即清零窗口计数
 *******************************************************************/

#include <MKRWAN.h>
#include <Wire.h>
#include <Adafruit_INA219.h>
#include <SunPosition.h>
#include <RTCZero.h>

// ─────────────────────────  LoRaWAN ─────────────────────────
LoRaModem modem;
const char *appEui = "0000000000000000";
const char *appKey = "5DEF1848C136307477DC8E930E463F8F";
const uint32_t UPLINK_MS = 5 * 60UL * 1000;  // uplink interval
uint32_t lastUplink = 0;

// ─────────────────────────  Sensors  ─────────────────────────
Adafruit_INA219 motorINA(0x41), logicINA(0x44), solarINA(0x40);
volatile long encCount = 0;  // encoder pulses

// —— 1‑min window accumulators ——
uint32_t solar_mWs = 0, motor_mWs = 0, logic_mWs = 0;  // mW·ms
uint32_t moveTimeMs = 0;                              // ms

// —— sampling timers ——
uint32_t lastMotor50 = 0;   // 50 ms motor power
uint32_t lastEnergy1s = 0;  // 1 s other power

// ─────────────────────────  Astro & RTC  ────────────────────
const float LAT = 51.538593, LON = -0.009006;
RTCZero     rtc;
SunPosition sun;

// ─────────────────────────  Tracker params  ────────────────
const float   PULSE_PER_DEG = 2.436;
const int16_t WEST_LIMIT = 72, EAST_LIMIT = 72;   // °
float         ROTATE_THRESHOLD = 5.0;             // °
const uint32_t DAY_TRACK_MS = 60UL * 1000;
const uint32_t NIGHT_TRACK_MS = 30 * 60UL * 1000;

// —— state ——
uint32_t nextPoll = 0;  // when to check next
bool     night    = false;

// ─────────────────────────  Pins  ──────────────────────────
const byte IN1 = 3, IN2 = 4, PWM = 5, STBY = 2;
const byte ENC_A = 6, ENC_B = 7;
const uint8_t PWM_DUTY = 100;
const byte LDR_EAST = A1, LDR_WEST = A0;

// ─────────────────────────  Helpers  ───────────────────────
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

bool moveToAngle(float tgtDeg, uint32_t timeout = 8000) {
  tgtDeg = constrain(tgtDeg, -EAST_LIMIT, WEST_LIMIT);
  long tgtPulse = lround(tgtDeg * PULSE_PER_DEG);
  bool fwd = tgtPulse > encCount;
  motorRun(fwd, PWM_DUTY);
  uint32_t t0 = millis();
  uint32_t tSample = t0;              // ★ 声明并初始化
  long prev = encCount;

  while (fwd ? encCount < tgtPulse : encCount > tgtPulse) {
    /* ← 新增：每 10 ms 累加一次 motor 能量 */
    uint32_t now = millis();
    if (now - tSample >= 10) {                           // 10 ms 粗采
      motor_mWs += motorINA.getPower_mW() * (now - tSample);
      tSample = now;
    }

    if (millis() - t0 > timeout || (millis() - t0 > 1500 && encCount == prev)) {
      motorBrake();
      return false;
    }
  }

  motorBrake();

  /* ★★★ 最后一次补齐 ★★★ */
  uint32_t now = millis();
  motor_mWs  += motorINA.getPower_mW() * (now - tSample);
  moveTimeMs += now - t0;
  encCount = constrain(encCount, -(long)(EAST_LIMIT * PULSE_PER_DEG), (long)(WEST_LIMIT * PULSE_PER_DEG));
  return true;
}

inline void put16(uint8_t *p, int &i, uint16_t v) {
  p[i++] = v & 0xFF;
  p[i++] = v >> 8;
}

inline void putS16(uint8_t *p, int &i, int16_t v) {
  put16(p, i, (uint16_t)v);
}


// ─────────────────────────  setup  ─────────────────────────
void setup() {
  pinMode(LED_BUILTIN, OUTPUT); // 板载LED作为状态指示

  Serial.begin(115200);
  delay(500);

  digitalWrite(LED_BUILTIN, HIGH);  // 状态0：上电
  delay(300);

  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(PWM, OUTPUT);
  pinMode(STBY, OUTPUT); digitalWrite(STBY, HIGH);
  pinMode(ENC_A, INPUT_PULLUP);
  pinMode(ENC_B, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ENC_A), isrEnc, RISING);

  Wire.begin();
  motorINA.begin();
  logicINA.begin();
  solarINA.begin();
  motorINA.setCalibration_32V_2A();
  logicINA.setCalibration_32V_2A();
  solarINA.setCalibration_32V_2A();

  rtc.begin();
  rtc.setTime(15, 50, 0); 

  rtc.setDate(3, 7, 2025);

  encCount = 0;
  digitalWrite(LED_BUILTIN, LOW);  // 状态1：初始化完毕
  delay(300);

  if (!modem.begin(EU868)) {
    digitalWrite(LED_BUILTIN, HIGH);  // 卡在 modem 初始化
    while (1);
  }

  modem.setADR(true);
  modem.setPort(1);

  if (!modem.joinOTAA(appEui, appKey)) {
    // 快速闪烁表示 Join 失败
    for (int i = 0; i < 10; i++) {
      digitalWrite(LED_BUILTIN, HIGH); delay(100);
      digitalWrite(LED_BUILTIN, LOW);  delay(100);
    }
    while (1);
  }

  // 🎉 加入成功后慢闪 3 次
  for (int i = 0; i < 3; i++) {
    digitalWrite(LED_BUILTIN, HIGH); delay(300);
    digitalWrite(LED_BUILTIN, LOW);  delay(300);
  }

  Serial.println("🎉 LoRaWAN joined");

  lastMotor50 = lastEnergy1s = millis();
}

// ─────────────────────────  loop  ──────────────────────────
void loop() {
  uint32_t now = millis();

  // —— integrate power ——
  if (now - lastMotor50 >= 50) {
    uint32_t dt = now - lastMotor50;
    motor_mWs += motorINA.getPower_mW() * dt;
    lastMotor50 = now;
  }

  if (now - lastEnergy1s >= 1000) {
    uint32_t dt = now - lastEnergy1s;
    solar_mWs += solarINA.getPower_mW() * dt;
    logic_mWs += logicINA.getPower_mW() * dt;
    lastEnergy1s = now;
  }

  // —— LoRa uplink ——
  if (now - lastUplink >= UPLINK_MS) {
    uint32_t utc = rtc.getEpoch();
    const long TZ_OFFSET = 3600;   // BST (+1 h)；冬令时用 0
    sun.compute(LAT, LON, utc - TZ_OFFSET);
    float alt = sun.altitude();
    float az = sun.azimuth();
    float rel = constrain(az - 180.0, -EAST_LIMIT, WEST_LIMIT);

    uint16_t tMin = (utc / 60UL) % 1440UL;
    int16_t cur10 = lround(getAngle() * 10);
    int16_t tgt10 = lround(rel * 10);
    int16_t alt10 = lround(alt * 10);

    uint16_t solar01 = min<uint32_t>(solar_mWs / 3600UL, 65535);
    uint16_t motor01 = min<uint32_t>(motor_mWs / 3600UL, 65535);
    uint16_t logic01 = min<uint32_t>(logic_mWs / 3600UL, 65535);

    uint16_t ldrE = analogRead(LDR_EAST);
    uint16_t ldrW = analogRead(LDR_WEST);
    uint16_t motor01s = min<uint32_t>(moveTimeMs / 100UL, 65535);

    uint8_t buf[22];
    int i = 0;
    put16(buf, i, tMin);
    putS16(buf, i, cur10);
    putS16(buf, i, tgt10);
    putS16(buf, i, alt10);
    put16(buf, i, solar01);
    put16(buf, i, motor01);
    put16(buf, i, logic01);
    put16(buf, i, ldrE);
    put16(buf, i, ldrW);
    put16(buf, i, motor01s);

    modem.beginPacket();
    modem.write(buf, sizeof(buf));
    int r = modem.endPacket(false);
    Serial.println(r == 0 ? "LoRa OK" : "LoRa ERR");

    solar_mWs = motor_mWs = logic_mWs = 0;
    moveTimeMs = 0;
    lastUplink = now;
  }

  // —— tracking logic ——
  if (now < nextPoll) return;

  uint32_t utc = rtc.getEpoch();
  const long TZ_OFFSET = 3600;   // BST (-1 h)；冬令时用 0
  sun.compute(LAT, LON, utc - TZ_OFFSET);
  float alt = sun.altitude();
  float az = sun.azimuth();
  float rel = constrain(az - 180.0, -EAST_LIMIT, WEST_LIMIT);

  Serial.print("☀ alt=");
  Serial.print(alt, 1);
  Serial.print("° az=");
  Serial.print(az, 1);
  Serial.print("° tgt=");
  Serial.print(rel, 1);
  Serial.print("° cur=");
  Serial.println(getAngle(), 1);

  Serial.print("encCount: ");
  Serial.println(encCount);

  if (alt <= 0) {
    if (!night) {
      night = true;
      Serial.println("🌙 Night – return east");
      moveToAngle(-EAST_LIMIT);
      encCount = lround(-EAST_LIMIT * PULSE_PER_DEG);
    }
    nextPoll = now + NIGHT_TRACK_MS;
    return;
  } else {
    if (night) {
      night = false;
      Serial.println("🌅 Sunrise!");
    }

    nextPoll = now + DAY_TRACK_MS;
    if (fabs(rel - getAngle()) >= ROTATE_THRESHOLD) {
      // Serial.print("REL");
      // Serial.println(rel);
      // Serial.print("getAngle");
      // Serial.println(getAngle());
      if (!moveToAngle(rel)) Serial.println("💥 Move failed");
    } else {
      Serial.println("🔹 No move.");
    }
  }
}
