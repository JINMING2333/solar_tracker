#include <Wire.h>
#include <MKRWAN.h>
#include <RTClib.h>
#include <SunPosition.h>
#include <ArduinoLowPower.h>
#include <math.h>
#include <Adafruit_INA219.h>

// ===== 位置与时间 =====
double LAT = 51.538593;      // London
double LON = -0.009006;
// 如果 RTC 存 BST(本地, UTC+1) → 1；如果 RTC 存 UTC → 0（推荐）
int timeZone = 1;
RTC_DS3231 rtc;

// ===== LoRaWAN =====
LoRaModem modem;
const char *appEui = "0000000000000000";
const char *appKey = "5DEF1848C136307477DC8E930E463F8F";
uint32_t lastJoinAttempt = 0;

// ===== 电机/编码器 =====
const byte IN1 = 3, IN2 = 4, PWM = 5, STBY = 2;
const uint8_t PWM_DUTY = 180;

const byte ENC_A = 6, ENC_B = 7;
volatile long encCount = 0;
const float PULSE_PER_DEG = 4.7;
const int16_t EAST_LIMIT = 90;    // -90 = 东端
const int16_t WEST_LIMIT = 90;    // +90 = 西端

// ===== LDR 与微调参数（简化）=====
const int LDR_EAST_PIN = A1;
const int LDR_WEST_PIN = A0;
const bool   LDR_LOW_IS_BRIGHT = true;  // 分压接法：越亮 ADC 越小
const bool   LDR_FLIP_DIR      = false; // 如方向反了改 true
const uint8_t LDR_SAMPLES      = 4;
const float   LDR_STEP_DEG     = 2.0;   // 微调步长
const uint16_t LDR_SETTLE_MS   = 120;   // 每步后等待
const int   LDR_EXIT_THRESH    = 30;    // 差值≤此值 → 结束微调
const uint32_t TRACK_TIME_BUDGET_MS = 2500; // 微调最长时长

// 夜间判定（避免深夜来回折腾）
const float EL_NIGHT = 2.0;  // 太阳高度 ≤ 2° 视为夜里，直接回东端

// —— 太阳板 INA219 —— 
Adafruit_INA219 solarINA(0x40);

// —— 本周期统计（唤醒→入睡之间）——
float solar_energy_mWh = 0.0f;
float solar_sumV = 0.0f, solar_sumI_mA = 0.0f;
uint32_t solar_samples = 0;
uint32_t solar_last_ms = 0;      // 上次采样时间戳
const uint16_t SOLAR_SAMPLE_MS = 100;  // 主动采样间隔（可调：50~200ms

// ===== 基础函数 =====
void isrEnc() { encCount += digitalRead(ENC_B) ? +1 : -1; }
float getAngle() { return encCount / PULSE_PER_DEG; }

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
  tgtDeg = constrain(tgtDeg, -90, 90);
  long tgtPulse = lround(tgtDeg * PULSE_PER_DEG);
  bool fwd = (tgtPulse > encCount);
  motorRun(fwd, PWM_DUTY);
  uint32_t t0 = millis();
  long prev = encCount;

  while (fwd ? encCount < tgtPulse : encCount > tgtPulse) {
    if (millis() - t0 > timeout || (millis() - t0 > 5000 && encCount == prev)) {
      motorBrake(); return false;
    }
    prev = encCount;
  }
  motorBrake();
  encCount = constrain(encCount, -90 * PULSE_PER_DEG, +90 * PULSE_PER_DEG);
  return true;
}

void autoHoming() {                  // 上电：回东端当作 -90°
  const uint32_t timeout = 10000;
  const uint8_t duty = 100;
  long lastCount = encCount; uint32_t start = millis();
  motorRun(false, duty);  // 向东
  while (millis() - start < timeout) {
    delay(200);
    if (encCount == lastCount) break; // 抵到机械限位
    lastCount = encCount; 
  }
  motorBrake();
  encCount = -90 * PULSE_PER_DEG;
  // delay(300);

  // // 2️⃣ 转到西端 (+90°)
  // moveToAngle(+90.0f);  // 如果你有 moveToAngleRobust() 建议用它
  // delay(300);

  // // 3️⃣ 再回到东端并重新校准
  // lastCount = encCount; 
  // start = millis();
  // motorRun(false, duty);
  // while (millis() - start < timeout) {
  //   delay(200);
  //   if (encCount == lastCount) break;
  //   lastCount = encCount; 
  // }
  // motorBrake();
  // encCount = -90 * PULSE_PER_DEG;  // 再次校准 -90°
}


int readLdrAvgRaw(int pin, uint8_t n=LDR_SAMPLES){
  long s=0; for(uint8_t i=0;i<n;i++){ s+=analogRead(pin); delay(2); } return (int)(s/n);
}
int ldrDeltaSuggest(bool &goWest, int &eRaw, int &wRaw){
  eRaw = readLdrAvgRaw(LDR_EAST_PIN);
  wRaw = readLdrAvgRaw(LDR_WEST_PIN);
  int eB = LDR_LOW_IS_BRIGHT ? (1023 - eRaw) : eRaw;  // 亮度(大=亮)
  int wB = LDR_LOW_IS_BRIGHT ? (1023 - wRaw) : wRaw;
  goWest = (wB > eB);          // 西更亮→往西
  if (LDR_FLIP_DIR) goWest = !goWest;
  return abs(wB - eB);
}

// —— 线性（日出→日落）映射：日出→-90°，日落→+90° —— //
float computeLinearTgt(SunPosition& sunNow, const DateTime& nowTime,
                       int eastLimit, int westLimit) {
  float sunEl = sunNow.altitude();
  uint16_t sr = sunNow.sunrise();                  // 分钟 0..1439
  uint16_t ss = sunNow.sunset();
  uint16_t nowMin = nowTime.hour()*60 + nowTime.minute();

  // 夜里/异常：保持东端
  if (sunEl <= EL_NIGHT || ss <= sr || nowMin < sr || nowMin > ss) {
    return -eastLimit;
  }
  float ratio = (float)(nowMin - sr) / (float)(ss - sr);
  ratio = constrain(ratio, 0.0f, 1.0f);

  float tgt = -90.0f + ratio * 180.0f;             // -90 → +90 线性
  return constrain(tgt, -eastLimit, westLimit);
}

// —— 到下一个整分钟（避免 <1.2s 的短睡）——
uint32_t msToNextMinute() {
  DateTime t = rtc.now();
  uint16_t total = t.minute() * 60 + t.second();     // 本小时已过秒数
  uint16_t next  = ((total / 180) + 1) * 180;        // 180s=3min
  uint16_t delta = next - total;
  uint32_t ms = (uint32_t)delta * 1000UL;
  if (ms < 1200) ms += 180000UL;                     // 避免短睡
  return ms;
}

// —— 睡前/醒后 —— //
void beforeSleep(uint32_t sleepMs) {
  motorBrake();
  pinMode(STBY, OUTPUT);
  digitalWrite(STBY, LOW);           // 关闭电机驱动
  modem.sleep(sleepMs / 1000);       // 让 LoRa 也打个盹（若支持）
  LowPower.deepSleep(sleepMs);       // SAMD21 Standby，SRAM 保持
}
void afterWake() {
  Wire.begin();                      // 唤醒后 I2C 重新 init 更稳
  pinMode(STBY, OUTPUT);
  digitalWrite(STBY, HIGH);          // 重新使能电机驱动
  detachInterrupt(digitalPinToInterrupt(ENC_A));
  attachInterrupt(digitalPinToInterrupt(ENC_A), isrEnc, RISING);
}

// ====== setup / loop ======
void setup() {
  Serial.begin(115200); delay(300);
  pinMode(IN1, OUTPUT); pinMode(IN2, OUTPUT); pinMode(PWM, OUTPUT); pinMode(STBY, OUTPUT);
  digitalWrite(STBY, HIGH);

  pinMode(ENC_A, INPUT_PULLUP); pinMode(ENC_B, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ENC_A), isrEnc, RISING);

  Wire.begin(); rtc.begin();

  // LoRa 初始化与入网
  if (!modem.begin(EU868)) { Serial.println("❌ LoRa init failed"); while(1); }
  modem.setADR(true); modem.setPort(1);
  if (modem.joinOTAA(appEui, appKey)) Serial.println("✅ LoRa joined");
  else Serial.println("⚠️ LoRa join failed (will retry later)");
  lastJoinAttempt = millis();

  autoHoming(); // 上电自校准一次（-90°）

  // 首次对齐整分
  uint32_t sleepMs = msToNextMinute();
  // beforeSleep(sleepMs);
  // afterWake();
}

void loop() {
  // —— RTC & SunPosition —— //
  DateTime nowTime = rtc.now();
  SunPosition sunNow(LAT, LON, nowTime.unixtime(), timeZone);
  float sunEl = sunNow.altitude();

  // 目标角（线性法）：无论 LDR，先到目标角
  float tgt = computeLinearTgt(sunNow, nowTime, EAST_LIMIT, WEST_LIMIT);

  // 夜里：回东端，跳过微调
  bool isNight = (sunEl <= EL_NIGHT) || (tgt <= -EAST_LIMIT + 0.5f);
  if (isNight) {
    moveToAngle(-EAST_LIMIT);
  } else {
    // ① 直接粗调到目标角（总是尝试；角度差很小则基本不动）
    moveToAngle(tgt);

    // ② LDR 微调（2° 步），直到差值 ≤ EXIT 或耗尽时间预算
    uint32_t tStart = millis();
    float cur = getAngle();
    while ((millis() - tStart) < TRACK_TIME_BUDGET_MS) {
      bool goW; int eRaw, wRaw;
      int diff = ldrDeltaSuggest(goW, eRaw, wRaw);
      // 上传会用到的快照（放在循环最后也行，这里只是刷新）
      // 但为了节能，通常只在外层上传一次；这里我们只用来判断
      if (diff <= LDR_EXIT_THRESH) break;

      float next = cur + (goW ? +LDR_STEP_DEG : -LDR_STEP_DEG);
      next = constrain(next, -EAST_LIMIT, WEST_LIMIT);
      if (fabs(next - cur) < 0.4f) break;  // 已到边缘
      if (!moveToAngle(next)) break;
      delay(LDR_SETTLE_MS);
      cur = getAngle();
    }
  }

  // —— LoRa 上报（12B：cur，tgt，LDR_E，LDR_W，sunEl，nowMin）——
  // 读一次最新 LDR（用于上报）
  int eRaw = readLdrAvgRaw(LDR_EAST_PIN);
  int wRaw = readLdrAvgRaw(LDR_WEST_PIN);

  if (!modem.connected()) {
    if (millis() - lastJoinAttempt > 300000UL) {  // 每5分钟尝试重入网
      Serial.println("🔁 Rejoin LoRa...");
      if (modem.joinOTAA(appEui, appKey)) Serial.println("✅ Rejoined");
      else Serial.println("❌ Rejoin failed");
      lastJoinAttempt = millis();
    }
  } else {
    uint8_t payload[12]; int i = 0;
    auto put16  = [&](uint16_t v){ payload[i++]= v & 0xFF; payload[i++]= v >> 8; };
    auto putS16 = [&](int16_t v){ put16((uint16_t)v); };

    int16_t cur_x10 = (int16_t)lround(getAngle() * 10.0f);
    int16_t tgt_x10 = (int16_t)lround(tgt * 10.0f);
    int16_t el_x10  = (int16_t)lround(sunEl * 10.0f);
    uint16_t nowMin = nowTime.hour()*60 + nowTime.minute();

    putS16(cur_x10);
    putS16(tgt_x10);
    put16((uint16_t)eRaw);
    put16((uint16_t)wRaw);
    putS16(el_x10);
    put16(nowMin);

    modem.beginPacket();
    modem.write(payload, sizeof(payload));
    int res = modem.endPacket(false);
    Serial.println(res == 0 ? "📡 LoRa OK" : "📡 LoRa FAIL");
  }

  // —— 简短日志 —— 
  Serial.print("RTC ");
  Serial.print(nowTime.hour()); Serial.print(':'); Serial.print(nowTime.minute());
  Serial.print(" | el=");  Serial.print(sunEl,1);
  Serial.print(" tgt=");   Serial.print(tgt,1);
  Serial.print(" cur=");   Serial.println(getAngle(),1);

  // —— 睡到下一整分钟（测试阶段 1 分钟；未来改 10 分钟） —— 
  uint32_t sleepMs = msToNextMinute(); // 将来可改为 600000UL（10min）
  beforeSleep(sleepMs);
  afterWake();
}
