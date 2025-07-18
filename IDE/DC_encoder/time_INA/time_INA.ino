#include <Arduino.h>
#include <Wire.h>
#include <SunPosition.h>
#include <Adafruit_INA219.h>
#include <RTCZero.h>

/* ---------- 1. 地理与时间 ---------- */
const float LAT = 51.5381, LON = -0.0099;
RTCZero rtc;
SunPosition sun;

/* ---------- 2. 追踪参数 ---------- */
const float   PULSE_PER_DEG    = 2.436;
const int16_t West_LIMIT  = 72;          // 面板顺时针(西侧)最大 +72 °
const int16_t East_LIMIT = 60;          // 面板逆时针（东侧)最大 –60 °
float         ROTATE_THRESHOLD  = 5.0;
const uint32_t DAY_TRACK_MS    = 1UL * 60UL * 1000;
const uint32_t NIGHT_TRACK_MS  = 30UL * 60UL * 1000;

/* 回差补偿量（测试后微调） */
const long BACKLASH_PULSES_CW  = 0;
const long BACKLASH_PULSES_CCW = 0;

/* ---------- 3. 硬件引脚 ---------- */
const byte IN1 = 3, IN2 = 4, PWM = 5, STBY = 2;
const byte ENC_A = 6, ENC_B = 7;
const uint8_t PWM_DUTY = 60;

Adafruit_INA219 motorINA(0x41);
Adafruit_INA219 logicINA(0x44);
Adafruit_INA219 solarINA(0x40);

/* ---------- LDR ---------- */
const byte  LDR_EAST_PIN = A0;   // 朝东
const byte  LDR_WEST_PIN = A1;   // 朝西
const uint32_t LDR_SAMPLE_MS = 60UL * 1000;   // 1 min 采样

/* ---------- 4. 编码器 ---------- */
volatile long encCount = 0;
void isrEnc() { encCount += digitalRead(ENC_B) ? -1 : +1; }
float getAngle() { return encCount / PULSE_PER_DEG; }

/* ---------- 5. 电机控制 ---------- */
void motorRun(bool fwd, uint8_t duty) {
  digitalWrite(IN1, fwd ? HIGH : LOW);
  digitalWrite(IN2, fwd ? LOW  : HIGH);
  analogWrite(PWM, duty);
}
void motorBrake() {
  analogWrite(PWM, 0);
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
}

/* ---------- 6. INA219 Logging ---------- */
void logINA219(Adafruit_INA219& sensor, const char* label) {
  float power = sensor.getPower_mW();
  Serial.print(label);
  Serial.print(power, 2);
  Serial.println(" mW");
}

/* ---------- 7. 精确旋转（含补偿） ---------- */
bool moveToAngle(float targetDeg, uint32_t timeout_ms = 8000) {
  targetDeg = constrain(targetDeg, -East_LIMIT, West_LIMIT);
  long  limitPulseWest  = West_LIMIT  * PULSE_PER_DEG;
  long  limitPulseEast = East_LIMIT * PULSE_PER_DEG;
  long basePulse   = lround(targetDeg * PULSE_PER_DEG);
  basePulse        = constrain(basePulse, -limitPulseEast, limitPulseWest);

  bool fwd = (basePulse > encCount);
  long tgtPulse = basePulse + (fwd ? BACKLASH_PULSES_CW : -BACKLASH_PULSES_CCW);

  Serial.print("🔁 "); Serial.print(fwd?"West":"East");
  Serial.print(" → "); Serial.print(targetDeg,1);
  Serial.print("° (pulse "); Serial.print(tgtPulse); Serial.println(")");

  motorRun(fwd, PWM_DUTY);
  uint32_t t0 = millis(); 
  uint32_t lastSample = t0;
  long prev = encCount;
  while (fwd? encCount<tgtPulse : encCount>tgtPulse) {
    if (millis()-t0 > timeout_ms) {
      Serial.println("⚠️ timeout, abort!"); 
      motorBrake(); 
      return false; }
    if (millis()-t0 > 1500 && encCount == prev) {
      Serial.println("❌ Encoder stuck!"); 
      motorBrake(); 
      return false; }
    if (millis() - lastSample >= 50) {
      Serial.print("🔎 Moving\n");
      logINA219(motorINA, "⚙ Motor: ");
      lastSample = millis();
    }
  }
  motorBrake(); 
  /* --- 钳位编码器，防止惯性或抖动把计数跑出 ±66° --- */
  encCount = constrain(encCount, -limitPulseEast, limitPulseWest);
  return true;
}

/* ---------- 8. 手动归零 ---------- */
void waitForManualZero() {
  Serial.println("📢 请手动将电机转到 0 位（正南），完成后输入 yes 回车...");
  Serial.flush();
  while (true) {
    if (Serial.available()) {
      String cmd = Serial.readStringUntil('\n'); cmd.trim();
      if (cmd.equalsIgnoreCase("yes")) { encCount = 0; Serial.println("✅ 编码器已清零"); break; }
      else { Serial.println("⏳ 未识别输入，请重新输入："); }
    }
  }
}

/* ---------- 9. 初始化 ---------- */
void setup() {
  Serial.begin(115200); 
  while (!Serial);

  pinMode(IN1,OUTPUT); pinMode(IN2,OUTPUT); pinMode(PWM,OUTPUT);
  pinMode(STBY,OUTPUT); digitalWrite(STBY,HIGH);
  pinMode(ENC_A,INPUT_PULLUP); pinMode(ENC_B,INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ENC_A),isrEnc,RISING);

  Wire.begin(); 
  motorINA.begin(); 
  logicINA.begin(); 
  solarINA.begin();

  rtc.begin(); 
  rtc.setTime(20,10,0); 
  rtc.setDate(26,6,2025);

  pinMode(LDR_EAST_PIN, INPUT);
  pinMode(LDR_WEST_PIN, INPUT);

  waitForManualZero();
  Serial.println("🏁 Homing...");
  moveToAngle(0.0);
  encCount=0;
  Serial.println("✅ Ready\n");
}

/* ---------- 10. 主循环 ---------- */
void loop() {
  static uint32_t nextPoll = 0;
  static bool night = false;
  static uint32_t lastSolarSample = 0;
  static uint32_t lastLogicSample = 0;
  static uint32_t lastMotorSample = 0;
  static uint32_t lastLdrSample = 0;

  uint32_t now = millis();

  /* ---------- INA219 采样 1min,1min,5min---------- */
  if (now - lastMotorSample >= 60UL * 1000) {
    logINA219(motorINA, "⚙ Motor: ");
    lastMotorSample = now;
  }
  if (now - lastSolarSample >= 60UL * 1000) {
    logINA219(solarINA, "☀ Solar: ");
    lastSolarSample = now;
  }
  if (now - lastLogicSample >= 5UL * 60UL * 1000) {
    logINA219(logicINA, "🧠 Logic: ");
    Serial.print("  diff=");      
    Serial.println(diff);
    lastLogicSample = now;
  }

  /* ---------- LDR 采样 ---------- */
  if (now - lastLdrSample >= LDR_SAMPLE_MS) {
    int eastRaw = analogRead(LDR_EAST_PIN);   // 0-1023, 值越大 → 越亮
    int westRaw = analogRead(LDR_WEST_PIN);
    int diff    = westRaw - eastRaw;          // >0: 西侧更亮; <0: 东侧更亮

    Serial.print("🌄 LDR east="); Serial.print(eastRaw);
    Serial.print("  west=");      Serial.print(westRaw);
    Serial.print("  diff=");      Serial.println(diff);
    // 未来可在此处用 diff 与阈值比较，决定是否微调面板

    lastLdrSample = now;
  }

  /* ---------- 追踪逻辑 ---------- */
  if (now < nextPoll) return;

  uint32_t utc = rtc.getEpoch();
  sun.compute(LAT, LON, utc);
  float alt = sun.altitude();
  float az  = sun.azimuth();
  float rel = constrain(az - 180.0, -East_LIMIT, West_LIMIT);

  Serial.print("☀ alt="); Serial.print(alt, 1);
  Serial.print("°  az="); Serial.print(az, 1);
  Serial.print("°  tgt="); Serial.print(rel, 1);
  Serial.print("°  cur="); Serial.print(getAngle(), 1);
  Serial.println("°");

  if (alt <= 0) {
    /* ---------------- 夜间处理 ---------------- */
    if (!night) {
      night = true;
      Serial.println("🌙 Night – return east");
      moveToAngle(-East_LIMIT); 
      encCount=lround(-East_LIMIT*PULSE_PER_DEG); 
    }
    nextPoll = now + NIGHT_TRACK_MS;
    return;                               // 夜间不再执行白天逻辑
  } else {
    /* ---------------- 白天处理 ---------------- */
    if (night) {
      Serial.println("🌅 Sunrise!");
      night = false;
    }
    nextPoll = now + DAY_TRACK_MS;

    if (abs(rel - getAngle()) >= ROTATE_THRESHOLD) {
      if (!moveToAngle(rel)) {
        Serial.println("💥 Move failed, check encoder wiring!");
      }
    } else {
      Serial.println("🔹 No move.");
    }
  }
}
