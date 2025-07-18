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
const float   PULSE_PER_DEG   = 2.436;
const int16_t ANGLE_LIMIT     = 72;
float         ROTATE_THRESHOLD= 3.0;
const uint32_t DAY_TRACK_MS   = 30UL * 1000;
const uint32_t NIGHT_TRACK_MS = 30UL * 60UL * 1000;

/* ---------- 3. 硬件引脚 ---------- */
const byte IN1 = 3, IN2 = 4, PWM = 5, STBY = 2;
const byte ENC_A = 6, ENC_B = 7;
const uint8_t PWM_DUTY = 60;

Adafruit_INA219 ina(0x41);
Adafruit_INA219 logicINA(0x44);     // 逻辑
Adafruit_INA219 solarINA(0x40);     // 光伏

/* ---------- 4. 编码器 ---------- */
volatile long encCount = 0;
void isrEnc() {
  encCount += digitalRead(ENC_B) ? -1 : +1;
}
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

/* ---------- 6. 精确旋转 ---------- */
bool moveToAngle(float targetDeg, uint32_t timeout_ms = 8000) {
  targetDeg = constrain(targetDeg, -ANGLE_LIMIT, ANGLE_LIMIT);
  long tgtPulse   = lround(targetDeg * PULSE_PER_DEG);
  long limitPulse = ANGLE_LIMIT * PULSE_PER_DEG;
  tgtPulse        = constrain(tgtPulse, -limitPulse, limitPulse);

  bool fwd = (tgtPulse > encCount);
  long prev = encCount;

  Serial.print("🔁 ");
  Serial.print(fwd ? "CW" : "CCW");
  Serial.print(" → ");
  Serial.print(targetDeg, 1);
  Serial.print("° (pulse ");
  Serial.print(tgtPulse);
  Serial.println(")");

  motorRun(fwd, PWM_DUTY);
  uint32_t t0 = millis();
  while ((fwd ? encCount < tgtPulse : encCount > tgtPulse)) {
    if (millis() - t0 > timeout_ms) {
      Serial.println("⚠️  timeout, abort!");
      motorBrake();
      return false;
    }
    if (millis() - t0 > 1500 && encCount == prev) {
      Serial.println("❌ Encoder stuck or not moving!");
      motorBrake();
      return false;
    }
  }
  motorBrake();
  return true;
}

/* ---------- 7. 手动归零 ---------- */
void waitForManualZero() {
  Serial.println("📢 请手动将电机转到 0 位（正南），完成后输入 yes 回车...");
  Serial.flush();

  while (true) {
    if (Serial.available()) {
      String cmd = Serial.readStringUntil('\n');
      cmd.trim();
      if (cmd.equalsIgnoreCase("yes")) {
        encCount = 0;
        Serial.println("✅ 编码器已清零");
        break;
      } else {
        Serial.println("⏳ 未识别输入，请重新输入：");
      }
    }
  }
}

/* ---------- 8. 初始化 ---------- */
void setup() {
  Serial.begin(115200);
  while (!Serial);  // 确保串口连接

  pinMode(IN1, OUTPUT); pinMode(IN2, OUTPUT); pinMode(PWM, OUTPUT);
  pinMode(STBY, OUTPUT); digitalWrite(STBY, HIGH);
  pinMode(ENC_A, INPUT_PULLUP); pinMode(ENC_B, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ENC_A), isrEnc, RISING); // ✅ 使用 RISING 中断

  Wire.begin();
  ina.begin();

  rtc.begin();
  rtc.setTime(20, 0, 0);
  rtc.setDate(26, 6, 2025);

  waitForManualZero();
  Serial.println("🏁 Homing...");
  moveToAngle(0.0);
  encCount = 0;
  Serial.println("✅ Ready\n");
}

/* ---------- 9. 主循环 ---------- */
void loop() {
  static uint32_t nextPoll = 0;
  static bool night = false;

  if (millis() < nextPoll) return;

  uint32_t utc = rtc.getEpoch();
  sun.compute(LAT, LON, utc);
  float alt = sun.altitude();
  float az  = sun.azimuth();
  float rel = constrain(az - 180.0, -ANGLE_LIMIT, ANGLE_LIMIT);

  Serial.print("☀ alt=");
  Serial.print(alt, 1); Serial.print("°  az=");
  Serial.print(az, 1);  Serial.print("°  tgt=");
  Serial.print(rel, 1); Serial.print("°  cur=");
  Serial.print(getAngle(), 1); Serial.println("°");

  if (alt <= 0) {
    night = true;
    rel = -ANGLE_LIMIT;
    nextPoll = millis() + NIGHT_TRACK_MS;
  } else {
    if (night) {
      Serial.println("🌅 Sunrise!");
      night = false;
    }
    nextPoll = millis() + DAY_TRACK_MS;
  }

  if (abs(rel - getAngle()) >= ROTATE_THRESHOLD) {
    if (!moveToAngle(rel)) {
      Serial.println("💥 Move failed, check encoder wiring!");
    }
  } else {
    Serial.println("🔹 No move.");
  }
}
