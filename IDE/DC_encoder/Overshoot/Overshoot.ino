/* ---------- 删除连续打印，只保留 overshoot ---------- */

#include <Arduino.h>

const byte IN1  = 3;
const byte IN2  = 4;
const byte PWM  = 5;
const byte STBY = 2;

const byte ENC_A = 6;
const byte ENC_B = 7;

const int  speedPWM      = 120;
const int  STEP_DEG      = 12;
const byte STEP_COUNT    = 10;
const unsigned long DWELL_MS    = 30000UL; // 30 s
const unsigned long FINAL_DWELL = 60000UL; // 60 s

const float PULSE_PER_DEG = 2.4;   // ← 你的实测数

long baseCount = 0;
double targetPulse = 0; 

volatile long encCount = 0;
void isrEncA() { encCount += digitalRead(ENC_B) ? -1 : +1; }

/* ------ 只留下电机驱动辅助函数 ------ */
void motorRun(bool fwd, uint8_t pwm)
{
  digitalWrite(IN1, fwd ? HIGH : LOW);
  digitalWrite(IN2, fwd ? LOW  : HIGH);
  analogWrite (PWM, pwm);
}
void motorBrake()
{
  analogWrite(PWM, 0);
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
}

void moveByDeg(int deg)
{
  bool fwd    = (deg > 0);
  targetPulse += deg * PULSE_PER_DEG;
  long target = baseCount + lround(targetPulse);

  motorRun(fwd, speedPWM);
  while (fwd ? encCount < target : encCount > target) { /* idle */ }
  motorBrake();

  float overs = (encCount - target) / PULSE_PER_DEG;
  Serial.print(F("overshoot: "));
  Serial.print(overs, 2);
  Serial.println(F(" deg"));
}

/* ---------- 简化后的主流程 ---------- */
void setup()
{
  Serial.begin(115200);

  pinMode(IN1, OUTPUT); pinMode(IN2, OUTPUT);
  pinMode(PWM, OUTPUT); pinMode(STBY, OUTPUT);
  digitalWrite(STBY, HIGH);

  pinMode(ENC_A, INPUT_PULLUP);
  pinMode(ENC_B, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ENC_A), isrEncA, RISING);

  encCount = 0;

  /* 等 “START” */
  // while (!Serial.available()) {}
  // while (Serial.available()) Serial.read();

  /* ---- 执行完整 0→120→0 周期 ---- */
  for (int i = 1; i <= STEP_COUNT; ++i) {
    delay(DWELL_MS);          // 30 s
    moveByDeg(STEP_DEG);      // +12 °
  }
  delay(FINAL_DWELL);         // 60 s 顶部驻停
  moveByDeg(-STEP_DEG * STEP_COUNT); // -120° 回原点

  /* 结束后比对是否回到基准 */
  long errPulse = encCount - baseCount;
  float errDeg  = errPulse / PULSE_PER_DEG;
  Serial.print(F("Return error = "));
  Serial.print(errDeg, 2);
  Serial.println(F(" deg"));

  Serial.println(F("Cycle done."));
}

void loop() {}
