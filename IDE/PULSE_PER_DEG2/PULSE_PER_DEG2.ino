// ====== 简易校准 & 三点位测试 ======
#include <Arduino.h>

// 电机驱动脚（按你现有连线）
const byte IN1 = 3, IN2 = 4, PWM = 5, STBY = 2;
uint8_t PWM_DUTY = 180;      // 工作占空比(可按负载微调)
const uint8_t PWM_KICK = 255; // 起步“踢一下”
const uint16_t KICK_MS = 200;

// 编码器
const byte ENC_A = 6, ENC_B = 7;
volatile long encCount = 0;

// 你当前使用的估计值（待校准）
float PULSE_PER_DEG = 4.8;   

// 机械限制（角度定义）
const int16_t EAST_LIMIT_DEG = -90;
const int16_t WEST_LIMIT_DEG =  90;

// —— 中断读取方向 —— 
void isrEnc() {
  encCount += digitalRead(ENC_B) ? +1 : -1;
}

// —— 基本电机控制 —— 
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

// —— 朝指定“脉冲”移动（不依赖 PULSE_PER_DEG）——
bool moveToPulse(long tgtPulse, uint32_t timeout = 15000) {
  bool fwd = (tgtPulse > encCount);
  // 起步踢一下
  motorRun(fwd, PWM_KICK);
  delay(KICK_MS);
  motorRun(fwd, PWM_DUTY);

  uint32_t t0 = millis();
  long last = encCount;
  uint32_t lastChange = millis();

  while (fwd ? (encCount < tgtPulse) : (encCount > tgtPulse)) {
    if (encCount != last) { last = encCount; lastChange = millis(); }
    // 3秒没动，补踢一次；再不动就失败
    if (millis() - lastChange > 3000) {
      long before = encCount;
      motorRun(fwd, PWM_KICK); delay(150);
      motorRun(fwd, PWM_DUTY);
      lastChange = millis();
      if (encCount == before) { motorBrake(); return false; }
    }
    if (millis() - t0 > timeout) { motorBrake(); return false; }
  }
  motorBrake();
  return true;
}

// —— 朝某个“角度”移动（使用当前 PULSE_PER_DEG）——
bool moveToAngle(float deg, uint32_t timeout = 15000) {
  deg = constrain(deg, EAST_LIMIT_DEG, WEST_LIMIT_DEG);
  long tgtPulse = lround(deg * PULSE_PER_DEG);
  return moveToPulse(tgtPulse, timeout);
}

// —— 沿某方向慢速找极限，返回极限时的 encCount ——
// 说明：fwd=false 表示向“东”（角度减小）；fwd=true 表示向“西”（角度增大）。
long findLimit(bool fwd) {
  const uint8_t duty = 160;
  Serial.println(fwd ? F("→ 寻找西端...") : F("→ 寻找东端..."));
  motorRun(fwd, duty);
  long last = encCount;
  uint32_t lastChange = millis();
  while (true) {
    if (encCount != last) { last = encCount; lastChange = millis(); }
    // 400ms 没变化视为触底
    if (millis() - lastChange > 400) break;
  }
  motorBrake();
  delay(300);
  Serial.print(F("   触底 encCount=")); Serial.println(encCount);
  return encCount;
}

void setup() {
  Serial.begin(115200);
  delay(500);

  pinMode(IN1, OUTPUT); pinMode(IN2, OUTPUT); pinMode(PWM, OUTPUT); pinMode(STBY, OUTPUT);
  digitalWrite(STBY, HIGH);

  pinMode(ENC_A, INPUT_PULLUP); pinMode(ENC_B, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ENC_A), isrEnc, RISING);

  Serial.println(F("=== 太阳能追踪校准测试 ==="));

  // 1) 找到东/西两个机械极限（只基于编码器脉冲）
  long eastPulse = findLimit(false); // 向东找
  delay(600);
  long westPulse = findLimit(true);  // 向西找
  delay(600);

  // 回到中间，避免卡在端点
  long midPulse_hw = (eastPulse + westPulse) / 2;
  moveToPulse(midPulse_hw);
  delay(500);

  // 2) 计算建议的 PULSE_PER_DEG（以 180° 跨度为准）
  long span = westPulse - eastPulse;
  float ppd_suggest = (span > 0) ? (span / 180.0f) : PULSE_PER_DEG;

  Serial.println(F("\n--- 校准结果 ---"));
  Serial.print(F("eastPulse = ")); Serial.println(eastPulse);
  Serial.print(F("westPulse = ")); Serial.println(westPulse);
  Serial.print(F("span      = ")); Serial.println(span);
  Serial.print(F("建议 PULSE_PER_DEG = ")); Serial.println(ppd_suggest, 4);
  Serial.println(F("-----------------\n"));

  // 3) 用“当前 PULSE_PER_DEG”测试 -90/0/+90 三个点位
  auto report = [&](const char* name, float tgtDeg){
    long tgtPulse_byAngle = lround(tgtDeg * PULSE_PER_DEG);
    bool ok = moveToAngle(tgtDeg);
    long actual = encCount;
    long errPulse = actual - tgtPulse_byAngle;
    float errDeg_bySuggest = (span>0) ? (errPulse / ppd_suggest) : 0.0f;

    Serial.print(name);
    Serial.print(F("  目标角=")); Serial.print(tgtDeg, 1);
    Serial.print(F("°  目标脉冲(按当前PPD)=")); Serial.print(tgtPulse_byAngle);
    Serial.print(F("  实际脉冲=")); Serial.print(actual);
    Serial.print(F("  脉冲误差=")); Serial.print(errPulse);
    Serial.print(F("  角度误差(按建议PPD)=")); Serial.print(errDeg_bySuggest, 2);
    Serial.print(F("°  结果=")); Serial.println(ok ? F("OK") : F("FAIL"));
    delay(800);
  };

  Serial.println(F("开始三点位测试（使用你当前的 PULSE_PER_DEG）..."));
  report("点位一：-90°", -90.0f);
  report("点位二：  0°",   0.0f);
  report("点位三：+90°", +90.0f);

  Serial.println(F("\n提示：若角度误差较大，请把上面打印的“建议 PULSE_PER_DEG”替换到代码里重试。"));
}

void loop() {
  // 空循环
}
