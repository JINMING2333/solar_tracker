// ------- 引脚定义 -------
const byte IN1 = 3, IN2 = 4, PWM = 5, STBY = 2;
const byte ENC_A = 6, ENC_B = 7;
const byte PWM_DUTY = 100;

volatile long encCount = 0;

void isrEnc() {
  encCount += digitalRead(ENC_B) ? +1 : -1;
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

// ---------------------- 设置目标角度 ----------------------
const float targetAngleDeg = -90.0;  // 修改这里为你要转的角度
const uint32_t timeoutMs = 10000;   // 最长转动时间

void setup() {
  Serial.begin(115200);
  delay(1000);

  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(PWM, OUTPUT);
  pinMode(STBY, OUTPUT);
  digitalWrite(STBY, HIGH);

  pinMode(ENC_A, INPUT_PULLUP);
  pinMode(ENC_B, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ENC_A), isrEnc, RISING);

  encCount = 0;
  delay(1000);
  Serial.println("🧪 开始角度校准");

  long startCount = encCount;
  long targetPulse = startCount + lround(targetAngleDeg * 5);  // 先用粗估值，例如 5 脉冲/度
  bool fwd = targetPulse > startCount;

  motorRun(fwd, PWM_DUTY);
  uint32_t t0 = millis();
  long lastCount = startCount;

  while ((fwd && encCount < targetPulse) || (!fwd && encCount > targetPulse)) {
    if (millis() - t0 > timeoutMs) {
      Serial.println("⏱ 超时未完成");
      break;
    }
    if (abs(encCount - lastCount) > 5) {
      lastCount = encCount;
      t0 = millis();  // 有移动就重置超时
    }
  }

  motorBrake();

  long endCount = encCount;
  long diff = abs(endCount - startCount);
  float pulsePerDeg = diff / targetAngleDeg;

  Serial.println("✅ 旋转完成");
  Serial.print("脉冲数差值 = ");
  Serial.println(diff);
  Serial.print("推荐 PULSE_PER_DEG = ");
  Serial.println(pulsePerDeg, 4);
}

void loop() {
  // 空
}
