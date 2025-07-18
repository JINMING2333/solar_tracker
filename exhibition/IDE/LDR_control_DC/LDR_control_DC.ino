// ----------- 引脚定义 -----------
const int LDR_rotating_L = A0;   // 单轴左
const int LDR_rotating_R = A1;   // 单轴右
const int LDR_fixed_L    = A2;   // 固定左（仅监测）
const int LDR_fixed_R    = A3;   // 固定右（仅监测）

// TB6612 直流电机驱动
const int PWMA = 5;
const int AIN1 = 7;
const int AIN2 = 4;
const int STBY = 6;

// 灯泡常亮（MOSFET 栅极）
const int lampMosfetPin = 3;

// ----------- 阈值与脉冲参数 -----------
const int LIGHT_THRESHOLD = 800; // 至少一侧 > 800 才考虑转动

const int THRESH_START = 40;     // 启动阈值（差值 > 60 时脉冲）
const int THRESH_STOP  = 20;     // 停止阈值（差值 ≤ 20 视为平衡）

const int MOTOR_SPEED = 100;     // PWM 占空
const int PULSE_MS    = 120;     // 单次脉冲持续
const int REST_MS     = 200;     // 脉冲结束后的等待

// ----------- 状态变量 -----------
bool motorRunning          = false;     // 当前是否处于脉冲阶段
unsigned long pulseStartMs = 0;         // 本次脉冲开始时刻

void setup() {
  Serial.begin(9600);

  // 灯泡常亮
  pinMode(lampMosfetPin, OUTPUT);
  digitalWrite(lampMosfetPin, HIGH);
  Serial.println("灯泡常亮");

  // 驱动与IO
  pinMode(PWMA, OUTPUT);
  pinMode(AIN1, OUTPUT);
  pinMode(AIN2, OUTPUT);
  pinMode(STBY, OUTPUT);
  digitalWrite(STBY, HIGH);

  Serial.println("系统初始化完成");
}

void loop() {

  // ---- 读取全部 LDR ----
  int rotL = analogRead(LDR_rotating_L);
  int rotR = analogRead(LDR_rotating_R);
  int fixL = analogRead(LDR_fixed_L);
  int fixR = analogRead(LDR_fixed_R);

  Serial.print("旋转左:"); Serial.print(rotL);
  Serial.print(" 右:");   Serial.print(rotR);
  Serial.print(" || 固定左:"); Serial.print(fixL);
  Serial.print(" 右:");   Serial.println(fixR);

  // ---- 如果当前不在脉冲中，决定是否发起新脉冲 ----
  if (!motorRunning) {
    int diff = abs(rotL - rotR);

    if ((rotL > LIGHT_THRESHOLD || rotR > LIGHT_THRESHOLD) && diff > THRESH_START) {
      if (rotL > rotR) pulseMotorLeft();
      else             pulseMotorRight();
    }
  }

  // ---- 检查脉冲到时并停机 ----
  checkPulseTimeout();
}

// ================= 电机控制函数 =================
void pulseMotorLeft() {
  digitalWrite(AIN1, LOW);
  digitalWrite(AIN2, HIGH);
  analogWrite(PWMA, MOTOR_SPEED);

  motorRunning = true;
  pulseStartMs = millis();
  Serial.println("← 左脉冲");
}

void pulseMotorRight() {
  digitalWrite(AIN1, HIGH);
  digitalWrite(AIN2, LOW);
  analogWrite(PWMA, MOTOR_SPEED);

  motorRunning = true;
  pulseStartMs = millis();
  Serial.println("→ 右脉冲");
}

void stopMotor() {
  analogWrite(PWMA, 0);
  digitalWrite(AIN1, LOW);
  digitalWrite(AIN2, LOW);
}

void checkPulseTimeout() {
  if (motorRunning && millis() - pulseStartMs >= PULSE_MS) {
    stopMotor();                 // 结束本次脉冲
    motorRunning = false;

    delay(REST_MS);              // 稳定等待再继续主循环

    // 重新读取旋转面板 LDR，判断是否需要下一脉冲
    int rotL = analogRead(LDR_rotating_L);
    int rotR = analogRead(LDR_rotating_R);
    int diff = abs(rotL - rotR);

    if (diff <= THRESH_STOP) {
      Serial.println("✔ 差值进入停止阈值，保持停止");
    } else {
      Serial.println("↻ 差值仍大，等待下一轮判断");
    }
  }
}
