// TB6612 控制引脚
const int PWMA = 5;
const int AIN1 = 7;
const int AIN2 = 4;
const int STBY = 6;

// 前进设置（慢速）
const int PWM_FORWARD     = 120;     // 慢速强度
const int STEP_DELAY_MS_F = 150;     // 转动时间
const int DWELL_MS_F      = 1516;    // 等待时间
const int NUM_STEPS_F     = 12;      // 正向步数

// 返回设置（快速）
const int PWM_BACKWARD     = 120;   // 快速强度
const int STEP_DELAY_MS_B  = 1800;   // 快速转动时间
const int DWELL_MS_B       = 0;   // 快速等待时间
const int NUM_STEPS_B      = 1;    // 快速反向步数（与前向一致）

void setup() {
  pinMode(PWMA, OUTPUT);
  pinMode(AIN1, OUTPUT);
  pinMode(AIN2, OUTPUT);
  pinMode(STBY, OUTPUT);

  Serial.begin(9600);
  digitalWrite(STBY, HIGH);  // 启动 TB6612 模块

  // === 正向慢速旋转 0° → 180° ===
  Serial.println("开始直流电机慢速旋转到 180°...");

  unsigned long startTime = millis();  // 记录开始时间

  for (int i = 1; i <= NUM_STEPS_F; i++) {
    // 正转
    digitalWrite(AIN1, HIGH);
    digitalWrite(AIN2, LOW);
    analogWrite(PWMA, PWM_FORWARD);

    delay(STEP_DELAY_MS_F);

    analogWrite(PWMA, 0);  // 停止
    delay(DWELL_MS_F);

    Serial.print("已完成正向步骤 ");
    Serial.print(i);
    Serial.print(" / ");
    Serial.println(NUM_STEPS_F);
  }

  unsigned long forwardEndTime = millis();  // 完成正转
  float forwardTime = (forwardEndTime - startTime) / 1000.0;
  Serial.print("完成 180° 正向耗时：");
  Serial.print(forwardTime, 2);
  Serial.println(" 秒");

  delay(3000);  // 中间停顿

  // === 快速反向回到 0° ===
  Serial.println("开始直流电机快速返回 0°...");

  unsigned long returnStartTime = millis();

  for (int i = 1; i <= NUM_STEPS_B; i++) {
    // 反转
    digitalWrite(AIN1, LOW);
    digitalWrite(AIN2, HIGH);
    analogWrite(PWMA, PWM_BACKWARD);

    delay(STEP_DELAY_MS_B);

    analogWrite(PWMA, 0);
    delay(DWELL_MS_B);

    Serial.print("已完成反向步骤 ");
    Serial.print(i);
    Serial.print(" / ");
    Serial.println(NUM_STEPS_B);
  }

  unsigned long endTime = millis();
  float totalTime = (endTime - startTime) / 1000.0;
  float returnTime = (endTime - returnStartTime) / 1000.0;

  Serial.print("快速返回耗时：");
  Serial.print(returnTime, 2);
  Serial.println(" 秒");

  Serial.print("总运行时间（来回）：");
  Serial.print(totalTime, 2);
  Serial.println(" 秒");
}

void loop() {
  // 无操作
}
