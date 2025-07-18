/* ============ 1. 头文件 & 常量 ============ */
#include <Adafruit_NeoPixel.h>

const int LEDS_ROT = 16;   // 单轴灯条 LED 数
const int LEDS_FIX = 16;   // 固定灯条 LED 数

// 建议改为非 0/1/13 等调试口
const int DATA_PIN_ROT = 13; // 旋转面板灯条 (你已接 D13)
const int DATA_PIN_FIX = 2;  // 固定面板灯条 

Adafruit_NeoPixel stripRot(LEDS_ROT, DATA_PIN_ROT, NEO_GRB + NEO_KHZ800);
Adafruit_NeoPixel stripFix(LEDS_FIX, DATA_PIN_FIX, NEO_GRB + NEO_KHZ800);

/* ============ 2. LDR 与电机原引脚保持不变 ============ */
const int LDR_rotating_L = A0;
const int LDR_rotating_R = A1;
const int LDR_fixed_L    = A2;
const int LDR_fixed_R    = A3;

const int PWMA = 5;
const int AIN1 = 7;
const int AIN2 = 4;
const int STBY = 6;

const int lampMosfetPin = 3;

/* ============ 3. 追光阈值 & 脉冲参数 ============ */
const int LIGHT_THRESHOLD = 800;
const int THRESH_START    = 40;
const int THRESH_STOP     = 20;
const int MOTOR_SPEED     = 100;
const int PULSE_MS        = 120;
const int REST_MS         = 200;
bool motorRunning         = false;
unsigned long pulseStartMs = 0;

/* ============ 4. LED 渐亮计时参数 ============ */
const int LIGHT_THRESHOLD_ROTLED     = 770;  // 旋转灯条
const int LIGHT_THRESHOLD_FIXLED = 780;   // 固定面板灯条阈值
const unsigned long MAX_TIME = 8000UL;  // 8 秒完全点亮
bool  rotActive = false, fixActive = false;
unsigned long rotStartMs = 0, fixStartMs = 0;

/* ============ 5. 初始化 ============ */
void setup() {
  Serial.begin(9600);
  pinMode(lampMosfetPin, OUTPUT);
  digitalWrite(lampMosfetPin, HIGH);

  pinMode(PWMA, OUTPUT); pinMode(AIN1, OUTPUT);
  pinMode(AIN2, OUTPUT); pinMode(STBY, OUTPUT);
  digitalWrite(STBY, HIGH);

  stripRot.begin();
  stripFix.begin();
  stripRot.show();  // 全灭
  stripFix.show();

  Serial.println("系统初始化完成");
}

/* ============ 6. 主循环 ============ */
void loop() {
  /* ---- 6.1 读取 LDR ---- */
  int rotL = analogRead(LDR_rotating_L);
  int rotR = analogRead(LDR_rotating_R);
  int fixL = analogRead(LDR_fixed_L);
  int fixR = analogRead(LDR_fixed_R);

   /* ===== 新增：串口显示四个 LDR 值 ===== */
  Serial.print("ROT L=");  Serial.print(rotL);
  Serial.print(" R=");     Serial.print(rotR);
  Serial.print(" | FIX L=");Serial.print(fixL);
  Serial.print(" R=");     Serial.println(fixR);

  /* ---- 6.2 灯条逻辑 ---- */
  handleStrip(rotL, rotR, stripRot, rotActive, rotStartMs, "ROT", LIGHT_THRESHOLD_ROTLED);
  handleStrip(fixL, fixR, stripFix, fixActive, fixStartMs, "FIX", LIGHT_THRESHOLD_FIXLED);

  /* ---- 6.3 直流电机追光 (与你原代码一致) ---- */
  if (!motorRunning) {
    int diff = abs(rotL - rotR);
    if ((rotL > LIGHT_THRESHOLD || rotR > LIGHT_THRESHOLD) && diff > THRESH_START) {
      if (rotL > rotR) pulseMotorLeft(); else pulseMotorRight();
    }
  }
  checkPulseTimeout();
}

/* ============ 7. 灯条控制函数 ============ */
void handleStrip(int ldrA, int ldrB,
                 Adafruit_NeoPixel &strip,
                 bool &activeFlag,
                 unsigned long &startMs,
                 const char* tag,
                 int thr) {

  if (ldrA >= thr && ldrB >= thr) {
    if (!activeFlag) {                   // 首次达到阈值
      activeFlag = true;
      startMs = millis();
      // Serial.print(tag); Serial.println(": 开始计时");
    }
    // 已经激活，计算亮灯颗数
    unsigned long elapsed = millis() - startMs;
    int ledsToLight = map(elapsed, 0, MAX_TIME, 0, strip.numPixels());
    ledsToLight = constrain(ledsToLight, 0, strip.numPixels());

    for (int i = 0; i < strip.numPixels(); i++) {
      if (i < ledsToLight) {
        strip.setPixelColor(i, strip.Color(255, 180, 0)); // 琥珀色
      } else {
        strip.setPixelColor(i, 0);                       // 熄灭
      }
    }
    strip.show();
  } else { // 任何一只 LDR 跌破阈值 → 重置
    if (activeFlag) {
      activeFlag = false;
      // Serial.print(tag); Serial.println(": 低于阈值，熄灭");
      for (int i = 0; i < strip.numPixels(); i++) strip.setPixelColor(i, 0);
      strip.show();
    }
  }
}

/* ============ 8. 直流电机脉冲函数保持不变 ============ */
void pulseMotorLeft()  { digitalWrite(AIN1, LOW);  digitalWrite(AIN2, HIGH); analogWrite(PWMA, MOTOR_SPEED); motorRunning = true;  pulseStartMs = millis(); Serial.println("← 左脉冲"); }
void pulseMotorRight() { digitalWrite(AIN1, HIGH); digitalWrite(AIN2, LOW);  analogWrite(PWMA, MOTOR_SPEED); motorRunning = true;  pulseStartMs = millis(); Serial.println("→ 右脉冲"); }
void stopMotor()       { analogWrite(PWMA, 0); digitalWrite(AIN1, LOW); digitalWrite(AIN2, LOW); }
void checkPulseTimeout(){
  if (motorRunning && millis() - pulseStartMs >= PULSE_MS) {
    stopMotor(); motorRunning = false; delay(REST_MS);
    int d = abs(analogRead(LDR_rotating_L) - analogRead(LDR_rotating_R));
    if (d <= THRESH_STOP) Serial.println("✔ 平衡停止");
  }
}
