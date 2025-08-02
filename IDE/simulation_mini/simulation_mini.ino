#include <Wire.h>
#include <Adafruit_INA219.h>

// 电机控制引脚
const byte IN1 = 3, IN2 = 4, PWM = 5, STBY = 2;
const uint8_t PWM_DUTY = 150;

// 编码器
const byte ENC_A = 6, ENC_B = 7;
volatile long encCount = 0;
const float PULSE_PER_DEG = 4.5;

const int16_t EAST_LIMIT = 90;      // 最大东向角度（默认可设为 90）
const int16_t WEST_LIMIT = 90;      // 最大西向角度（默认可设为 90）
const float   ROTATE_THRESHOLD = 5.0; // 超过 5° 才转动

float mockAz = -90;  // 🌞 模拟太阳从东边升起
const float mockStep = 1.5;   // 每次移动 5°
const uint32_t STEP_INTERVAL = 1000; // 每步间隔 4 秒
uint32_t lastStep = 0;

// 电流传感器（可选）
Adafruit_INA219 motorINA(0x41);
Adafruit_INA219 logicINA(0x44);
Adafruit_INA219 solarINA(0x40);

// energy
float energyActive_mWh = 0.0;
float energyIdle_mWh = 0.0;
float energyLogic_mWh = 0.0;
float energySolar_mWh = 0.0;

//LDR
const int LDR_EAST_PIN = A1;
const int LDR_WEST_PIN = A0;

int ldrEastVal = 0;
int ldrWestVal = 0;

uint32_t lastIdleSample = 0;
uint32_t lastLogicSample = 0;
uint32_t lastSolarSample = 0;
const uint32_t SAMPLE_IDLE_MS = 500;
const uint32_t SAMPLE_OTHER_MS = 500;


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

// 包装一次读取功耗信息
float printPowerLog(long encNow, long tgtPulse, long errNow, uint8_t duty) {
  float busVoltage = motorINA.getBusVoltage_V();
  float current_mA = motorINA.getCurrent_mA();
  float power_mW = busVoltage * current_mA;

  Serial.print("enc=");
  Serial.print(encNow);
  Serial.print("  tgt=");
  Serial.print(tgtPulse);
  Serial.print("  err=");
  Serial.print(errNow);
  Serial.print("  duty=");
  Serial.print(duty);
  Serial.print("  Vm=");
  Serial.print(busVoltage, 2);
  Serial.print(" V  Im=");
  Serial.print(current_mA, 1);
  Serial.print(" mA  Pm=");
  Serial.print(power_mW, 1);
  Serial.println(" mW");

  return power_mW;  // 👈 返回功率
}

bool moveToAngle(float tgtDeg, uint32_t timeout = 8000) {
  tgtDeg = constrain(tgtDeg, -90, 90);
  long tgtPulse = lround(tgtDeg * PULSE_PER_DEG);
  bool fwd = tgtPulse > encCount;
  motorRun(fwd, PWM_DUTY);
  uint32_t t0 = millis();
  long prev = encCount;
  uint32_t lastPrint = 0;

  while (fwd ? encCount < tgtPulse : encCount > tgtPulse) {
    if (millis() - lastPrint > 50) {
      float power = printPowerLog(encCount, tgtPulse, tgtPulse - encCount, PWM_DUTY);
      energyActive_mWh += power * 0.05 / 3600.0; // 50ms间隔，转换为小时
      lastPrint = millis();
    }
    
    if (millis() - t0 > timeout || (millis() - t0 > 1500 && encCount == prev)) {
      motorBrake();
      return false;
    }
  }

  motorBrake();
  encCount = constrain(encCount, -90 * PULSE_PER_DEG, 90 * PULSE_PER_DEG);
  return true;
}

void setup() {
  Serial.begin(115200);
  delay(500);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(PWM, OUTPUT);
  pinMode(STBY, OUTPUT);
  digitalWrite(STBY, HIGH);

  pinMode(ENC_A, INPUT_PULLUP);
  pinMode(ENC_B, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ENC_A), isrEnc, RISING);

  Wire.begin();
  motorINA.begin();
  motorINA.setCalibration_32V_2A();

  logicINA.begin();
  logicINA.setCalibration_32V_2A();

  solarINA.begin();
  solarINA.setCalibration_32V_2A();

  encCount = 0;
  Serial.println("⚙️ 开始太阳轨迹模拟");
}

void loop() {
  uint32_t now = millis();
  float curAngle = getAngle();
  float tgt = constrain(mockAz, -EAST_LIMIT, WEST_LIMIT);  // 模拟太阳方位
  
  // 打印当前状态
  Serial.print("🌞 mockAz=");
  Serial.print(mockAz);
  Serial.print("° → 🎯 tgt=");
  Serial.print(tgt, 1);
  Serial.print("° → 📍 cur=");
  Serial.print(curAngle, 1);
  Serial.print("° → Δ=");
  Serial.println(fabs(tgt - curAngle), 1);

  // 读取光敏电阻模拟值（0~1023）
  ldrEastVal = analogRead(LDR_EAST_PIN);
  ldrWestVal = analogRead(LDR_WEST_PIN);

  // 打印光照强度
  Serial.print("🌅 LDR_EAST=");
  Serial.print(ldrEastVal);
  Serial.print("  🌇 LDR_WEST=");
  Serial.println(ldrWestVal);

  if (fabs(tgt - curAngle) >= ROTATE_THRESHOLD) {
    Serial.println("🔄 Moving...");
    bool ok = moveToAngle(tgt);
    if (!ok) Serial.println("⚠️ Move failed!");
  } else {
    Serial.println("✅ No move needed.");

    // 采样间隔判断
    if (now - lastIdleSample >= SAMPLE_IDLE_MS) {
      float voltage = motorINA.getBusVoltage_V();
      float current = motorINA.getCurrent_mA();
      float power_mW = voltage * current;

      // 打印
      printPowerLog(encCount, lround(tgt * PULSE_PER_DEG), 0, 0);

      power_mW = max(0.0, power_mW);  // 👈 只记录正功率
      energyIdle_mWh += power_mW * SAMPLE_IDLE_MS / 1000.0 / 3600.0;
      lastIdleSample = now;
    }
  }

  if (now - lastLogicSample >= SAMPLE_OTHER_MS) {
    float logicV = logicINA.getBusVoltage_V();
    float logicI = logicINA.getCurrent_mA();
    float logicP = logicV * logicI;
    // 累计能耗
    energyLogic_mWh += logicP * SAMPLE_OTHER_MS / 1000.0 / 3600.0;

    Serial.print("  Vmcu="); Serial.print(logicINA.getBusVoltage_V(), 2);
    Serial.print(" V  Imcu="); Serial.print(logicINA.getCurrent_mA(), 2);
    Serial.print(" mA  📟 MCU P="); Serial.print(logicP, 1); Serial.println(" mW");
    lastLogicSample = now;
  }

  if (now - lastSolarSample >= SAMPLE_OTHER_MS) {
    float solarV = solarINA.getBusVoltage_V();
    float solarI = solarINA.getCurrent_mA();
    float solarP = solarV * solarI;
    // 累计能耗
    energySolar_mWh += solarP * SAMPLE_OTHER_MS / 1000.0 / 3600.0;
    Serial.print("  Vsolar="); Serial.print(solarINA.getBusVoltage_V(), 2);
    Serial.print(" V  Isolar="); Serial.print(solarINA.getCurrent_mA(), 2);
    Serial.print(" mA  🔆 Solar P="); Serial.print(solarP, 1); Serial.println(" mW");
    lastSolarSample = now;
  }

  // 模拟太阳移动：每隔一段时间推进 mockAz
  if (now - lastStep > STEP_INTERVAL) {
    Serial.print("⚡ 运行能耗: ");
    Serial.print(energyActive_mWh, 3);
    Serial.print(" mWh，静止能耗: ");
    Serial.print(energyIdle_mWh, 3);
    Serial.print(" mWh，总能耗: ");
    Serial.print(energyActive_mWh + energyIdle_mWh, 3);
    Serial.print(" mWh，逻辑电路: "); Serial.print(energyLogic_mWh, 3);
    Serial.print(" mWh，太阳吸收: "); Serial.print(energySolar_mWh, 3);
    Serial.println(" mWh");

    mockAz += mockStep;
    if (mockAz > 90) {
      Serial.println("🌙 模拟日落，重新开始！");
      mockAz = -90;
    }
    lastStep = now;
  }
}
