#include <Wire.h>
#include <MKRWAN.h>
#include <ArduinoLowPower.h>
#include <Adafruit_INA219.h>

// ==== LoRa 配置 ====
LoRaModem modem;
const char *appEui = "0000000000000000";
const char *appKey = "88F69C465D40376DEA90FE92F3764A36";  // 替换成你的真实 AppKey
uint32_t lastJoinAttempt = 0;

// ==== LDR 配置 ====
const int LDR_EAST_PIN = A2;
const int LDR_WEST_PIN = A1;
const int LDR_THRESH = 800;  // 光照阈值（越小越亮）

// ==== INA219 ====
Adafruit_INA219 solarINA(0x40); // 太阳能板侧
Adafruit_INA219 logicINA(0x44); // 逻辑电路侧

// ==== 能耗统计 ====
float solar_energy_mWh = 0, logic_energy_mWh = 0;
float solar_sumV = 0, solar_sumI_mA = 0;
float logic_sumV = 0, logic_sumI_mA = 0;
uint32_t solar_samples = 0, logic_samples = 0;
uint32_t lastSampleMs = 0;
const uint16_t SAMPLE_INTERVAL_MS = 200;

// ==== 唤醒时间 ====
uint32_t wakeStartMs = 0;
uint16_t wakeDuration_s = 0;

bool isDaytime() {
  int eVal = analogRead(LDR_EAST_PIN);
  int wVal = analogRead(LDR_WEST_PIN);
  return (eVal < LDR_THRESH) || (wVal < LDR_THRESH);
}

void beforeSleep(uint32_t sleepMs) {
  wakeDuration_s = (millis() - wakeStartMs) / 1000;
  modem.sleep(sleepMs / 1000);
  LowPower.deepSleep(sleepMs);
}

void afterWake() {
  Wire.begin();
  wakeStartMs = millis();

  // 重置统计
  solar_energy_mWh = logic_energy_mWh = 0;
  solar_sumV = solar_sumI_mA = 0; solar_samples = 0;
  logic_sumV = logic_sumI_mA = 0; logic_samples = 0;
  lastSampleMs = 0;
}

void sampleSensors(uint32_t nowMs) {
  if (lastSampleMs != 0 && nowMs - lastSampleMs < SAMPLE_INTERVAL_MS) return;
  lastSampleMs = nowMs;

  // Solar
  float vS = solarINA.getBusVoltage_V();
  float iS = solarINA.getCurrent_mA();
  if (vS >= 0 && iS >= 0) {
    solar_sumV += vS;
    solar_sumI_mA += iS;
    solar_energy_mWh += (vS * iS) * (SAMPLE_INTERVAL_MS / 3600000.0f);
    solar_samples++;
  }

  // Logic
  float vL = logicINA.getBusVoltage_V();
  float iL = logicINA.getCurrent_mA();
  if (vL >= 0 && iL >= 0) {
    logic_sumV += vL;
    logic_sumI_mA += iL;
    logic_energy_mWh += (vL * iL) * (SAMPLE_INTERVAL_MS / 3600000.0f);
    logic_samples++;
  }
}

void setup() {
  Serial.begin(115200); delay(300);
  pinMode(LDR_EAST_PIN, INPUT);
  pinMode(LDR_WEST_PIN, INPUT);
  Wire.begin();
  solarINA.begin();
  logicINA.begin();

  if (!modem.begin(EU868)) {
    Serial.println("❌ LoRa init failed");
    while (1);
  }
  modem.setADR(true);
  modem.setPort(1);

  if (modem.joinOTAA(appEui, appKey)) {
    Serial.println("✅ LoRa joined");
  } else {
    Serial.println("⚠️ LoRa join failed (will retry later)");
  }
  lastJoinAttempt = millis();

  afterWake();  // 初始化
}

void loop() {
  bool day = isDaytime();
  int eVal = analogRead(LDR_EAST_PIN);
  int wVal = analogRead(LDR_WEST_PIN);

  Serial.print("🌞 Daylight: "); Serial.println(day ? "YES" : "NO");

  // ==== 采样一段时间 ====
  uint32_t holdStart = millis();
  while (millis() - holdStart < 3000) { // 可调整采样窗口时长
    sampleSensors(millis());
    delay(10);
  }

  // ==== LoRa 上传 ====
  if (!modem.connected()) {
    if (millis() - lastJoinAttempt > 300000UL) {
      Serial.println("🔁 Rejoin LoRa...");
      if (modem.joinOTAA(appEui, appKey)) {
        Serial.println("✅ Rejoined");
      } else {
        Serial.println("❌ Rejoin failed");
      }
      lastJoinAttempt = millis();
    }
  } else {
    float v_solar = solar_samples ? (solar_sumV / solar_samples) : 0;
    float i_solar = solar_samples ? (solar_sumI_mA / solar_samples) : 0;
    float v_logic = logic_samples ? (logic_sumV / logic_samples) : 0;
    float i_logic = logic_samples ? (logic_sumI_mA / logic_samples) : 0;

    // === 缩放打包 ===
    uint8_t payload[20]; int i = 0;
    auto put16 = [&](uint16_t v) {
      payload[i++] = v & 0xFF;
      payload[i++] = v >> 8;
    };

    put16((uint16_t)eVal);
    put16((uint16_t)wVal);
    put16((uint16_t)wakeDuration_s);  // 单位：秒
    put16((uint16_t)(solar_energy_mWh * 100));
    put16((uint16_t)(v_solar * 100));
    put16((uint16_t)(i_solar * 100));
    put16((uint16_t)(logic_energy_mWh * 100));
    put16((uint16_t)(v_logic * 100));
    put16((uint16_t)(i_logic * 100));

    modem.beginPacket();
    modem.write(payload, i);
    int res = modem.endPacket(false);
    Serial.println(res == 0 ? "📡 LoRa OK" : "📡 LoRa FAIL");
  }

  // ==== 睡眠 ====
  uint32_t sleepMs = day ? 10 * 60 * 1000UL : 30 * 60 * 1000UL;
  beforeSleep(sleepMs);
  afterWake();
}
