#include <ArduinoLowPower.h>
#include <Wire.h>
#include <Adafruit_INA219.h>
#include <MKRWAN.h>

/*** INA219 ***/
Adafruit_INA219 inaPV(0x40);     // 太阳能板侧
Adafruit_INA219 inaSYS(0x44);    // 系统侧

/*** LDR 引脚 ***/
const int LDR_E_PIN = A2;        // 东侧 LDR → A1（上拉到3.3V，分压到GND）
const int LDR_W_PIN = A1;        // 西侧 LDR → A2（上拉到3.3V，分压到GND）

/*** LoRa ***/
LoRaModem modem;
String appEui = "0000000000000000";
String appKey = "88F69C465D40376DEA90FE92F3764A36";   // ← 换成你的

/*** 周期 ***/
const uint32_t SLEEP_MS = 1 * 60000;       // 每 60 s 一次
const uint8_t  N_SAMPLES = 5;
const uint16_t SAMPLE_SPACING_MS = 100;

/*** 小工具 ***/
inline void putU16(uint8_t *b, int i, uint16_t v){ b[i] = v >> 8; b[i+1] = v; }
inline void putS16(uint8_t *b, int i, int16_t  v){ b[i] = v >> 8; b[i+1] = v; }

static void reinitINA() {
  // 唤醒后把 I2C/INA219 重新初始化更稳妥
  Wire.end();
  delay(2);
  Wire.begin();
  inaPV.begin();  inaPV.setCalibration_32V_2A();
  inaSYS.begin(); inaSYS.setCalibration_32V_2A();
}

// —— 一次测量并上报（附带“活跃时长”）——
void measure_and_send() {
  uint32_t t_begin = millis();   // ★ 从唤醒后开始计时

  // 采样做小平均（和你现在一样）
  double sumVpv=0, sumIpv=0, sumPpv=0;
  double sumVsys=0, sumIsys=0, sumPsys=0;

  for (uint8_t k=0; k<N_SAMPLES; ++k) {
    float v1 = inaPV.getBusVoltage_V();
    float i1 = inaPV.getCurrent_mA();
    sumVpv += v1; sumIpv += i1; sumPpv += v1 * i1;

    float v2 = inaSYS.getBusVoltage_V();
    float i2 = inaSYS.getCurrent_mA();
    sumVsys += v2; sumIsys += i2; sumPsys += v2 * i2;

    delay(SAMPLE_SPACING_MS);
  }

  float Vpv  = sumVpv  / N_SAMPLES;
  float Ipv  = sumIpv  / N_SAMPLES;
  float Ppv  = sumPpv  / N_SAMPLES;
  float Vsys = sumVsys / N_SAMPLES;
  float Isys = sumIsys / N_SAMPLES;
  float Psys = sumPsys / N_SAMPLES;

  // 读取 LDR
  uint16_t ldrE = analogRead(LDR_E_PIN);
  uint16_t ldrW = analogRead(LDR_W_PIN);

  // 计算活跃时长（毫秒 → 用 0.1s 精度发 16bit 更省字节）
  uint32_t active_ms = millis() - t_begin;
  uint16_t active_cs = (uint16_t)min<uint32_t>(65535, (active_ms + 5) / 10); // 0.1s

  // 缩放并打包（与原先一致）
  int16_t  Ppv10   = (int16_t)  lround(Ppv  * 10.0);
  int16_t  Psys10  = (int16_t)  lround(Psys * 10.0);
  uint16_t Vpv100  = (uint16_t) lround(Vpv  * 100.0);
  uint16_t Vsys100 = (uint16_t) lround(Vsys * 100.0);
  int16_t  Ipv100  = (int16_t)  lround(Ipv  * 100.0);
  int16_t  Isys100 = (int16_t)  lround(Isys * 100.0);

  // 原 16 字节 + 2 字节 active_cs = 18 字节
  uint8_t buf[18];
  putS16(buf,  0, Ppv10);
  putS16(buf,  2, Psys10);
  putU16(buf,  4, Vpv100);
  putU16(buf,  6, Vsys100);
  putS16(buf,  8, Ipv100);
  putS16(buf, 10, Isys100);
  putU16(buf, 12, ldrE);
  putU16(buf, 14, ldrW);
  putU16(buf, 16, active_cs);  // ★ 新增：本次活跃 0.1s

  modem.beginPacket();
  modem.write(buf, sizeof(buf));
  int err = modem.endPacket(false);    // 阻塞到发送完成
  // 简单诊断输出（可选）
  if (err == 0) {
    // Serial.println("TX ok");
  } else {
    // Serial.print("TX err="); Serial.println(err);
  }
  delay(1500);                          // 等 RX1/RX2，避免栈乱掉
}


/*** setup ***/
void setup() {
  pinMode(LED_BUILTIN, OUTPUT);

  // 串口仅首启调试用
  Serial.begin(115200);
  uint32_t t0 = millis();
  while (!Serial && millis() - t0 < 2000) {}

  // ADC 分辨率（SAMD21 缺省 10bit；改成 12bit 更顺滑）
  // analogReadResolution(12);

  Wire.begin();
  inaPV.begin();  inaPV.setCalibration_32V_2A();
  inaSYS.begin(); inaSYS.setCalibration_32V_2A();

  pinMode(LDR_E_PIN, INPUT);
  pinMode(LDR_W_PIN, INPUT);

  // LoRa 初始化 / 入网
  if (!modem.begin(EU868)) {
    while (1) { digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN)); delay(200); }
  }
  modem.setADR(true);
  modem.dataRate(5);        // EU868: SF7
  modem.setPort(1);

  if (!modem.joinOTAA(appEui, appKey, 60000)) {
    for (int i=0;i<10;i++){ digitalWrite(LED_BUILTIN, HIGH); delay(100); digitalWrite(LED_BUILTIN, LOW); delay(100); }
    while (1);
  }

  // 先测一次并发出去
  measure_and_send();
}

/*** loop ***/
void loop() {
  // modem.wakeUp();           // 唤醒 LoRa 模块
  reinitINA();              // 你已有的 I2C/INA 复位
  // 心跳：每次唤醒闪一下
  digitalWrite(LED_BUILTIN, HIGH); delay(120); digitalWrite(LED_BUILTIN, LOW);


  // 测量 + 上报
  measure_and_send();

  // ★★★ 仅此处新增：进睡前把两块 INA219 置为省电 ★★★
  inaPV.powerSave(true);
  inaSYS.powerSave(true);

  // …… measure_and_send(); 之后
  modem.sleep();            // 让射频模块也低功耗（很关键）
  inaPV.powerSave(true);
  inaSYS.powerSave(true);
  LowPower.deepSleep(SLEEP_MS);
}
