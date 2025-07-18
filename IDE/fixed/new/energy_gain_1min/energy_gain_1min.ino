#include <Adafruit_INA219.h>
Adafruit_INA219 ina;

const uint16_t SAMPLE_INTERVAL_MS = 1000;   // 1 s
const uint16_t BLOCK_MS           = 60000;  // 1 min

uint16_t  sampleCnt = 0; // 采样计数
float    sumV = 0, sumI_mA = 0, sumP_mW = 0;  // 功率和（用于计算平均值）；
float    minP_mW =  1e9, maxP_mW = -1e9;      // 功率最小值和最大值（初始化为极端值）

uint32_t  tNext = 0;   // 下一次采样时间戳

void setup() {
  Serial.begin(115200);
  ina.begin();
  ina.setCalibration_32V_2A();       //设置传感器的校准参数，使其适用于测量最高 32V 电压和最大 2A 电流的场景。

  // 等待 Python 发来 START
  while (!Serial.available()) {}
  if (Serial.readStringUntil('\n') != "START") while (1);
  
  tNext = millis();
}

void loop() {
  if (millis() < tNext) return;      // 等待到采样点
  tNext += SAMPLE_INTERVAL_MS;

  // ---- 采样 ----
  float v = ina.getBusVoltage_V();   // V
  float i = ina.getCurrent_mA();     // mA
  float p = v * i;                   // mW

  // ---- 累加 ----
  sumV      += v;
  sumI_mA   += i;
  sumP_mW   += p;
  minP_mW    = min(minP_mW, p);
  maxP_mW    = max(maxP_mW, p);
  ++sampleCnt;

  // ---- 到 1 分钟了吗？ ----
  static uint32_t tBlockStart = millis();        // 统计周期的起始时间
  if (millis() - tBlockStart >= BLOCK_MS) {
    float Vavg   = sumV      / sampleCnt;        
    float Iavg_mA= sumI_mA   / sampleCnt;        
    float Pavg_mW= sumP_mW   / sampleCnt;        

    // 时间戳（秒）：用累积的块起始时间更准确
    uint32_t ts = tBlockStart / 1000;
    Serial.print(ts);        Serial.print(',');
    Serial.print(Vavg,   3); Serial.print(',');  // Volt
    Serial.print(Iavg_mA,1); Serial.print(',');  // mA  (1 位小数就够)
    Serial.print(Pavg_mW,1); Serial.print(',');  // mW
    Serial.print(minP_mW,1); Serial.print(',');
    Serial.println(maxP_mW,1);

    // 重置
    tBlockStart = tBlockStart + BLOCK_MS;   // 防止漂移
    sampleCnt   = 0;
    sumV = sumI_mA = sumP_mW = 0;
    minP_mW = 1e9;  maxP_mW = -1e9;
  }
}