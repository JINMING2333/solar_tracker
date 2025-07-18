#include <Adafruit_INA219.h>
Adafruit_INA219 ina;

#include <MKRWAN.h>
LoRaModem modem;

/* —— LoRaWAN keys —— */
const char *appEui = "0000000000000000";
const char *appKey = "817A05EB77508E8C2FC8410C040CB706";

const uint8_t  SEND_EVERY_MIN = 1;     // 每 N 分钟发一次，后期可改 15
const uint16_t SAMPLE_INTERVAL_MS = 1000;   // 1 s
const uint16_t BLOCK_MS           = 60000;  // 1 min

uint16_t  sampleCnt = 0; // 采样计数
float    sumV = 0, sumI_mA = 0, sumP_mW = 0;  // 功率和（用于计算平均值）；
float    minP_mW =  1e9, maxP_mW = -1e9;      // 功率最小值和最大值（初始化为极端值）

uint32_t  tNextSample = 0;   // 下一次采样时间戳
uint16_t  minuteCounter = 0;           // 日内分钟 idx

/* --------------------------- */
void sendLoRa(const uint8_t *payload, uint8_t len) {
  modem.beginPacket();
  modem.write(payload, len);
  modem.endPacket(true);               // async = true → 立刻返回
}

void setup() {
  Serial.begin(115200);
  while (!Serial);

  ina.begin();
  ina.setCalibration_32V_2A();       //设置传感器的校准参数，使其适用于测量最高 32V 电压和最大 2A 电流的场景。

  // connect to lora gateway
  if (!modem.begin(EU868)) {
    Serial.println("Failed to start module");while (1);
  }
  modem.setADR(true);                  // 自适应速率
  modem.dataRate(5);                      // SF7 (EU868)  — 可调
 
  Serial.print("Your device EUI is: ");
  Serial.println(modem.deviceEUI());

  int connected = modem.joinOTAA(appEui, appKey);
  if (!connected) {
    Serial.println("Something went wrong");while (1);
  }
  Serial.println("Joined TTN");
  
  tNextSample = millis();
}

void loop() {
  if (millis() < tNextSample) return;      // 等待到采样点
  tNextSample += SAMPLE_INTERVAL_MS;

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

    /* 仅每 SEND_EVERY_MIN 分钟上传一次 */
    if (minuteCounter % SEND_EVERY_MIN == 0) {

      uint16_t PmW = (uint16_t)(Pavg_mW * 10.0 + 0.5);   // 0.01 W 单位
      uint16_t V_V = (uint16_t)(Vavg* 100.0 + 0.5);
      uint16_t ImA = (uint16_t)(Iavg_mA* 10.0 + 0.5);

      uint8_t buf[7];
      buf[0]=PmW>>8; buf[1]=PmW;
      buf[2]=V_V>>8; buf[3]=V_V;
      buf[4]=ImA>>8; buf[5]=ImA;
      buf[6]=minuteCounter & 0xFF;

      sendLoRa(buf, 7);
      Serial.println("LoRa uplink sent");
    }

    // 重置
    tBlockStart += BLOCK_MS;   // 防止漂移
    sampleCnt   = 0;
    sumV = sumI_mA = sumP_mW = 0;
    minP_mW = 1e9;  maxP_mW = -1e9;
    ++minuteCounter; 
  }
}