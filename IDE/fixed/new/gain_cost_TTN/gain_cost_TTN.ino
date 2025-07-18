#include <Adafruit_INA219.h>
Adafruit_INA219 inaPV(0x40);   // 太阳能板侧
Adafruit_INA219 inaSYS(0x41);  // 系统耗电侧

#include <MKRWAN.h>
LoRaModem modem;

/* —— LoRaWAN keys —— */
const char *appEui = "0000000000000000";
const char *appKey = "817A05EB77508E8C2FC8410C040CB706";

const uint8_t  SEND_EVERY_MIN = 5;     // 每 N 分钟发一次，后期可改 15
const uint16_t SAMPLE_INTERVAL_MS = 1000;   // 1 s
const uint16_t BLOCK_MS           = 60000;  // 1 min

uint16_t  nPV = 0; // 采样计数
double   sumVPV = 0, sumIPV = 0, sumPPV = 0;  // 功率和（用于计算平均值）；

uint16_t nSYS=0;
double   sumVS = 0, sumIS = 0, sumPS  = 0; 

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

  unsigned long t0 = millis();
  while (!Serial && millis() - t0 < 4000) {}   // 最多等 4 秒

  inaPV.begin();  inaPV.setCalibration_32V_2A();
  inaSYS.begin(); inaSYS.setCalibration_32V_2A();       //设置传感器的校准参数，使其适用于测量最高 32V 电压和最大 2A 电流的场景。

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

  // ---- 采样+累加 ----
  /* PV */
  double v1=inaPV.getBusVoltage_V();
  double i1=inaPV.getCurrent_mA();
  double p1=v1*i1;
  sumVPV+=v1; sumIPV+=i1; sumPPV+=p1; ++nPV;

  /* SYS */
  double v2=inaSYS.getBusVoltage_V();
  double i2=inaSYS.getCurrent_mA();
  double p2=v2*i2;
  sumVS+=v2; sumIS+=i2; sumPS+=p2; ++nSYS;

  // ---- 到 1 分钟了吗？ ----
  static uint32_t tBlockStart = millis();        // 统计周期的起始时间
  if (millis() - tBlockStart >= BLOCK_MS) {
    double Vpv = sumVPV/nPV;
    double Ipv = sumIPV/nPV;
    double Ppv = sumPPV/nPV;

    double Vsys= sumVS/nSYS;
    double Isys= sumIS/nSYS;
    double Psys= sumPS/nSYS;       

    // 时间戳（秒）：用累积的块起始时间更准确
    uint32_t ts = tBlockStart / 1000;
    Serial.print(ts);        Serial.print(',');
    Serial.print(Vpv,   3); Serial.print(',');  // Volt
    Serial.print(Ipv,1); Serial.print(',');  // mA  (1 位小数就够)
    Serial.print(Ppv,1); Serial.print(',');  // mW
    Serial.print(Vsys,   3); Serial.print(',');  // Volt
    Serial.print(Isys,1); Serial.print(',');  // mA  (1 位小数就够)
    Serial.println(Psys,1);  // mW

    /* 仅每 SEND_EVERY_MIN 分钟上传一次 */
    if (minuteCounter % SEND_EVERY_MIN == 0) {

      int16_t Ipv100  = (int16_t)(Ipv  *100.0);
      int16_t Ppv100  = (int16_t)(Ppv  *100.0);   // 可正可负
      int16_t Isys100 = (int16_t)(Isys *100.0);
      int16_t Psys100 = (int16_t)(Psys *100.0);

      uint16_t Vpv100  = (uint16_t)(Vpv  *100.0 + 0.5);
      uint16_t Vsys100 = (uint16_t)(Vsys *100.0 + 0.5);


      uint8_t buf[14];
      auto put = [&](int i, int16_t v){ buf[i]=v>>8; buf[i+1]=v; };
      put(0, Ppv100);  put(2, Psys100);
      buf[4]=Vpv100>>8;  buf[5]=Vpv100;
      buf[6]=Vsys100>>8; buf[7]=Vsys100;
      put(8, Ipv100);  put(10, Isys100);
      buf[12]= minuteCounter >> 8;
      buf[13]= minuteCounter & 0xFF;

      sendLoRa(buf, 14);
      Serial.println("LoRa uplink sent");
    }

    // 重置
    tBlockStart += BLOCK_MS;   // 防止漂移
    nPV=nSYS=0;
    sumVPV=sumIPV=sumPPV=0;
    sumVS=sumIS=sumPS=0;
    minuteCounter=(minuteCounter+1)%1440;
  }
}