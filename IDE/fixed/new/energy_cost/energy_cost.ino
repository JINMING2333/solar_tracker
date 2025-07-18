#include <Wire.h>
#include <Adafruit_INA219.h>

/* ------------ 2 个 INA 实例 ------------ */
Adafruit_INA219 inaPV(0x40);   // 太阳能板侧
Adafruit_INA219 inaSYS(0x41);  // 系统耗电侧

/* ------------ 统计参数 ------------ */
const uint16_t SAMPLE_MS = 1000;   // 1 Hz
const uint16_t BLOCK_MS  = 60000;  // 1 min

/* 累加器 for PV */
uint32_t tNext = 0;
uint16_t nPV = 0;
double   sumVPV = 0, sumIPV_mA = 0, sumPPV_mW = 0;

/* 累加器 for SYS */
uint16_t nSYS = 0;
double   sumVS = 0, sumIS_mA = 0, sumPS_mW = 0;

void setup() {
  Serial.begin(115200);
  Wire.begin();

  inaPV.begin();
  inaSYS.begin();

  inaPV.setCalibration_32V_2A();   // 可按需要调整量程
  inaSYS.setCalibration_32V_2A();

  /* 等待 Python START */
  // while (!Serial.available()) {}
  // if (Serial.readStringUntil('\n') != "START") while (1);

  tNext = millis();
}

void loop() {

  /* -------- 每秒采样两颗 INA -------- */
  if (millis() >= tNext) {
    tNext += SAMPLE_MS;

    /* PV */
    float v1 = inaPV.getBusVoltage_V();      // V
    float i1 = inaPV.getCurrent_mA();        // mA
    float p1 = v1 * i1;                      // mW
    sumVPV      += v1;
    sumIPV_mA   += i1;
    sumPPV_mW   += p1;
    ++nPV;

    /* SYS */
    float v2 = inaSYS.getBusVoltage_V();
    float i2 = inaSYS.getCurrent_mA();
    float p2 = v2 * i2;
    sumVS      += v2;
    sumIS_mA   += i2;
    sumPS_mW   += p2;
    ++nSYS;
  }

  /* -------- 每分钟输出一次 -------- */
  static uint32_t tBlockStart = millis();
  if (millis() - tBlockStart >= BLOCK_MS) {

    float VPV_avg  = sumVPV    / nPV;
    float IPV_avg  = sumIPV_mA / nPV;
    float PPV_avg  = sumPPV_mW / nPV;

    float VSYS_avg = sumVS     / nSYS;
    float ISYS_avg = sumIS_mA  / nSYS;
    float PSYS_avg = sumPS_mW  / nSYS;

    uint32_t ts = tBlockStart / 1000;   // 秒级时间戳

    /* CSV 格式：ts,PV_V,PV_I_mA,PV_P_mW,SYS_V,SYS_I_mA,SYS_P_mW */
    Serial.print(ts);            Serial.print(',');
    Serial.print(VPV_avg, 3);    Serial.print(',');
    Serial.print(IPV_avg, 1);    Serial.print(',');
    Serial.print(PPV_avg, 1);    Serial.print(',');
    Serial.print(VSYS_avg, 3);   Serial.print(',');
    Serial.print(ISYS_avg, 1);   Serial.print(',');
    Serial.println(PSYS_avg, 1);

    /* ------ 重置计数器 ------ */
    tBlockStart += BLOCK_MS;
    nPV = nSYS = 0;
    sumVPV = sumIPV_mA = sumPPV_mW = 0;
    sumVS  = sumIS_mA  = sumPS_mW  = 0;
  }
}
