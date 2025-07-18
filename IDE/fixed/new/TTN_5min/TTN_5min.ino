/******************************************************************
 *  MKR WAN 1310  ·  Two INA219  ·  5-min Energy Report (mWh×1000)
 ******************************************************************/
#include <Arduino.h>
#include <math.h>
#include <Adafruit_INA219.h>
#include <MKRWAN.h>

/* ------------ INA219 ------------ */
Adafruit_INA219 inaPV (0x40);
Adafruit_INA219 inaSYS(0x41);

/* ------------ LoRa Keys ---------- */
String appEuiStr = "0000000000000000";
String appKeyStr = "817A05EB77508E8C2FC8410C040CB706";

/* ------------ Timing ------------- */
const uint16_t SAMPLE_MS       = 1000;          // 1 s
const uint8_t  UP_INT_MIN      = 5;             // upload every 5 min
const uint32_t WINDOW_MS       = UP_INT_MIN * 60UL * 1000UL;

/* ------------ Accumulators ------- */
uint32_t tNextSample = 0, lastSample = 0, winStart = 0;

int64_t  PV_mWms  = 0;      // mW·ms
int64_t  SYS_mWms = 0;

double   sumVPV = 0, sumIPV = 0, sumVS = 0, sumIS = 0;
uint16_t nSamples = 0;

/* ------------ Helpers ------------ */
LoRaModem modem;
inline void putU16(uint8_t *b,int i,uint16_t v){ b[i]=v>>8; b[i+1]=v; }
inline void putS16(uint8_t *b,int i,int16_t v){ b[i]=v>>8; b[i+1]=v; }

void sendLoRa(const uint8_t *p, uint8_t len){
  modem.beginPacket(); modem.write(p, len); modem.endPacket(true);
}

void setup(){
  Serial.begin(115200);
  while(!Serial && millis() < 4000){}

  inaPV.begin();  inaPV.setCalibration_32V_2A();
  inaSYS.begin(); inaSYS.setCalibration_32V_2A();

  /* --- LoRa --- */
  if(!modem.begin(EU868)){ Serial.println("Modem init fail"); while(1);}
  modem.setADR(true); modem.dataRate(5);     // SF7
  if(!modem.joinOTAA(appEuiStr, appKeyStr, 60000)){
    Serial.println("Join fail"); while(1);
  }
  Serial.println("Joined TTN ✔");

  tNextSample = lastSample = winStart = millis();
}

void loop(){
  uint32_t now = millis();
  if(now < tNextSample) return;
  tNextSample += SAMPLE_MS;

  /* ---------- Sample ---------- */
  uint32_t dt = now - lastSample; lastSample = now;

  double vPV  = inaPV.getBusVoltage_V();
  double iPV  = inaPV.getCurrent_mA();
  double vSYS = inaSYS.getBusVoltage_V();
  double iSYS = inaSYS.getCurrent_mA();

  /* ---- Noise gate / sign ---- */
  double pPV  = (fabs(iPV ) < 0.5 || iPV  < 0) ? 0 : vPV  * iPV;   // mW
  double pSYS = (fabs(iSYS) < 0.5 || iSYS < 0) ? 0 : vSYS * iSYS;  // mW

  PV_mWms  += (int64_t) lround(pPV  * dt);     // mW·ms
  SYS_mWms += (int64_t) lround(pSYS * dt);

  sumVPV += vPV;  sumIPV += iPV;
  sumVS  += vSYS; sumIS  += iSYS;
  ++nSamples;

  /* ---------- Window end? ---------- */
  if(now - winStart >= WINDOW_MS){
    /* -- averages -- */
    double Vpv = sumVPV / nSamples;
    double Ipv = sumIPV / nSamples;
    double Vsys= sumVS  / nSamples;
    double Isys= sumIS  / nSamples;

    /* -- mWh ×1000 (3 decimals) -- */
    int32_t pv_mWh1000  = (int32_t) lround(PV_mWms  / 3600.0);   // ÷3 600 000 ×1000
    int32_t sys_mWh1000 = (int32_t) lround(SYS_mWms / 3600.0);

    uint16_t pv_u16  = pv_mWh1000  < 0 ? 0 : (pv_mWh1000  > 65535 ? 65535 : pv_mWh1000);
    uint16_t sys_u16 = sys_mWh1000 < 0 ? 0 : (sys_mWh1000 > 65535 ? 65535 : sys_mWh1000);

    uint16_t Vpv100  = (uint16_t) lround(Vpv  * 100);
    uint16_t Vsys100 = (uint16_t) lround(Vsys * 100);
    int16_t  Ipv100  = (int16_t)  lround(Ipv  * 100);
    int16_t  Isys100 = (int16_t)  lround(Isys * 100);

    uint8_t buf[12];
    putU16(buf,0, pv_u16);
    putU16(buf,2, sys_u16);
    putU16(buf,4, Vpv100);
    putU16(buf,6, Vsys100);
    putS16(buf,8, Ipv100);
    putS16(buf,10,Isys100);

    sendLoRa(buf,12);
    Serial.println("Uplink ✔");

    /* reset window */
    PV_mWms = SYS_mWms = 0;
    sumVPV = sumIPV = sumVS = sumIS = 0;
    nSamples = 0;
    winStart += WINDOW_MS;
  }
}
