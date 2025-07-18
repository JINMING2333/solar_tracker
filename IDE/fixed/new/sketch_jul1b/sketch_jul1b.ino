/******************************************************************
 *  MKR WAN 1310  ·  Fixed Panel  ·  5-min Energy (mWh×1000)
 *  2025-07-xx  仅负值门限：<0 mA 不计，≥0 mA 全部积分
 ******************************************************************/
#include <Arduino.h>
#include <Adafruit_INA219.h>
#include <MKRWAN.h>

/* ------------ INA219 ------------ */
Adafruit_INA219 inaPV (0x40);
Adafruit_INA219 inaSYS(0x41);

/* ------------ LoRa Keys ---------- */
String appEuiStr = "0000000000000000";
String appKeyStr = "817A05EB77508E8C2FC8410C040CB706";

/* ------------ Timing ------------- */
const uint16_t SAMPLE_MS  = 1000;                 // 1 s
const uint8_t  INT_MIN    = 5;                    // uplink every 5 min
const uint32_t WIN_MS     = INT_MIN * 60UL * 1000UL;

/* ------------ Accumulators ------- */
uint32_t tNext = 0, tLast = 0, tWin = 0;
int64_t  PV_mWms  = 0, SYS_mWms = 0;
double   sumVPV = 0, sumIPV = 0, sumVS = 0, sumIS = 0;
uint16_t nSmp   = 0;

/* ------------ Helpers ------------ */
LoRaModem modem;
inline void putU16(uint8_t *b,int i,uint16_t v){ b[i]=v>>8; b[i+1]=v; }
inline void putS16(uint8_t *b,int i,int16_t v){ b[i]=v>>8; b[i+1]=v; }

void sendLoRa(const uint8_t *p, uint8_t len){
  modem.beginPacket(); modem.write(p, len); modem.endPacket(true);
}

/* ------------ Setup -------------- */
void setup(){
  Serial.begin(115200); while(!Serial && millis()<4000){}

  inaPV.begin();  inaPV.setCalibration_32V_2A();
  inaSYS.begin(); inaSYS.setCalibration_32V_2A();

  if(!modem.begin(EU868)){ while(1); }
  modem.setADR(true); modem.dataRate(5);
  if(!modem.joinOTAA(appEuiStr, appKeyStr, 60000)){ while(1); }

  tNext = tLast = tWin = millis();
}

/* ------------ Loop --------------- */
void loop(){
  uint32_t now = millis();
  if(now < tNext) return;
  tNext += SAMPLE_MS;

  uint32_t dt = now - tLast; tLast = now;

  /* ---------- Sample ---------- */
  double vPV  = inaPV.getBusVoltage_V();
  double iPV  = inaPV.getCurrent_mA();
  double vSYS = inaSYS.getBusVoltage_V();
  double iSYS = inaSYS.getCurrent_mA();

  /* ---- 仅负值门限 ---- */
  double pPV  = (iPV  < 0) ? 0 : vPV  * iPV;     // mW
  double pSYS = (iSYS < 0) ? 0 : vSYS * iSYS;    // mW

  PV_mWms  += (int64_t) lround(pPV  * dt);       // mW·ms
  SYS_mWms += (int64_t) lround(pSYS * dt);

  sumVPV += vPV;  sumIPV += iPV;
  sumVS  += vSYS; sumIS  += iSYS;
  ++nSmp;

  /* ---------- Window end ---------- */
  if(now - tWin >= WIN_MS){
    /* averages */
    double Vpv  = sumVPV / nSmp;
    double Ipv  = sumIPV / nSmp;
    double Vsys = sumVS  / nSmp;
    double Isys = sumIS  / nSmp;

    /* mWh ×1000 */
    uint16_t pv_u16  = min<int32_t>( max<int32_t>(PV_mWms  / 3600, 0), 65535);
    uint16_t sys_u16 = min<int32_t>( max<int32_t>(SYS_mWms / 3600, 0), 65535);

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
    Serial.println("Uplink sent");

    /* reset */
    PV_mWms = SYS_mWms = 0;
    sumVPV = sumIPV = sumVS = sumIS = 0;
    nSmp   = 0;
    tWin  += WIN_MS;
  }
}
