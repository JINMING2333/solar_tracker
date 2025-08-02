#include <Arduino.h>
#include <math.h>
#include <Adafruit_INA219.h>
#include <MKRWAN.h>
#include <RTCZero.h>

/* ------------ INA219 ------------ */
Adafruit_INA219 inaPV (0x40);
Adafruit_INA219 inaSYS(0x41);

/* ------------ LoRa Keys ---------- */
String appEuiStr = "0000000000000000";
String appKeyStr = "817A05EB77508E8C2FC8410C040CB706";

/* ------------ Timing ------------- */
const uint32_t UP_INT_SEC = 5 * 60;  // 每5分钟唤醒一次

/* ------------ Accumulators ------- */
int64_t PV_mWms  = 0;    // mW·ms
int64_t SYS_mWms = 0;

double sumVPV = 0, sumIPV = 0, sumVS = 0, sumIS = 0;
uint16_t nSamples = 0;

/* ------------ Helpers ------------ */
LoRaModem modem;
RTCZero rtc;
volatile bool rtcWake = false;

inline void putU16(uint8_t *b,int i,uint16_t v){ b[i]=v>>8; b[i+1]=v; }
inline void putS16(uint8_t *b,int i,int16_t v){ b[i]=v>>8; b[i+1]=v; }

void sendLoRa(const uint8_t *p, uint8_t len){
  modem.beginPacket(); modem.write(p, len); modem.endPacket(true);
}

void alarmMatch() { rtcWake = true; }

void setup(){
  Serial.begin(115200);
  while(!Serial && millis() < 4000){}

  inaPV.begin();  inaPV.setCalibration_32V_2A();
  inaSYS.begin(); inaSYS.setCalibration_32V_2A();

  /* --- LoRa --- */
  if(!modem.begin(EU868)){ Serial.println("Modem init fail"); while(1); }
  modem.setADR(true); modem.dataRate(5);
  if(!modem.joinOTAA(appEuiStr, appKeyStr, 60000)){
    Serial.println("Join fail"); while(1);
  }
  Serial.println("Joined TTN ✔");

  rtc.begin();
  rtc.attachInterrupt(alarmMatch);
}

void loop(){
  rtcWake = false;

  /* ---------- Sample once per wakeup ---------- */
  double vPV  = inaPV.getBusVoltage_V();
  double iPV  = inaPV.getCurrent_mA();
  double vSYS = inaSYS.getBusVoltage_V();
  double iSYS = inaSYS.getCurrent_mA();

  double pPV  = (fabs(iPV ) < 0.5 || iPV  < 0) ? 0 : vPV  * iPV;
  double pSYS = (fabs(iSYS) < 0.5 || iSYS < 0) ? 0 : vSYS * iSYS;

  PV_mWms  += (int64_t) lround(pPV  * 1000.0);   // 1s采样，简化dt=1000ms
  SYS_mWms += (int64_t) lround(pSYS * 1000.0);

  sumVPV += vPV;  sumIPV += iPV;
  sumVS  += vSYS; sumIS  += iSYS;
  ++nSamples;

  /* ---- 每5分钟上传一次 ---- */
  if(nSamples >= UP_INT_SEC){
    double Vpv = sumVPV / nSamples;
    double Ipv = sumIPV / nSamples;
    double Vsys= sumVS  / nSamples;
    double Isys= sumIS  / nSamples;

    int32_t pv_mWh1000  = (int32_t) lround(PV_mWms  / 3600.0);
    int32_t sys_mWh1000 = (int32_t) lround(SYS_mWms / 3600.0);

    uint16_t pv_u16  = constrain(pv_mWh1000 , 0, 65535);
    uint16_t sys_u16 = constrain(sys_mWh1000, 0, 65535);
    uint16_t Vpv100  = (uint16_t) lround(Vpv  * 100);
    uint16_t Vsys100 = (uint16_t) lround(Vsys * 100);
    int16_t  Ipv100  = (int16_t)  lround(Ipv  * 100);
    int16_t  Isys100 = (int16_t)  lround(Isys * 100);

    uint8_t buf[12];
    putU16(buf,0, pv_u16);   putU16(buf,2, sys_u16);
    putU16(buf,4, Vpv100);   putU16(buf,6, Vsys100);
    putS16(buf,8, Ipv100);   putS16(buf,10,Isys100);

    sendLoRa(buf,12);
    Serial.println("Uplink ✔");

    // reset
    PV_mWms = SYS_mWms = 0;
    sumVPV = sumIPV = sumVS = sumIS = 0;
    nSamples = 0;
  }

  /* --------- Set RTC Alarm --------- */
  uint8_t s = rtc.getSeconds();
  uint8_t m = rtc.getMinutes();
  uint8_t h = rtc.getHours();

  uint8_t next_s = (s + 60) % 60;
  uint8_t next_m = (s + 60 >= 60) ? (m + 1) % 60 : m;
  uint8_t next_h = (s + 60 >= 60 && next_m == 0) ? (h + 1) % 24 : h;

  rtc.setAlarmSeconds(next_s);
  rtc.setAlarmMinutes(next_m);
  rtc.setAlarmHours(next_h);
  rtc.enableAlarm(rtc.MATCH_HHMMSS);

  /* ---- Sleep ---- */
  Serial.println("Sleeping…");
  LowPower.deepSleep();  // until RTC alarm
}
