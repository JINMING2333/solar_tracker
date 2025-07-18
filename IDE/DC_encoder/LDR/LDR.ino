#include <MKRWAN.h>
#include <Arduino.h>

/* —— LoRaWAN keys —— */
const char *APP_EUI = "0000000000000000";
const char *APP_KEY = "2A69608407FA3758BDA09FC6651B0C02";

/* —— 追踪器硬件常量 —— */
const byte IN1=3, IN2=4, PWM=5, STBY=2;
const byte ENC_A=6, ENC_B=7;
const byte LDR_L=A0, LDR_R=A1;
const float PULSE_PER_DEG = 2.463;
const long  MAX_PULSE     = 120 * PULSE_PER_DEG;
const uint8_t MOTOR_PWM   = 120;
int  THRESH_START = 100, THRESH_STOP = 10;

/* —— 全局 —— */
volatile long encCount = 0;
LoRaModem modem;
bool joined = false;
unsigned long nextUplink = 0;        // 下次允许发包的时间戳

/* === ISR === */
void isrEncA() { encCount += digitalRead(ENC_B) ? -1 : +1; }

/* === 辅助 === */
inline void motorRun(bool fwd,uint8_t pwm){
  digitalWrite(IN1,fwd);  digitalWrite(IN2,!fwd); analogWrite(PWM,pwm);}
inline void motorBrake(){
  analogWrite(PWM,0); digitalWrite(IN1,LOW); digitalWrite(IN2,LOW);}
inline bool atLeftEnd (){ return encCount<=0;}
inline bool atRightEnd(){ return encCount>=MAX_PULSE;}

void setup() {
  Serial.begin(115200);
  pinMode(IN1,OUTPUT); pinMode(IN2,OUTPUT); pinMode(PWM,OUTPUT);
  pinMode(STBY,OUTPUT); digitalWrite(STBY,HIGH);
  pinMode(ENC_A,INPUT_PULLUP); pinMode(ENC_B,INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ENC_A), isrEncA, RISING);

  /* --- LoRa init --- */
  if(!modem.begin(EU868)){ Serial.println("LoRa init failed"); while(1);}
  modem.setPort(Serial);                 // 打开调试
  Serial.print("DevEUI: "); Serial.println(modem.deviceEUI());

  joinNetwork();                         // 一次 join
}

void joinNetwork(){
  for(int i=0;i<5 && !joined;i++){
    Serial.print("Join attempt "); Serial.println(i+1);
    joined = modem.joinOTAA(APP_EUI,APP_KEY);
    if(!joined) delay(5000);
  }
  if(joined) Serial.println("Joined!");
  else       Serial.println("All join attempts failed");
}

void sendUplink(uint16_t, uint16_t, int16_t);

void loop(){

/* ================ 采集 LDR & 控制 ================ */
  int lVal = analogRead(LDR_L), rVal = analogRead(LDR_R);
  int diff = rVal - lVal, adiff = abs(diff);
  if(adiff >= THRESH_START){
      bool fwd = diff < 0;
      if( !(fwd && atRightEnd()) && !(!fwd && atLeftEnd()) ){
        long start = encCount;
        motorRun(fwd,MOTOR_PWM);
        while(true){
          lVal = analogRead(LDR_L); rVal = analogRead(LDR_R);
          diff = rVal - lVal; adiff = abs(diff);
          if(adiff<=THRESH_STOP) break;
          if( (fwd&&atRightEnd()) || (!fwd&&atLeftEnd()) ) break;
        }
        motorBrake();
      }
  }

/* ================ 每 60 s 发送一次 ================ */
  if(millis() > nextUplink){
    sendUplink(lVal, rVal, (int16_t)(encCount/PULSE_PER_DEG*10));
    nextUplink = millis() + 60000UL;        // 1 分钟一次
  }
}

/* === 封包并发送 === */
void sendUplink(uint16_t lVal,uint16_t rVal,int16_t degX10){
  if(!joined){ joinNetwork(); if(!joined) return; }   // 无网则重试 join

  byte buf[6] = {
    highByte(lVal), lowByte(lVal),
    highByte(rVal), lowByte(rVal),
    highByte(degX10), lowByte(degX10)
  };

  modem.beginPacket();
  modem.write(buf,sizeof(buf));
  int ret = modem.endPacket(false);        // ❗ false = 不要求 ACK
  if(ret>0) Serial.println("Uplink OK");
  else      Serial.println("Uplink failed");
}
