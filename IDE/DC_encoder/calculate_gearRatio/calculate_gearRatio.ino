#include <Arduino.h>

const int IN1  = 3;
const int IN2  = 4;
const int PWM  = 5;
const int STBY = 2;          

const int ENC_A = 6;         // 蓝
const int ENC_B = 7;         // 白
const int ENC_PPR = 3;

volatile long encCount = 0;   //编码器脉冲记数
void ISR_encA() {
  encCount += digitalRead(ENC_B) ? -1 : +1;  //根据B相电平判断方向：高电平-反转-计数+1；低电平-正转-计数-1
}

void setup() {
  Serial.begin(115200);

  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(PWM, OUTPUT);
  pinMode(STBY, OUTPUT);          
  digitalWrite(STBY, HIGH);       // 退出休眠

  pinMode(ENC_A, INPUT_PULLUP);
  pinMode(ENC_B, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ENC_A), ISR_encA, RISING); //A相上升，中断

  /* 全速正转 */
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  analogWrite(PWM, 255); //全速旋转

  Serial.println(F("Spinning for 10 s …"));
  delay(1000); //持续10S

  /* 刹车 */
  analogWrite(PWM, 0); //关闭PWM
  digitalWrite(IN1, LOW); //两引脚降低，停止旋转
  digitalWrite(IN2, LOW);

  Serial.print(F("Pulse total = ")); Serial.println(encCount); //打印总脉冲数
}

void loop() {}
