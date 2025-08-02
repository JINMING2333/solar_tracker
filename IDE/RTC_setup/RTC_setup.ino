#include <Wire.h>
#include "RTClib.h"

RTC_DS3231 rtc;

void setup () {
  Serial.begin(9600);
  while (!Serial);

  if (!rtc.begin()) {
    Serial.println("找不到 DS3231，检查接线！");
    while (1);
  }

  // rtc.adjust(DateTime(F(__DATE__), F(__TIME__))); // 初始运行
  
  // 仅当掉电后才设置时间
  if (rtc.lostPower()) {
    Serial.println("DS3231 掉电，正在设置时间...");
    rtc.adjust(DateTime(F(__DATE__), F(__TIME__))); // 设置为电脑编译时间
  }

  Serial.println("DS3231 初始化完成");
}

void loop () {
  DateTime now = rtc.now();
  Serial.print("当前时间: ");
  Serial.print(now.year()); Serial.print('/');
  Serial.print(now.month()); Serial.print('/');
  Serial.print(now.day()); Serial.print(" ");
  Serial.print(now.hour()); Serial.print(':');
  Serial.print(now.minute()); Serial.print(':');
  Serial.println(now.second());
  delay(1000);
}
