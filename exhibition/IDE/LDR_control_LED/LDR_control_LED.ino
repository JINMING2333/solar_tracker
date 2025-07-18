#include <Adafruit_NeoPixel.h>

#define PIN 6
#define NUMPIXELS 16
Adafruit_NeoPixel strip(NUMPIXELS, PIN, NEO_GRB + NEO_KHZ800);

const int ldrLeft = A0;
const int ldrRight = A1;

const int lightThreshold = 350;
const unsigned long intervalPerGroup = 3000; // 每组灯点亮间隔：3 秒
const unsigned long ldrCheckInterval = 1000; // 每次读取 LDR 的频率：1 秒

unsigned long lightStartTime = 0;
bool lightingStarted = false;

unsigned long lastLdrCheckTime = 0;
int lastGroupCount = 0;

void setup() {
  Serial.begin(9600);
  strip.begin();
  strip.show(); // 所有灯初始化为熄灭
  Serial.println("系统初始化完成。等待光照信号...");
}

void loop() {
  // 限制每次读取 LDR 的频率为 1 秒
  if (millis() - lastLdrCheckTime >= ldrCheckInterval) {
    lastLdrCheckTime = millis();

    int leftVal = analogRead(ldrLeft);
    int rightVal = analogRead(ldrRight);

    Serial.print("LDR Left: ");
    Serial.print(leftVal);
    Serial.print(" | LDR Right: ");
    Serial.println(rightVal);

    if (leftVal >= lightThreshold && rightVal >= lightThreshold) {
      if (!lightingStarted) {
        lightingStarted = true;
        lightStartTime = millis();  // 开始计时
        lastGroupCount = 0;
        Serial.println("💡 光照检测成功，开始计时！");
      }

      // 计算已过去的时间
      unsigned long elapsed = millis() - lightStartTime;

      // 计算当前应点亮多少组灯（最多 4 组）
      int currentGroupCount = min(4, elapsed / intervalPerGroup);

      if (currentGroupCount != lastGroupCount) {
        Serial.print("✨ 第 ");
        Serial.print(currentGroupCount);
        Serial.println(" 组灯已点亮");
        lastGroupCount = currentGroupCount;
      }

      lightGroups(currentGroupCount);
    } else {
      // 光线不足，重置
      strip.clear();
      strip.show();
      if (lightingStarted) {
        Serial.println("🌑 光照消失，灯组已重置");
      }
      lightingStarted = false;
      lastGroupCount = 0;
    }
  }
}

void lightGroups(int count) {
  strip.clear();

  for (int i = 0; i < count * 4; i++) {
    if (i < 4)
      strip.setPixelColor(i, strip.Color(255, 255, 255)); // 白
    else if (i < 8)
      strip.setPixelColor(i, strip.Color(255, 255, 0));   // 黄
    else if (i < 12)
      strip.setPixelColor(i, strip.Color(255, 165, 0));   // 橙
    else
      strip.setPixelColor(i, strip.Color(255, 0, 0));     // 红
  }

  strip.show();
}
