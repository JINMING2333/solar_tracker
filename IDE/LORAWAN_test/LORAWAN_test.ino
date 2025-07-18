#include <MKRWAN.h>
LoRaModem modem;

const char *appEui = "0000000000000000";
const char *appKey = "5DEF1848C136307477DC8E930E463F8F";   // 32 hex chars

void setup() {
  Serial.begin(115200);
  while (!Serial);

  if (!modem.begin(EU868)) {
    Serial.println("LoRa modem init failed");
    while (1);
  }

  Serial.print("DevEUI: "); Serial.println(modem.deviceEUI());

  int tries = 0;
  while (!modem.joinOTAA(appEui, appKey) && tries++ < 8) {
    Serial.println("Join failed — move closer to a window and retrying...");
    delay(5000);
  }
  if (tries > 8) {
    Serial.println("Cannot join. Check keys / antenna / gateway.");
    while (1);
  }
  Serial.println("Joined TTN!");
}

void loop() {
  // 只做一次简单发送验证
  uint8_t payload[1] = { 0x01 };
  modem.beginPacket();
  modem.write(payload, 1);
  int err = modem.endPacket(true);   // 使用 confirmed uplink
  Serial.println(err > 0 ? "Uplink OK" : "Uplink failed");
  delay(60000);                      // 遵守占空比，每分钟一次
}