/******************************************************************
 *  Dump-Load Battery Monitor with BQ24074 + MKR WAN 1310
 *  Features:
 *    - INA219 monitors PV and system power
 *    - A0 reads battery voltage (via voltage divider)
 *    - D13 reads CHG pin (charging state)
 *    - D7 controls MOSFET gate (dump load resistor)
 ******************************************************************/
#include <Arduino.h>
#include <Adafruit_INA219.h>

// ───── Pin Definitions ─────
const int PIN_CHG    = 13;   // CHG pin from BQ24074
const int PIN_DUMP   = 7;    // MOSFET Gate
const int PIN_VBAT   = A0;   // Voltage divider input

// ───── Voltage Divider ─────
const float VBAT_DIVIDER_RATIO = 2.0;   // 100k/100k divider
const float ADC_REF_VOLTAGE    = 3.3;   // MKR ADC reference
const int   ADC_MAX_VALUE      = 1023;

// ───── Dump Load Control ─────
const float VBAT_FULL   = 4.20;   // Battery considered full
const float VBAT_RESUME = 4.05;   // Resume charging
bool dumping = false;

// ───── INA219 ─────
Adafruit_INA219 inaPV(0x40);
Adafruit_INA219 inaSYS(0x41);

void setup() {
  Serial.begin(115200);
  delay(300);

  pinMode(PIN_CHG, INPUT);
  pinMode(PIN_DUMP, OUTPUT);
  digitalWrite(PIN_DUMP, LOW); // Start with dump load off

  inaPV.begin();
  inaPV.setCalibration_32V_2A();
  inaSYS.begin();
  inaSYS.setCalibration_32V_2A();

  Serial.println("\n🔋 Dump Load Monitor Initialized");
}

void loop() {
  // ───── Battery Voltage ─────
  int raw = analogRead(PIN_VBAT);
  float vbat = raw * ADC_REF_VOLTAGE * VBAT_DIVIDER_RATIO / ADC_MAX_VALUE;

  // ───── Charging Status ─────
  bool charging = (digitalRead(PIN_CHG) == LOW);  // LOW = charging

  // ───── INA Measurements ─────
  float vPV = inaPV.getBusVoltage_V();
  float iPV = inaPV.getCurrent_mA();
  float vSYS = inaSYS.getBusVoltage_V();
  float iSYS = inaSYS.getCurrent_mA();

  // ───── Dump Logic ─────
  if (!charging && vbat >= VBAT_FULL && !dumping) {
    digitalWrite(PIN_DUMP, HIGH);
    dumping = true;
    Serial.println("⚠️ Battery full — starting dump");
  } else if (dumping && vbat <= VBAT_RESUME) {
    digitalWrite(PIN_DUMP, LOW);
    dumping = false;
    Serial.println("✅ Dump complete — resuming charge");
  }

  // ───── Serial Print ─────
  Serial.print("VBAT: "); Serial.print(vbat, 2);
  Serial.print(" V  |  CHG: "); Serial.print(charging ? "CHARGING" : "FULL");
  Serial.print("  |  DUMP: "); Serial.print(dumping ? "ON" : "OFF");
  Serial.print("  |  PV: "); Serial.print(vPV, 2);
  Serial.print(" V / "); Serial.print(iPV, 1); Serial.print(" mA");
  Serial.print("  |  SYS: "); Serial.print(vSYS, 2);
  Serial.print(" V / "); Serial.print(iSYS, 1); Serial.println(" mA");

  delay(1000);
}
