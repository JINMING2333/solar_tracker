#include <Wire.h>
#include <Adafruit_INA219.h>

/* ---------- pin map ---------- */
const byte IN1   = 3;
const byte IN2   = 4;
const byte PWM   = 5;
const byte STBY  = 2;
const byte ENC_A = 6;         // blue  – A-phase
const byte ENC_B = 7;         // white – B-phase

/* ---------- motion parameters ---------- */
const uint8_t  STEP_DEG     = 12;
const uint8_t  STEP_COUNT   = 10;            // 0 → 120 °
const uint16_t speedPWM     = 60;            // TB6612 PWM (0-255)
const unsigned long DWELL_MS      = 30000UL; // 30 s
const unsigned long FINAL_DWELL_MS= 60000UL; // 60 s

/* ---------- encoder constants ---------- */
const float    PULSE_PER_DEG= 2.436; 

/* ---------- INA219 ---------- */
Adafruit_INA219 ina219(0x41);
const unsigned long SAMPLE_MS = 50;          // 20 Hz
unsigned long lastSample = 0;

/* ---------- globals ---------- */
volatile long encCount = 0;                  // updated in ISR
double  targetPulse = 0;                  // 逻辑目标脉冲（累积，double 不取整）
long    baseCount   = 0;                  // 0 ° 时的基准

/* ---------------- encoder ISR -------------- */
void isrEncA() { 
  encCount += digitalRead(ENC_B) ? -1 : +1; 
}

/* ------------- motor helpers --------------- */
inline void motorRun(bool fwd, uint8_t pwm)
{
  digitalWrite(IN1, fwd ? HIGH : LOW);
  digitalWrite(IN2, fwd ? LOW  : HIGH);
  analogWrite (PWM, pwm);
}
inline void motorBrake()
{
  analogWrite(PWM, 0);
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
}

/* ------------- INA219 sampler -------------- */
void serviceINA219()
{
   unsigned long now = millis();
  if (now - lastSample >= SAMPLE_MS)
  {
    lastSample = now;
    float v = ina219.getBusVoltage_V();
    float i = ina219.getCurrent_mA();
    float p = v * i;

    float angleDeg = (encCount - baseCount) / PULSE_PER_DEG;

    Serial.print(now);      Serial.print(',');
    Serial.print(angleDeg,1);     Serial.print(',');
    Serial.print(v,2);            Serial.print(',');
    Serial.print(i,1);            Serial.print(',');
    Serial.println(p,2);
  }
}

/* -------- pause that keeps sampling -------- */
void dwellWithSampling(unsigned long dur)
{
  unsigned long t0 = millis();
  while (millis() - t0 < dur)
    serviceINA219();
}

/* ------------- precise move ---------------- */
void moveByDeg(int deg)
{
  targetPulse += deg * PULSE_PER_DEG;
  long target = baseCount + lround(targetPulse);

  bool fwd = (deg > 0);
  motorRun(fwd, speedPWM);
  while (fwd ? (encCount < target) : (encCount > target))
    serviceINA219();              // non-blocking wait
  motorBrake();
}

/* ================== SETUP ================== */
void setup()
{
  Serial.begin(115200);
  pinMode(IN1, OUTPUT);  pinMode(IN2, OUTPUT);
  pinMode(PWM, OUTPUT);  pinMode(STBY, OUTPUT);
  digitalWrite(STBY, HIGH);        // awake

  pinMode(ENC_A, INPUT_PULLUP);
  pinMode(ENC_B, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ENC_A), isrEncA, RISING);

  Wire.begin();
  ina219.begin();
  ina219.setCalibration_32V_2A();

  /* ----------- wait for “START” ---------- */
  while (!Serial.available()) {}
  if (Serial.readStringUntil('\n') != "START") while (1);

  baseCount   = encCount;   // record 0°
  targetPulse = 0.0;
}

/* ================== LOOP =================== */
void loop()
{
  /* ---- 0 → 120 °, 10 steps ---- */
  for (uint8_t s = 0; s < STEP_COUNT; ++s)
  {
    dwellWithSampling(DWELL_MS);   // 30 s rest
    moveByDeg(STEP_DEG);           // +12 °
  }

  /* ---- dwell at 120 ° ---- */
  dwellWithSampling(FINAL_DWELL_MS);

  /* ---- one shot back to zero ---- */
  moveByDeg(-STEP_COUNT * STEP_DEG);
  //Serial.println(F("Cycle done."));

  while (true) serviceINA219();    // keep printing if you like
} 