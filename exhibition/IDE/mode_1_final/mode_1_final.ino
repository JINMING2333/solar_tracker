// === 非阻塞模式一：步进电机与直流电机同步运行并归位 ===
#include <AccelStepper.h>

// 步进电机设置（28BYJ-48）
const int stepsPerRevolution = 2048;
AccelStepper stepper(AccelStepper::FULL4WIRE, 8, 10, 9, 11);

// 直流电机控制引脚（TB6612）
const int PWMA = 5;
const int AIN1 = 7;
const int AIN2 = 4;
const int STBY = 6;

// 模式按钮（船型按钮）
const int mode1ButtonPin = 12;

// 状态控制
bool mode1Requested = false;
bool mode1Running = false;
bool mode1Finished = true;
unsigned long lastDebounceTime = 0;
const unsigned long debounceDelay = 50;
bool lastButtonState = HIGH;
bool currentButtonState = HIGH;

// 步进电机状态
long stepperTarget = 0;
bool stepperForward = true;
unsigned long stepperStartTime = 0;
bool stepperHoldBeforeReturn = false;
unsigned long stepperHoldStartTime = 0;
const unsigned long stepperHoldDelay = 3000;  // 3秒延迟

// 直流电机状态（缓慢前进+快速回转）
const int DC_STEP_DELAY = 150;
const int DC_STEP_WAIT = 1516;
const int DC_NUM_STEPS = 12;
int dcMotorStep = 0;  // 0=停止，1=缓慢前进，2=等待回转，3=快速回转
unsigned long dcLastStepTime = 0;
int dcCurrentStep = 0;
const int PWM_VALUE = 120;

unsigned long dcReturnTriggerTime = 0;
bool dcReadyToReturn = false;

void setup() {
  Serial.begin(9600);

  pinMode(mode1ButtonPin, INPUT_PULLUP);

  pinMode(PWMA, OUTPUT);
  pinMode(AIN1, OUTPUT);
  pinMode(AIN2, OUTPUT);
  pinMode(STBY, OUTPUT);
  digitalWrite(STBY, HIGH);

  stepper.setMaxSpeed(50);
  stepper.setAcceleration(200);
  stepper.moveTo(0);

  Serial.println("等待模式一按钮...");
}

void loop() {
  // 按钮防抖逻辑
  bool reading = digitalRead(mode1ButtonPin);
  if (reading != lastButtonState) {
    lastDebounceTime = millis();
  }
  if ((millis() - lastDebounceTime) > debounceDelay) {
    if (reading != currentButtonState) {
      currentButtonState = reading;
      if (currentButtonState == LOW) {
        mode1Requested = true;
        Serial.println("→ 模式一请求开启");
      } else {
        mode1Requested = false;
        Serial.println("← 模式一请求关闭");
      }
    }
  }
  lastButtonState = reading;

  // 启动模式一
  if (mode1Requested && !mode1Running) {
    mode1Running = true;
    mode1Finished = false;
    stepperForward = true;
    stepper.setMaxSpeed(50);
    stepper.moveTo(-stepsPerRevolution / 2);
    stepperStartTime = millis();
    dcMotorStep = 0;
    dcCurrentStep = 0;
    dcReadyToReturn = false;
    stepperHoldBeforeReturn = false;
    Serial.println("→ 步进电机开始缓慢旋转 180°");
  }

  // 步进电机运行逻辑
  if (mode1Running) {
    stepper.run();

    // 启动直流电机（延迟3秒）
    if (stepperForward && dcMotorStep == 0 && millis() - stepperStartTime >= 3000) {
      dcMotorStep = 1;
      dcLastStepTime = millis();
      dcCurrentStep = 0;
      Serial.println("→ 直流电机延迟后开始缓慢前进");
    }

    // 步进电机达到目标后，延迟一段时间再回转
    if (stepper.distanceToGo() == 0) {
      if (stepperForward && !stepperHoldBeforeReturn) {
        stepperHoldBeforeReturn = true;
        stepperHoldStartTime = millis();
        Serial.println("⏸ 步进电机已达 180°，等待 3 秒再回转...");
      }
      else if (!stepperForward && !dcReadyToReturn) {
        dcReadyToReturn = true;
        dcReturnTriggerTime = millis();
        Serial.println("✔ 步进电机归位，准备直流电机回转");
      }
    }

    // 超过等待时间后回转步进电机
    if (stepperHoldBeforeReturn && millis() - stepperHoldStartTime >= stepperHoldDelay) {
      stepperForward = false;
      stepper.setMaxSpeed(600);
      stepper.moveTo(0);
      stepperHoldBeforeReturn = false;
      Serial.println("← 步进电机快速回到 0°");
    }
  }

  // 直流电机缓慢前进控制
  if (dcMotorStep == 1 && millis() - dcLastStepTime >= DC_STEP_WAIT && dcCurrentStep < DC_NUM_STEPS) {
    digitalWrite(AIN1, HIGH);
    digitalWrite(AIN2, LOW);
    analogWrite(PWMA, PWM_VALUE);
    delay(DC_STEP_DELAY);
    analogWrite(PWMA, 0);
    dcLastStepTime = millis();
    dcCurrentStep++;
    Serial.print("直流电机步数：");
    Serial.println(dcCurrentStep);

    if (dcCurrentStep >= DC_NUM_STEPS) {
      dcMotorStep = 2;
      Serial.println("→ 直流电机等待回转");
    }
  }

  // 步进归位后延迟 1 秒再回转直流电机
  if (dcReadyToReturn && millis() - dcReturnTriggerTime >= 1000 && dcMotorStep == 2) {
    dcMotorStep = 3;
    dcLastStepTime = millis();
    digitalWrite(AIN1, LOW);
    digitalWrite(AIN2, HIGH);
    analogWrite(PWMA, PWM_VALUE);
    Serial.println("← 直流电机快速归位中...");
  }

  // 停止直流电机归位后
  if (dcMotorStep == 3 && millis() - dcLastStepTime >= 1800) {
    analogWrite(PWMA, 0);
    digitalWrite(AIN1, LOW);
    digitalWrite(AIN2, LOW);
    dcMotorStep = 0;
    dcReadyToReturn = false;
    Serial.println("✔ 直流电机已归位");

    // 完整周期结束
    mode1Running = false;
    mode1Finished = true;
    Serial.println("✅ 模式一周期完成");
    delay(3000);
  }

  // 如果按钮断开 + 完成，确保停机并释放步进电机
  if (!mode1Requested && mode1Finished) {
    analogWrite(PWMA, 0);
    digitalWrite(AIN1, LOW);
    digitalWrite(AIN2, LOW);
    stepper.stop();
    stepper.moveTo(0);
    for (int pin = 8; pin <= 11; pin++) {
      digitalWrite(pin, LOW);
    }
  }
}
