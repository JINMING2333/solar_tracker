// 电机控制引脚定义
const int IN1  = 7;   // AIN1
const int IN2  = 4;   // AIN2
const int PWM  = 5;   // PWMA
const int STBY = 6;   // STBY

const int speedPWM = 200;         // 电机转速 PWM（0~255）
const unsigned long interval = 5000; // 每种状态持续时间（ms）

enum MotorState { FORWARD, STOPPED, BACKWARD };
MotorState currentState = FORWARD;
unsigned long lastSwitchTime = 0;

void setup() {
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(PWM, OUTPUT);
  pinMode(STBY, OUTPUT);

  Serial.begin(115200);
  digitalWrite(STBY, HIGH); // 启用驱动器

  setMotorState(currentState);
  lastSwitchTime = millis();
}

void loop() {
  unsigned long now = millis();
  if (now - lastSwitchTime >= interval) {
    // 状态切换
    switch (currentState) {
      case FORWARD:
        currentState = STOPPED;
        break;
      case STOPPED:
        currentState = BACKWARD;
        break;
      case BACKWARD:
        currentState = FORWARD;
        break;
    }
    setMotorState(currentState);
    lastSwitchTime = now;
  }
}

void setMotorState(MotorState state) {
  switch (state) {
    case FORWARD:
      digitalWrite(IN1, HIGH);
      digitalWrite(IN2, LOW);
      analogWrite(PWM, speedPWM);
      Serial.println("电机正转");
      break;
    case BACKWARD:
      digitalWrite(IN1, LOW);
      digitalWrite(IN2, HIGH);
      analogWrite(PWM, speedPWM);
      Serial.println("电机反转");
      break;
    case STOPPED:
      digitalWrite(IN1, LOW);
      digitalWrite(IN2, LOW);
      analogWrite(PWM, 0);
      Serial.println("电机停止");
      break;
  }
}
