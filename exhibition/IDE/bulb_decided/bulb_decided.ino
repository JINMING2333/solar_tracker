const int lampPin = 3;  // 控制灯泡的数字口（连接 MOSFET gate）
const int ldr1Pin = A0; // LDR1
const int ldr2Pin = A1; // LDR2

void setup() {
  pinMode(lampPin, OUTPUT);
  digitalWrite(lampPin, LOW); // 初始关闭灯泡
  Serial.begin(9600);
  delay(1000);
  Serial.println("开始测试灯泡对LDR的影响...");
}

void loop() {
  // 点亮灯泡
  digitalWrite(lampPin, HIGH);
  delay(1000); // 给灯泡一个稳定时间

  // 读取 LDR 数值
  int ldr1Value = analogRead(ldr1Pin);
  int ldr2Value = analogRead(ldr2Pin);

  // 打印结果
  Serial.print("灯泡 ON | LDR1: ");
  Serial.print(ldr1Value);
  Serial.print(" | LDR2: ");
  Serial.println(ldr2Value);

  delay(2000);

  // 熄灭灯泡
  digitalWrite(lampPin, LOW);
  delay(1000); // 暗环境下读一遍 LDR

  // 再读一次 LDR
  ldr1Value = analogRead(ldr1Pin);
  ldr2Value = analogRead(ldr2Pin);

  Serial.print("灯泡 OFF | LDR1: ");
  Serial.print(ldr1Value);
  Serial.print(" | LDR2: ");
  Serial.println(ldr2Value);

  Serial.println("-----");
  delay(3000); // 3 秒休息再测下一轮
}
