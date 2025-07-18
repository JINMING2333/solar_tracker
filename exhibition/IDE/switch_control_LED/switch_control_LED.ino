const int buttonPin = 13;     // 按钮接 D2
const int lampMosfetPin = 24; // 控制 MOSFET 的 D3

void setup() {
  pinMode(buttonPin, INPUT_PULLUP); // 上拉输入
  pinMode(lampMosfetPin, OUTPUT);   // 控制 MOSFET
  digitalWrite(lampMosfetPin, LOW); // 默认灯灭
  Serial.begin(9600);
}

void loop() {
  int buttonState = digitalRead(buttonPin);

  if (buttonState == LOW) {  // 按下按钮
    digitalWrite(lampMosfetPin, HIGH);  // 点亮灯泡
    Serial.println("灯泡已点亮");
  } else {
    digitalWrite(lampMosfetPin, LOW);   // 灯灭
    Serial.println("灯泡已熄灭");
  }

  delay(100); // 避免读数抖动
}