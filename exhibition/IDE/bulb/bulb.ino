void setup() {
  Serial.begin(9600);
  pinMode(3, OUTPUT);
  digitalWrite(3, HIGH);
  Serial.println("灯泡应当点亮");
}

void loop() {
}