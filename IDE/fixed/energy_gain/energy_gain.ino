#include <Wire.h>
#include <Adafruit_INA219.h>
#include <Servo.h>

Adafruit_INA219 ina219_input(0x40);   // 默认地址，用于太阳能输入
Adafruit_INA219 ina219_output(0x41);  // 改过地址的模块，用于系统耗电
Servo myServo;

void setup() {
  Wire.begin();
  Serial.begin(115200);
  delay(2000);

  myServo.attach(6);  // 舵机信号线接 D6
  myServo.write(90);  // 初始位置

  if (!ina219_input.begin()) {
  Serial.println("Failed to find INA219 at 0x40 (Input)");
  while (1);
}

  if (!ina219_output.begin()) {
    Serial.println("Failed to find INA219 at 0x41 (Output)");
    while (1);
  }

  Serial.println("Dual INA219 Monitor Ready!");
}

void loop() {
  // 驱动舵机旋转（增加功耗）
  myServo.write(0);
  delay(1000);
  myServo.write(180);
  delay(1000);
  
  // 太阳能输入 INA219
  float v_in = ina219_input.getBusVoltage_V();
  float i_in = ina219_input.getCurrent_mA();
  float p_in = ina219_input.getPower_mW();

  // 系统耗电 INA219
  float v_out = ina219_output.getBusVoltage_V();
  float i_out = ina219_output.getCurrent_mA();
  float p_out = ina219_output.getPower_mW();

  // 计算净功率
  float net_power = p_in - p_out;

  // 打印结果
  Serial.println("=== Energy Monitor ===");
  Serial.print("Solar Input: ");
  Serial.print(v_in); Serial.print(" V, ");
  Serial.print(i_in); Serial.print(" mA, ");
  Serial.print(p_in); Serial.println(" mW");

  Serial.print("System Output: ");
  Serial.print(v_out); Serial.print(" V, ");
  Serial.print(i_out); Serial.print(" mA, ");
  Serial.print(p_out); Serial.println(" mW");

  Serial.print("Net Power: ");
  Serial.print(net_power); Serial.println(" mW");

  Serial.println("------------------------\n");

  delay(1000);
}