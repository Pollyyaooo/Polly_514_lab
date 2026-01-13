#include <Arduino.h>

const int adcPin = 4;   // XIAO ESP32C3 上的 GPIO4（ADC）

void setup() {
  Serial.begin(115200);
  delay(1000); // 给串口一点时间
}

void loop() {
  int adcValue = analogRead(adcPin);
  float voltage = (adcValue / 4095.0) * 3.3;

  Serial.print("ADC Value: ");
  Serial.print(adcValue);
  Serial.print(" | Voltage: ");
  Serial.print(voltage, 3);
  Serial.println(" V");

  delay(1000);
}
