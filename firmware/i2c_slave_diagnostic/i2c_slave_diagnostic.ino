#include <Arduino.h>
#include <Wire.h>

volatile byte lastCommand = 0;

void receiveCommand(int count) {
  if (count > 0 && Wire.available()) lastCommand = Wire.read();
  while (Wire.available()) Wire.read();
}

void sendState() {
  Wire.write(static_cast<byte>(0));  // IDLE
  Wire.write(lastCommand);
}

void setup() {
  // Actuator 출력은 진단 중에도 항상 OFF로 유지한다.
  pinMode(A0, OUTPUT);
  pinMode(A1, OUTPUT);
  pinMode(9, OUTPUT);
  digitalWrite(A0, HIGH);
  digitalWrite(A1, HIGH);
  digitalWrite(9, LOW);

  Serial.begin(9600);
  Wire.begin(0x09);
  Wire.onReceive(receiveCommand);
  Wire.onRequest(sendState);
  Serial.println(F("[BOOT] minimal I2C slave 0x09; outputs OFF"));
}

void loop() {}
