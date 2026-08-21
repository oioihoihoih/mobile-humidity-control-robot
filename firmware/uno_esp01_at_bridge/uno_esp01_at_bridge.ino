#include <Arduino.h>
#include <SoftwareSerial.h>

// ESP-01 진단용 USB <-> SoftwareSerial 중계기
// ESP-01 TX -> D6, ESP-01 RX <- D7(3.3V 분압)
SoftwareSerial esp8266(6, 7);

void setup() {
  Serial.begin(9600);
  esp8266.begin(9600);
  Serial.println(F("ESP-01 AT BRIDGE READY"));
}

void loop() {
  while (Serial.available()) esp8266.write(Serial.read());
  while (esp8266.available()) Serial.write(esp8266.read());
}
