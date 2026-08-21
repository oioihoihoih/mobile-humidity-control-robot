#include <Arduino.h>

// I2C 버스를 끌어내리는 보드를 찾기 위한 일시 진단 코드입니다.
// 릴레이/MOS는 OFF하고 A4/A5는 고임피던스 입력으로 완전히 해제합니다.
constexpr bool RELAY_ACTIVE_LOW = true;

void forceOutputsOff() {
  pinMode(A0, OUTPUT);
  pinMode(A1, OUTPUT);
  pinMode(9, OUTPUT);
  digitalWrite(A0, RELAY_ACTIVE_LOW ? HIGH : LOW);
  digitalWrite(A1, RELAY_ACTIVE_LOW ? HIGH : LOW);
  digitalWrite(9, LOW);
}

void printLines() {
  Serial.print(F("[PASSIVE] local A4="));
  Serial.print(digitalRead(A4));
  Serial.print(F(" A5="));
  Serial.println(digitalRead(A5));
}

void setup() {
  forceOutputsOff();
  pinMode(A4, INPUT);
  pinMode(A5, INPUT);
  digitalWrite(A4, LOW);  // 내부 풀업도 끈다.
  digitalWrite(A5, LOW);
  Serial.begin(9600);
  delay(1000);
  Serial.println(F("[BOOT] I2C pins passively released; outputs OFF"));
  printLines();
}

void loop() {
  static unsigned long lastLogAt = 0;
  static byte previousA4 = digitalRead(A4);
  static byte previousA5 = digitalRead(A5);
  static unsigned int a4Edges = 0;
  static unsigned int a5Edges = 0;
  const byte currentA4 = digitalRead(A4);
  const byte currentA5 = digitalRead(A5);
  if (currentA4 != previousA4) {
    previousA4 = currentA4;
    ++a4Edges;
  }
  if (currentA5 != previousA5) {
    previousA5 = currentA5;
    ++a5Edges;
  }
  if (millis() - lastLogAt >= 1000) {
    lastLogAt = millis();
    printLines();
    Serial.print(F("[EDGES/1s] A4="));
    Serial.print(a4Edges);
    Serial.print(F(" A5="));
    Serial.println(a5Edges);
    a4Edges = 0;
    a5Edges = 0;
  }
}
