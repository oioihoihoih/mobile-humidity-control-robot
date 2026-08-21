#include <Arduino.h>
#include <Wire.h>

// 세 UNO의 A4/A5/GND 연결을 확인할 때 잠시 올리는 진단 전용 스케치입니다.
// ActuatorUno에서 실행해도 가습/제습 출력이 켜지지 않도록 가장 먼저 OFF합니다.
constexpr byte HUMIDIFIER_RELAY_PIN = A0;
constexpr byte PELTIER_RELAY_PIN = A1;
constexpr byte FAN_MOSFET_PIN = 7;
constexpr bool RELAY_ACTIVE_LOW = true;

void setSafeOutputs() {
  pinMode(HUMIDIFIER_RELAY_PIN, OUTPUT);
  pinMode(PELTIER_RELAY_PIN, OUTPUT);
  pinMode(FAN_MOSFET_PIN, OUTPUT);
  digitalWrite(HUMIDIFIER_RELAY_PIN, RELAY_ACTIVE_LOW ? HIGH : LOW);
  digitalWrite(PELTIER_RELAY_PIN, RELAY_ACTIVE_LOW ? HIGH : LOW);
  digitalWrite(FAN_MOSFET_PIN, LOW);
}

void releaseLine(byte pin) {
  pinMode(pin, INPUT_PULLUP);
}

void pullLineLow(byte pin) {
  digitalWrite(pin, LOW);
  pinMode(pin, OUTPUT);
}

// 선택한 SensorUno의 A4/A5가 실제 버스에 반대로 꽂혔는지 확인하는 저속 ACK 프로브.
bool softwareProbe(byte address, byte sdaPin, byte sclPin) {
  releaseLine(sdaPin);
  releaseLine(sclPin);
  delayMicroseconds(30);
  if (digitalRead(sdaPin) == LOW || digitalRead(sclPin) == LOW) return false;

  pullLineLow(sdaPin);
  delayMicroseconds(20);
  pullLineLow(sclPin);
  byte value = static_cast<byte>(address << 1);
  for (byte mask = 0x80; mask; mask >>= 1) {
    if (value & mask) releaseLine(sdaPin);
    else pullLineLow(sdaPin);
    delayMicroseconds(10);
    releaseLine(sclPin);
    delayMicroseconds(25);
    pullLineLow(sclPin);
  }
  releaseLine(sdaPin);
  delayMicroseconds(10);
  releaseLine(sclPin);
  delayMicroseconds(25);
  const bool ack = digitalRead(sdaPin) == LOW;
  pullLineLow(sclPin);
  pullLineLow(sdaPin);
  delayMicroseconds(10);
  releaseLine(sclPin);
  delayMicroseconds(10);
  releaseLine(sdaPin);
  return ack;
}

void scanBus() {
  byte found = 0;
  Serial.println(F("[SCAN] start 0x01..0x7E"));
  for (byte address = 1; address < 127; ++address) {
    Wire.beginTransmission(address);
    const byte result = Wire.endTransmission();
    if (result == 0) {
      Serial.print(F("[SCAN] ACK address=0x"));
      if (address < 0x10) Serial.print('0');
      Serial.println(address, HEX);
      ++found;
    } else if (result == 4) {
      Serial.print(F("[SCAN] bus error at 0x"));
      Serial.println(address, HEX);
    }
  }
  Serial.print(F("[SCAN] complete, devices="));
  Serial.println(found);
  Serial.print(F("[LINES] SDA(A4)="));
  Serial.print(digitalRead(SDA));
  Serial.print(F(" SCL(A5)="));
  Serial.println(digitalRead(SCL));
}

void setup() {
  setSafeOutputs();
  Serial.begin(9600);
  delay(1500);
  Serial.println(F("[BOOT] I2C BUS DIAGNOSTIC; outputs forced OFF"));
  Wire.begin();
  Wire.setClock(10000);
  Wire.setWireTimeout(25000UL, true);
  scanBus();
  Wire.end();
  Serial.print(F("[SWAPPED A5=SDA/A4=SCL] Motor 0x08="));
  Serial.println(softwareProbe(0x08, A5, A4) ? F("ACK") : F("NO ACK"));
  Serial.print(F("[SWAPPED A5=SDA/A4=SCL] LCD 0x27="));
  Serial.println(softwareProbe(0x27, A5, A4) ? F("ACK") : F("NO ACK"));
}

void loop() {
  if (Serial.available()) {
    const char command = Serial.read();
    if (command == 's' || command == 'S') scanBus();
  }
}
