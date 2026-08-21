#include <Arduino.h>
#include <SPI.h>
#include <MFRC522.h>

// 자동차 RC522 단독 진단용
// SDA/SCK/MOSI/MISO/x/GNDRST//3.3V  -> D9/D13/D11/D12/ x /  GND / D8 / 3.3V
constexpr byte RFID_SS_PIN = 9;
constexpr byte RFID_RST_PIN = 8;

MFRC522 rfid(RFID_SS_PIN, RFID_RST_PIN);
unsigned long lastWaitingLogAt = 0;

void printUid() {
  Serial.print(F("[RFID TEST] UID="));
  for (byte index = 0; index < rfid.uid.size; ++index) {
    if (rfid.uid.uidByte[index] < 0x10) Serial.print('0');
    Serial.print(rfid.uid.uidByte[index], HEX);
    if (index + 1 < rfid.uid.size) Serial.print(' ');
  }
  Serial.println();
}

void setup() {
  Serial.begin(9600);
  pinMode(10, OUTPUT);  // UNO를 SPI 마스터 상태로 유지
  SPI.begin();
  rfid.PCD_Init();
  delay(50);
  rfid.PCD_SetAntennaGain(MFRC522::RxGain_max);
  rfid.PCD_AntennaOn();

  const byte version = rfid.PCD_ReadRegister(MFRC522::VersionReg);
  Serial.println();
  Serial.println(F("[RFID TEST] RC522 standalone diagnostic"));
  Serial.print(F("[RFID TEST] version=0x"));
  if (version < 0x10) Serial.print('0');
  Serial.println(version, HEX);
  Serial.println(F("[RFID TEST] antenna gain=MAX"));
  Serial.println(F("[RFID TEST] keep the card on the antenna"));
}

void loop() {
  if (!rfid.PICC_IsNewCardPresent()) {
    if (millis() - lastWaitingLogAt >= 1000) {
      lastWaitingLogAt = millis();
      Serial.println(F("[RFID TEST] no card"));
    }
    return;
  }

  Serial.println(F("[RFID TEST] card field detected"));
  if (!rfid.PICC_ReadCardSerial()) {
    Serial.println(F("[RFID TEST] UID read failed"));
    return;
  }
  printUid();
  rfid.PICC_HaltA();
  rfid.PCD_StopCrypto1();
  delay(800);
}
