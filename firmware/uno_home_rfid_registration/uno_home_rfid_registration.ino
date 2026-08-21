#include <Arduino.h>

#include "SoftwareMFRC522.h"

// 현재 SensorUno RC522 배선을 그대로 사용하는 HOME 카드 UID 읽기 전용 예제.
// RC522의 SDA 표기는 I2C SDA가 아니라 SPI의 SS/CS 신호다.
constexpr byte RFID_SS_PIN = 8;
constexpr byte RFID_SCK_PIN = 9;
constexpr byte RFID_MOSI_PIN = 10;
constexpr byte RFID_MISO_PIN = 11;
constexpr byte RFID_RST_PIN = 12;

constexpr unsigned long WAITING_LOG_INTERVAL_MS = 2000UL;

SoftwareMFRC522 rfid(RFID_SS_PIN, RFID_RST_PIN, RFID_SCK_PIN,
                     RFID_MOSI_PIN, RFID_MISO_PIN);

bool rfidReady = false;
unsigned long lastWaitingLogAt = 0;

void printUidBytes() {
  for (byte index = 0; index < rfid.uid.size; ++index) {
    if (rfid.uid.uidByte[index] < 0x10) Serial.print('0');
    Serial.print(rfid.uid.uidByte[index], HEX);
    if (index + 1 < rfid.uid.size) Serial.print(' ');
  }
}

void printRegistrationResult() {
  Serial.println();
  Serial.print(F("[HOME UID] "));
  printUidBytes();
  Serial.println();

  Serial.print(F("[UID LENGTH] "));
  Serial.print(rfid.uid.size);
  Serial.println(F(" bytes"));

  Serial.print(F("[COPY TO PRIVATE CONFIG] #define ROBOT_HOME_UID \""));
  printUidBytes();
  Serial.println('"');
  Serial.println(F("[NOTICE] Keep this value only in ignored robot_network_config.h."));
  Serial.println(F("[NOTICE] HOME RFID supplements; it does not replace HOME marker calibration."));
  Serial.println(F("[NEXT] Remove the card, then scan again to verify the same UID."));
  Serial.println();
}

void setup() {
  Serial.begin(115200);
  rfid.PCD_Init();
  delay(50);

  const byte version =
      rfid.PCD_ReadRegister(SoftwareMFRC522::VersionReg);
  rfidReady = version != 0x00 && version != 0xFF;

  Serial.println();
  Serial.println(F("[BOOT] HOME RFID UID registration reader"));
  Serial.println(F("[SAFE] UID read only: no ESP, I2C, motor, relay, or EEPROM access"));
  Serial.println(F("[PINS] SS=D8 SCK=D9 MOSI=D10 MISO=D11 RST=D12 POWER=3.3V"));
  Serial.print(F("[RC522] VersionReg=0x"));
  if (version < 0x10) Serial.print('0');
  Serial.println(version, HEX);

  if (!rfidReady) {
    Serial.println(F("[ERROR] RC522 not detected; check 3.3V, GND, and D8-D12 wiring."));
    return;
  }

  Serial.println(F("[READY] Present exactly one HOME card to the antenna."));
}

void loop() {
  if (!rfidReady) return;

  if (!rfid.PICC_IsNewCardPresent()) {
    if (millis() - lastWaitingLogAt >= WAITING_LOG_INTERVAL_MS) {
      lastWaitingLogAt = millis();
      Serial.println(F("[WAIT] HOME card not detected"));
    }
    return;
  }

  if (!rfid.PICC_ReadCardSerial()) {
    Serial.println(F("[ERROR] Card field detected but UID read failed"));
    return;
  }

  printRegistrationResult();
  rfid.PICC_HaltA();
  rfid.PCD_StopCrypto1();
}
