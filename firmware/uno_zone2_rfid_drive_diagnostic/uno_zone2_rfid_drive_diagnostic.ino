// SensorUno + MotorUno 전용 ZONE2 RFID 주행 진단
//
// 목적:
//   1. HOME의 검은 마커에서 MotorUno 4WD v2 프로토콜과 위치를 동기화
//   2. 네 바퀴로 ZONE2 방향 전진
//   3. 주행 중 등록된 ZONE2 RFID를 읽는 즉시 MotorUno STOP
//
// 이 스케치는 ESP-01, 서버와 DHT22를 사용하지 않는다. ActuatorUno에는
// 부팅 STOP만 보내며 가동 명령은 보내지 않는다. 시험 전 고전력 부하는
// 물리적으로 분리한다.
//
// 시리얼 115200:
//   C = HOME 보정(차량이 HOME 마커 위에서 ZONE2 방향을 보고 있어야 함)
//   G = ZONE2 방향 주행 시작
//   0 또는 X = 즉시 정지
//   ? = 상태 출력

// 안전:
//   - 명령 G 없이는 움직이지 않는다.
//   - ZONE2 이외의 카드도 즉시 정지시킨다.
//   - 20초 안에 카드를 못 찾으면 자동 정지한다.
//   - 400ms마다 KEEPALIVE를 보내며, 링크가 끊기면 MotorUno의 2초
//     watchdog도 독립적으로 네 모터를 정지시킨다.

#include <Arduino.h>
#include <Wire.h>

#include "SoftwareMFRC522.h"
#include "zone2_rfid_drive_config.h"

constexpr byte MOTOR_ADDRESS = 0x08;
constexpr byte ACTUATOR_ADDRESS = 0x09;

constexpr byte MOTOR_STOP = 0;
// 운영 MotorUno의 versioned 4WD 전진값. 구형 값 1은 INVALID로 정지한다.
constexpr byte MOTOR_FORWARD = 0x11;
constexpr byte MOTOR_KEEPALIVE = 5;
constexpr byte MOTOR_HOME_SYNC = 6;
constexpr byte MOTOR_PROTOCOL_SYNC = 7;

constexpr byte STATUS_IDLE = 0;
constexpr byte STATUS_RUNNING = 1;
constexpr byte STATUS_OBSTACLE = 2;
constexpr byte STATUS_STOP_LINE = 3;
constexpr byte STATUS_CALIBRATION_REQUIRED = 7;
constexpr byte STATUS_PROTOCOL_REQUIRED = 8;

constexpr byte ACTUATOR_CONTROL_MAGIC = 0xA5;
constexpr byte ACTUATOR_STOP = 0;

constexpr byte RFID_SS_PIN = 8;
constexpr byte RFID_SCK_PIN = 9;
constexpr byte RFID_MOSI_PIN = 10;
constexpr byte RFID_MISO_PIN = 11;
constexpr byte RFID_RST_PIN = 12;

constexpr unsigned long MOTOR_ACK_TIMEOUT_MS = 800;
constexpr unsigned long KEEPALIVE_INTERVAL_MS = 400;
constexpr unsigned long RFID_SCAN_INTERVAL_MS = 40;
constexpr unsigned long DRIVE_TIMEOUT_MS = 20000;

SoftwareMFRC522 rfid(RFID_SS_PIN, RFID_RST_PIN, RFID_SCK_PIN,
                     RFID_MOSI_PIN, RFID_MISO_PIN);

byte sequenceCounter = 0;
byte actuatorSequenceCounter = 0;
byte activeDriveSequence = 0;
byte lastMotorStatus = 0xFF;
bool homeCalibrated = false;
bool driving = false;
bool rfidReady = false;
unsigned long driveStartedAt = 0;
unsigned long lastKeepaliveAt = 0;
unsigned long lastRfidScanAt = 0;
char uidText[30] = "";

void configureI2c10kHz() {
  // AVR에서 정확한 10kHz: prescaler=4, TWBR=198.
  TWSR = static_cast<byte>((TWSR & ~(_BV(TWPS0) | _BV(TWPS1))) |
                           _BV(TWPS0));
  TWBR = 198;
}

void restoreI2cAfterTimeout() {
  if (!Wire.getWireTimeoutFlag()) return;
  Wire.clearWireTimeoutFlag();
  configureI2c10kHz();
}

byte crc8Atm(const byte* data, byte length) {
  byte crc = 0;
  for (byte index = 0; index < length; ++index) {
    crc ^= data[index];
    for (byte bit = 0; bit < 8; ++bit) {
      crc = (crc & 0x80) ? static_cast<byte>((crc << 1) ^ 0x07)
                         : static_cast<byte>(crc << 1);
    }
  }
  return crc;
}

bool sendActuatorStopOnce() {
  byte frame[4] = {
      ACTUATOR_CONTROL_MAGIC, ++actuatorSequenceCounter, ACTUATOR_STOP, 0};
  frame[3] = crc8Atm(frame, 3);
  Wire.beginTransmission(ACTUATOR_ADDRESS);
  Wire.write(frame, sizeof(frame));
  const byte result = Wire.endTransmission();
  restoreI2cAfterTimeout();
  return result == 0;
}

void stopActuatorBestEffort() {
  bool sent = false;
  for (byte attempt = 0; attempt < 3 && !sent; ++attempt) {
    sent = sendActuatorStopOnce();
  }
  Serial.println(sent ? F("[STOP] ActuatorUno STOP sent")
                      : F("[STOP] ActuatorUno absent or STOP failed"));
}

bool sendMotorFrame(byte command, byte sequence) {
  Wire.beginTransmission(MOTOR_ADDRESS);
  Wire.write(command);
  Wire.write(sequence);
  const byte result = Wire.endTransmission();
  restoreI2cAfterTimeout();
  if (result == 0) return true;

  Serial.print(F("[I2C] TX failed result="));
  Serial.println(result);
  return false;
}

bool readMotorState(byte& status, byte& command, byte& sequence) {
  const byte received =
      Wire.requestFrom(static_cast<int>(MOTOR_ADDRESS), 3);
  restoreI2cAfterTimeout();
  if (received < 3 || Wire.available() < 3) {
    while (Wire.available()) Wire.read();
    return false;
  }
  status = static_cast<byte>(Wire.read());
  command = static_cast<byte>(Wire.read());
  sequence = static_cast<byte>(Wire.read());
  return true;
}

bool expectedStatus(byte command, byte status) {
  if (command == MOTOR_PROTOCOL_SYNC) {
    return status == STATUS_PROTOCOL_REQUIRED;
  }
  if (command == MOTOR_STOP) {
    return status == STATUS_IDLE ||
           status == STATUS_CALIBRATION_REQUIRED ||
           status == STATUS_PROTOCOL_REQUIRED;
  }
  if (command == MOTOR_HOME_SYNC) return status == STATUS_IDLE;
  if (command == MOTOR_FORWARD) {
    return status == STATUS_RUNNING || status == STATUS_OBSTACLE ||
           status == STATUS_STOP_LINE;
  }
  return false;
}

bool sendMotorCommandChecked(byte command) {
  const byte sequence = ++sequenceCounter;
  if (!sendMotorFrame(command, sequence)) return false;

  const unsigned long startedAt = millis();
  while (millis() - startedAt < MOTOR_ACK_TIMEOUT_MS) {
    byte status = 0xFF;
    byte appliedCommand = 0xFF;
    byte appliedSequence = 0xFF;
    if (readMotorState(status, appliedCommand, appliedSequence)) {
      lastMotorStatus = status;
      if (appliedCommand == command && appliedSequence == sequence &&
          expectedStatus(command, status)) {
        if (command == MOTOR_FORWARD) activeDriveSequence = sequence;
        return true;
      }
    }
    delay(10);
  }

  Serial.print(F("[I2C] ACK timeout command="));
  Serial.println(command);
  return false;
}

bool stopMotorConfirmed(const __FlashStringHelper* reason) {
  // 첫 STOP ACK가 유실돼도 세 번 확인한다. 모두 실패하면 MotorUno의
  // keepalive watchdog이 마지막 방어선으로 작동한다.
  bool stopped = false;
  for (byte attempt = 0; attempt < 3 && !stopped; ++attempt) {
    stopped = sendMotorCommandChecked(MOTOR_STOP);
  }
  driving = false;
  Serial.print(F("[STOP] "));
  Serial.print(reason);
  Serial.println(stopped ? F(" / ACK OK") : F(" / ACK FAILED"));
  return stopped;
}

void makeUidText() {
  byte position = 0;
  for (byte index = 0; index < rfid.uid.size && position < 29; ++index) {
    if (rfid.uid.uidByte[index] < 0x10) uidText[position++] = '0';
    char hex[3];
    itoa(rfid.uid.uidByte[index], hex, 16);
    uidText[position++] = static_cast<char>(toupper(hex[0]));
    if (hex[1] && position < 29) {
      uidText[position++] = static_cast<char>(toupper(hex[1]));
    }
    if (index + 1 < rfid.uid.size && position < 29) uidText[position++] = ' ';
  }
  uidText[position] = '\0';
}

bool refreshRfidHealth() {
  const byte version = rfid.PCD_ReadRegister(SoftwareMFRC522::VersionReg);
  rfidReady = version != 0x00 && version != 0xFF;
  Serial.print(F("[RFID] VersionReg=0x"));
  if (version < 0x10) Serial.print('0');
  Serial.print(version, HEX);
  Serial.println(rfidReady ? F(" READY") : F(" ERROR"));
  return rfidReady;
}

void calibrateHome() {
  if (driving) {
    Serial.println(F("[CAL] rejected: stop first"));
    return;
  }

  homeCalibrated = false;
  const bool protocolOk = sendMotorCommandChecked(MOTOR_PROTOCOL_SYNC);
  const bool stopOk = sendMotorCommandChecked(MOTOR_STOP);
  const bool homeOk = protocolOk && stopOk &&
                      sendMotorCommandChecked(MOTOR_HOME_SYNC);
  homeCalibrated = homeOk;
  if (homeOk) {
    Serial.println(F("[CAL] HOME_SYNC OK / ready for G"));
  } else {
    stopMotorConfirmed(F("HOME calibration failed"));
    Serial.println(F("[CAL] put both IR sensors on HOME black marker"));
  }
}

void startZone2Drive() {
  if (driving) {
    Serial.println(F("[DRIVE] already running"));
    return;
  }
  if (!homeCalibrated) {
    Serial.println(F("[DRIVE] rejected: send C at HOME first"));
    return;
  }
  if (!refreshRfidHealth()) {
    stopMotorConfirmed(F("RC522 unavailable"));
    return;
  }
  if (!sendMotorCommandChecked(MOTOR_FORWARD)) {
    stopMotorConfirmed(F("forward start failed"));
    return;
  }

  driving = true;
  driveStartedAt = millis();
  lastKeepaliveAt = millis();
  lastRfidScanAt = 0;
  Serial.println(F("[DRIVE] FORWARD -> present ZONE2 RFID; 0=STOP"));
}

void serviceMotorLink() {
  if (!driving || millis() - lastKeepaliveAt < KEEPALIVE_INTERVAL_MS) return;
  lastKeepaliveAt = millis();

  if (!sendMotorFrame(MOTOR_KEEPALIVE, activeDriveSequence)) {
    stopMotorConfirmed(F("keepalive TX failed"));
    return;
  }

  byte status = 0xFF;
  byte appliedCommand = 0xFF;
  byte appliedSequence = 0xFF;
  if (!readMotorState(status, appliedCommand, appliedSequence) ||
      appliedCommand != MOTOR_FORWARD ||
      appliedSequence != activeDriveSequence ||
      (status != STATUS_RUNNING && status != STATUS_OBSTACLE)) {
    lastMotorStatus = status;
    stopMotorConfirmed(F("motor state fault"));
    return;
  }
  lastMotorStatus = status;
}

void serviceRfid() {
  if (!driving || millis() - lastRfidScanAt < RFID_SCAN_INTERVAL_MS) return;
  lastRfidScanAt = millis();
  if (!rfid.PICC_IsNewCardPresent()) return;
  if (!rfid.PICC_ReadCardSerial()) {
    stopMotorConfirmed(F("RFID UID read failed"));
    return;
  }

  makeUidText();
  const bool isZone2 = strcmp(uidText, ZONE2_RFID_UID) == 0;
  // 로그보다 물리 정지가 먼저다.
  const bool stopped = stopMotorConfirmed(
      isZone2 ? F("ZONE2 RFID detected") : F("unexpected RFID detected"));

  Serial.println(isZone2 ? F("[RFID] ZONE2_MATCH")
                         : F("[RFID] UNEXPECTED_CARD"));
  Serial.println(isZone2 && stopped
                     ? F("[RESULT] ZONE2_CONFIRMED / MOTOR_STOPPED")
                     : F("[RESULT] RFID_STOPPED_NOT_ZONE2_OR_ACK_FAILED"));
  rfid.PICC_HaltA();
  rfid.PCD_StopCrypto1();
}

void printStatus() {
  byte status = 0xFF;
  byte command = 0xFF;
  byte sequence = 0xFF;
  const bool linkOk = readMotorState(status, command, sequence);
  Serial.print(F("[STATUS] calibrated="));
  Serial.print(homeCalibrated ? 1 : 0);
  Serial.print(F(" driving="));
  Serial.print(driving ? 1 : 0);
  Serial.print(F(" rfid="));
  Serial.print(rfidReady ? 1 : 0);
  Serial.print(F(" motor="));
  if (linkOk) {
    Serial.print(status);
    Serial.print('/');
    Serial.print(command);
    Serial.print('/');
    Serial.println(sequence);
  } else {
    Serial.println(F("NO_ACK"));
  }
}

void handleSerial() {
  while (Serial.available()) {
    const char input = static_cast<char>(Serial.read());
    if (input == '0' || input == 'X' || input == 'x') {
      stopMotorConfirmed(F("serial emergency stop"));
    } else if (input == 'C' || input == 'c') {
      calibrateHome();
    } else if (input == 'G' || input == 'g') {
      startZone2Drive();
    } else if (input == '?') {
      printStatus();
    }
  }
}

void setup() {
  Serial.begin(115200);
  Wire.begin();
  configureI2c10kHz();
  Wire.setWireTimeout(25000UL, true);

  // SensorUno가 주행 중 교체되었을 수도 있으므로 사용자 입력을 기다리기
  // 전에 두 출력 보드에 STOP을 먼저 보낸다. Motor ACK 실패 시 watchdog이
  // 마지막 방어선이며 고전력 부하는 이미 물리적으로 분리돼 있어야 한다.
  stopMotorConfirmed(F("diagnostic boot"));
  stopActuatorBestEffort();

  rfid.PCD_Init();
  Serial.println();
  Serial.println(F("[BOOT] ZONE2 RFID drive diagnostic"));
  Serial.println(F("[SAFE] no actuator/server; timeout=20s"));
  Serial.println(F("[COMMAND] C=HOME_SYNC G=GO 0/X=STOP ?=STATUS"));
  refreshRfidHealth();
}

void loop() {
  handleSerial();
  if (!driving) return;

  serviceMotorLink();
  if (!driving) return;
  serviceRfid();
  if (!driving) return;

  if (millis() - driveStartedAt >= DRIVE_TIMEOUT_MS) {
    stopMotorConfirmed(F("20s RFID timeout"));
    Serial.println(F("[RESULT] ZONE2_NOT_FOUND"));
  }
}
