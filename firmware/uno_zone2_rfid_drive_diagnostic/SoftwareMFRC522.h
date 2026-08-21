#pragma once

#include <Arduino.h>

// RC522를 Uno의 임의 디지털 핀에 연결하기 위한 작은 소프트웨어 SPI 드라이버다.
// 이 프로젝트에서는 카드의 UID 확인만 필요하므로 MFRC522 라이브러리의
// 읽기/쓰기/인증 기능 전체를 포함하지 않는다. 4/7/10바이트 UID 선택은 지원한다.
class SoftwareMFRC522 {
 public:
  static constexpr byte FIFO_SIZE = 64;

  struct Uid {
    byte size;
    byte uidByte[10];
    byte sak;
  };

  // MFRC522 데이터시트의 SPI 레지스터 주소(주소 비트가 한 칸 이동된 값).
  enum Register : byte {
    CommandReg = 0x01 << 1,
    ComIrqReg = 0x04 << 1,
    DivIrqReg = 0x05 << 1,
    ErrorReg = 0x06 << 1,
    Status2Reg = 0x08 << 1,
    FIFODataReg = 0x09 << 1,
    FIFOLevelReg = 0x0A << 1,
    ControlReg = 0x0C << 1,
    BitFramingReg = 0x0D << 1,
    CollReg = 0x0E << 1,
    ModeReg = 0x11 << 1,
    TxModeReg = 0x12 << 1,
    RxModeReg = 0x13 << 1,
    TxControlReg = 0x14 << 1,
    TxASKReg = 0x15 << 1,
    CRCResultRegH = 0x21 << 1,
    CRCResultRegL = 0x22 << 1,
    ModWidthReg = 0x24 << 1,
    TModeReg = 0x2A << 1,
    TPrescalerReg = 0x2B << 1,
    TReloadRegH = 0x2C << 1,
    TReloadRegL = 0x2D << 1,
    VersionReg = 0x37 << 1
  };

  SoftwareMFRC522(byte chipSelectPin, byte resetPin, byte clockPin,
                  byte mosiPin, byte misoPin);

  void PCD_Init();
  byte PCD_ReadRegister(Register reg);
  bool PICC_IsNewCardPresent();
  bool PICC_ReadCardSerial();
  void PICC_HaltA();
  void PCD_StopCrypto1();

  Uid uid;

 private:
  enum Status : byte {
    STATUS_OK,
    STATUS_ERROR,
    STATUS_COLLISION,
    STATUS_TIMEOUT,
    STATUS_NO_ROOM,
    STATUS_CRC_WRONG
  };

  enum Command : byte {
    PCD_IDLE = 0x00,
    PCD_CALC_CRC = 0x03,
    PCD_TRANSCEIVE = 0x0C,
    PCD_SOFT_RESET = 0x0F
  };

  byte chipSelectPin_;
  byte resetPin_;
  byte clockPin_;
  byte mosiPin_;
  byte misoPin_;

  byte transferByte(byte value);
  void writeRegister(Register reg, byte value);
  void writeRegister(Register reg, byte count, const byte* values);
  void readRegister(Register reg, byte count, byte* values);
  void setRegisterBits(Register reg, byte mask);
  void clearRegisterBits(Register reg, byte mask);
  Status calculateCrc(const byte* data, byte length, byte* result);
  Status transceive(const byte* sendData, byte sendLength, byte* receiveData,
                    byte* receiveLength, byte* validBits = nullptr);
  Status requestA();
  Status selectUid();
  void clearUid();
};
