#include "SoftwareMFRC522.h"

SoftwareMFRC522::SoftwareMFRC522(byte chipSelectPin, byte resetPin,
                                 byte clockPin, byte mosiPin, byte misoPin)
    : chipSelectPin_(chipSelectPin),
      resetPin_(resetPin),
      clockPin_(clockPin),
      mosiPin_(mosiPin),
      misoPin_(misoPin) {
  clearUid();
}

void SoftwareMFRC522::clearUid() {
  uid.size = 0;
  uid.sak = 0;
}

// SPI mode 0, MSB first. digitalWrite 자체 지연이 RC522의 최소 setup/hold
// 시간보다 충분히 길어서 별도의 delayMicroseconds는 필요하지 않다.
byte SoftwareMFRC522::transferByte(byte value) {
  byte received = 0;
  for (byte mask = 0x80; mask; mask >>= 1) {
    digitalWrite(mosiPin_, (value & mask) ? HIGH : LOW);
    digitalWrite(clockPin_, HIGH);
    received <<= 1;
    if (digitalRead(misoPin_)) received |= 1;
    digitalWrite(clockPin_, LOW);
  }
  return received;
}

void SoftwareMFRC522::writeRegister(Register reg, byte value) {
  digitalWrite(chipSelectPin_, LOW);
  transferByte(static_cast<byte>(reg));
  transferByte(value);
  digitalWrite(chipSelectPin_, HIGH);
}

void SoftwareMFRC522::writeRegister(Register reg, byte count,
                                    const byte* values) {
  if (!count) return;
  if (!values) return;
  digitalWrite(chipSelectPin_, LOW);
  transferByte(static_cast<byte>(reg));
  for (byte i = 0; i < count; ++i) transferByte(values[i]);
  digitalWrite(chipSelectPin_, HIGH);
}

byte SoftwareMFRC522::PCD_ReadRegister(Register reg) {
  digitalWrite(chipSelectPin_, LOW);
  transferByte(static_cast<byte>(0x80 | reg));
  const byte value = transferByte(0);
  digitalWrite(chipSelectPin_, HIGH);
  return value;
}

void SoftwareMFRC522::readRegister(Register reg, byte count, byte* values) {
  if (!count) return;
  if (!values) return;
  const byte address = static_cast<byte>(0x80 | reg);
  digitalWrite(chipSelectPin_, LOW);
  transferByte(address);
  for (byte i = 0; i < count; ++i) {
    // 마지막 바이트에서는 주소 대신 0을 보내 연속 읽기를 끝낸다.
    values[i] = transferByte(i + 1 < count ? address : 0);
  }
  digitalWrite(chipSelectPin_, HIGH);
}

void SoftwareMFRC522::setRegisterBits(Register reg, byte mask) {
  writeRegister(reg, static_cast<byte>(PCD_ReadRegister(reg) | mask));
}

void SoftwareMFRC522::clearRegisterBits(Register reg, byte mask) {
  writeRegister(reg, static_cast<byte>(PCD_ReadRegister(reg) & ~mask));
}

void SoftwareMFRC522::PCD_Init() {
  pinMode(chipSelectPin_, OUTPUT);
  digitalWrite(chipSelectPin_, HIGH);
  pinMode(clockPin_, OUTPUT);
  digitalWrite(clockPin_, LOW);
  pinMode(mosiPin_, OUTPUT);
  digitalWrite(mosiPin_, LOW);
  pinMode(misoPin_, INPUT);

  // RST를 확실히 LOW→HIGH로 전환해 전원 인가 상태와 무관하게 초기화한다.
  pinMode(resetPin_, OUTPUT);
  digitalWrite(resetPin_, LOW);
  delayMicroseconds(2);
  digitalWrite(resetPin_, HIGH);
  delay(50);

  writeRegister(CommandReg, PCD_SOFT_RESET);
  byte attempts = 0;
  do {
    delay(50);
  } while ((PCD_ReadRegister(CommandReg) & 0x10) && ++attempts < 3);

  writeRegister(TxModeReg, 0x00);
  writeRegister(RxModeReg, 0x00);
  writeRegister(ModWidthReg, 0x26);
  writeRegister(TModeReg, 0x80);
  writeRegister(TPrescalerReg, 0xA9);
  writeRegister(TReloadRegH, 0x03);
  writeRegister(TReloadRegL, 0xE8);
  writeRegister(TxASKReg, 0x40);
  writeRegister(ModeReg, 0x3D);
  setRegisterBits(TxControlReg, 0x03);
}

SoftwareMFRC522::Status SoftwareMFRC522::calculateCrc(
    const byte* data, byte length, byte* result) {
  // MFRC522 FIFO는 64바이트다. 잘못된 포인터/길이가 들어와도 FIFO나
  // Uno 메모리를 건드리지 않도록 내부 API에서도 경계를 확인한다.
  if (!result || length > FIFO_SIZE || (length && !data)) {
    return STATUS_NO_ROOM;
  }
  writeRegister(CommandReg, PCD_IDLE);
  writeRegister(DivIrqReg, 0x04);
  writeRegister(FIFOLevelReg, 0x80);
  writeRegister(FIFODataReg, length, data);
  writeRegister(CommandReg, PCD_CALC_CRC);

  const unsigned long startedAt = millis();
  while (millis() - startedAt < 90) {
    if (PCD_ReadRegister(DivIrqReg) & 0x04) {
      writeRegister(CommandReg, PCD_IDLE);
      result[0] = PCD_ReadRegister(CRCResultRegL);
      result[1] = PCD_ReadRegister(CRCResultRegH);
      return STATUS_OK;
    }
  }
  return STATUS_TIMEOUT;
}

SoftwareMFRC522::Status SoftwareMFRC522::transceive(
    const byte* sendData, byte sendLength, byte* receiveData,
    byte* receiveLength, byte* validBits) {
  if (sendLength > FIFO_SIZE || (sendLength && !sendData)) {
    return STATUS_NO_ROOM;
  }
  // 수신 버퍼와 길이 포인터는 둘 다 있거나 둘 다 없어야 한다.
  // 둘 다 없는 형태는 PICC_HaltA처럼 응답을 버리는 정상 호출이다.
  if ((receiveData == nullptr) != (receiveLength == nullptr)) {
    return STATUS_ERROR;
  }
  const byte txLastBits = validBits ? *validBits : 0;
  if (txLastBits > 7) return STATUS_ERROR;
  writeRegister(CommandReg, PCD_IDLE);
  writeRegister(ComIrqReg, 0x7F);
  writeRegister(FIFOLevelReg, 0x80);
  writeRegister(FIFODataReg, sendLength, sendData);
  writeRegister(BitFramingReg, txLastBits);
  writeRegister(CommandReg, PCD_TRANSCEIVE);
  setRegisterBits(BitFramingReg, 0x80);

  const unsigned long startedAt = millis();
  bool complete = false;
  while (millis() - startedAt < 36) {
    const byte irq = PCD_ReadRegister(ComIrqReg);
    if (irq & 0x30) {
      complete = true;
      break;
    }
    if (irq & 0x01) return STATUS_TIMEOUT;
  }
  if (!complete) return STATUS_TIMEOUT;

  const byte error = PCD_ReadRegister(ErrorReg);
  if (error & 0x13) return STATUS_ERROR;

  if (receiveData && receiveLength) {
    const byte count = PCD_ReadRegister(FIFOLevelReg);
    if (count > *receiveLength) return STATUS_NO_ROOM;
    *receiveLength = count;
    readRegister(FIFODataReg, count, receiveData);
    if (validBits) *validBits = PCD_ReadRegister(ControlReg) & 0x07;
  }
  if (error & 0x08) return STATUS_COLLISION;
  return STATUS_OK;
}

SoftwareMFRC522::Status SoftwareMFRC522::requestA() {
  byte command = 0x26;
  byte answer[2];
  byte answerLength = sizeof(answer);
  byte validBits = 7;
  clearRegisterBits(CollReg, 0x80);
  const Status status =
      transceive(&command, 1, answer, &answerLength, &validBits);
  if (status != STATUS_OK) return status;
  return answerLength == 2 && validBits == 0 ? STATUS_OK : STATUS_ERROR;
}

bool SoftwareMFRC522::PICC_IsNewCardPresent() {
  writeRegister(TxModeReg, 0x00);
  writeRegister(RxModeReg, 0x00);
  writeRegister(ModWidthReg, 0x26);
  const Status result = requestA();
  return result == STATUS_OK || result == STATUS_COLLISION;
}

SoftwareMFRC522::Status SoftwareMFRC522::selectUid() {
  clearUid();
  clearRegisterBits(CollReg, 0x80);

  for (byte level = 0; level < 3; ++level) {
    const byte selectCommand =
        level == 0 ? 0x93 : (level == 1 ? 0x95 : 0x97);

    // 한 장의 카드가 안테나 앞에 있다는 프로젝트 조건에서 전체 32비트와
    // BCC를 한 번에 요청한다. 여러 카드가 동시에 있으면 안전하게 실패한다.
    byte antiCollision[2] = {selectCommand, 0x20};
    byte block[5] = {0, 0, 0, 0, 0};
    byte blockLength = sizeof(block);
    byte validBits = 0;
    Status status = transceive(antiCollision, sizeof(antiCollision), block,
                               &blockLength, &validBits);
    if (status != STATUS_OK) return status;
    if (blockLength != 5 || validBits != 0) return STATUS_ERROR;
    if ((block[0] ^ block[1] ^ block[2] ^ block[3]) != block[4]) {
      return STATUS_CRC_WRONG;
    }

    const bool hasCascadeTag = block[0] == 0x88;
    // Cascade Level 3에는 더 이어질 UID가 없으므로 CT가 올 수 없다.
    if (level == 2 && hasCascadeTag) return STATUS_ERROR;
    const byte firstUidByte = hasCascadeTag ? 1 : 0;
    const byte bytesToCopy = hasCascadeTag ? 3 : 4;
    if (uid.size + bytesToCopy > sizeof(uid.uidByte)) return STATUS_NO_ROOM;
    for (byte i = 0; i < bytesToCopy; ++i) {
      uid.uidByte[uid.size++] = block[firstUidByte + i];
    }

    byte selectFrame[9];
    selectFrame[0] = selectCommand;
    selectFrame[1] = 0x70;
    for (byte i = 0; i < 5; ++i) selectFrame[i + 2] = block[i];
    status = calculateCrc(selectFrame, 7, &selectFrame[7]);
    if (status != STATUS_OK) return status;

    byte selectAnswer[3] = {0, 0, 0};
    byte answerLength = sizeof(selectAnswer);
    validBits = 0;
    status = transceive(selectFrame, sizeof(selectFrame), selectAnswer,
                        &answerLength, &validBits);
    if (status != STATUS_OK) return status;
    if (answerLength != 3 || validBits != 0) return STATUS_ERROR;

    byte answerCrc[2];
    status = calculateCrc(selectAnswer, 1, answerCrc);
    if (status != STATUS_OK || answerCrc[0] != selectAnswer[1] ||
        answerCrc[1] != selectAnswer[2]) {
      return STATUS_CRC_WRONG;
    }

    uid.sak = selectAnswer[0];
    const bool moreLevels = (uid.sak & 0x04) != 0;
    // CT(0x88)와 SAK의 Cascade bit는 항상 같은 뜻이어야 한다. 이를
    // 확인하지 않으면 손상된 응답을 3/6/9바이트 UID로 오인할 수 있다.
    if (moreLevels != hasCascadeTag) return STATUS_ERROR;
    if (!moreLevels) {
      return (uid.size == 4 || uid.size == 7 || uid.size == 10)
                 ? STATUS_OK
                 : STATUS_ERROR;
    }
  }
  return STATUS_ERROR;
}

bool SoftwareMFRC522::PICC_ReadCardSerial() {
  const Status status = selectUid();
  if (status != STATUS_OK) clearUid();
  return status == STATUS_OK;
}

void SoftwareMFRC522::PICC_HaltA() {
  byte frame[4] = {0x50, 0x00, 0, 0};
  if (calculateCrc(frame, 2, &frame[2]) != STATUS_OK) return;
  // HALT는 정상일 때 응답이 없으므로 결과를 의도적으로 무시한다.
  transceive(frame, sizeof(frame), nullptr, nullptr, nullptr);
}

void SoftwareMFRC522::PCD_StopCrypto1() {
  clearRegisterBits(Status2Reg, 0x08);
}
