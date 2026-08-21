#include <array>
#include <cstdint>
#include <deque>
#include <iostream>
#include <stdexcept>
#include <string>
#include <vector>

#include "SoftwareMFRC522.h"

namespace {

constexpr byte kCsPin = 8;
constexpr byte kClockPin = 9;
constexpr byte kMosiPin = 10;
constexpr byte kMisoPin = 11;
constexpr byte kResetPin = 12;

constexpr byte kCommandReg = 0x01 << 1;
constexpr byte kComIrqReg = 0x04 << 1;
constexpr byte kDivIrqReg = 0x05 << 1;
constexpr byte kErrorReg = 0x06 << 1;
constexpr byte kFifoDataReg = 0x09 << 1;
constexpr byte kFifoLevelReg = 0x0A << 1;
constexpr byte kControlReg = 0x0C << 1;
constexpr byte kBitFramingReg = 0x0D << 1;
constexpr byte kCrcResultRegH = 0x21 << 1;
constexpr byte kCrcResultRegL = 0x22 << 1;
constexpr byte kVersionReg = 0x37 << 1;

struct Exchange {
  std::vector<byte> request;
  std::vector<byte> response;
  byte txLastBits = 0;
  byte rxLastBits = 0;
  byte error = 0;
  byte irq = 0x20;
};

[[noreturn]] void fail(const std::string& message) {
  throw std::runtime_error(message);
}

void expect(bool condition, const std::string& message) {
  if (!condition) fail(message);
}

std::array<byte, 2> crcA(const std::vector<byte>& data) {
  std::uint16_t crc = 0x6363;
  for (byte value : data) {
    byte ch = static_cast<byte>(value ^ (crc & 0x00FF));
    ch = static_cast<byte>(ch ^ static_cast<byte>(ch << 4));
    crc = static_cast<std::uint16_t>(
        (crc >> 8) ^ (static_cast<std::uint16_t>(ch) << 8) ^
        (static_cast<std::uint16_t>(ch) << 3) ^ (ch >> 4));
  }
  return {static_cast<byte>(crc & 0xFF), static_cast<byte>(crc >> 8)};
}

class FakeMfrc522 {
 public:
  void reset() {
    registers_.fill(0);
    registers_[kVersionReg] = 0x82;
    fifo_.clear();
    exchanges_.clear();
    selected_ = false;
    firstByte_ = true;
    readTransaction_ = false;
    registerAddress_ = 0;
    inputByte_ = 0;
    outputByte_ = 0;
    bitIndex_ = 0;
    protocolError_.clear();
    completedExchanges_ = 0;
    continuedReads_ = 0;
    terminatedReads_ = 0;
  }

  void queue(const Exchange& exchange) { exchanges_.push_back(exchange); }

  void select() {
    selected_ = true;
    firstByte_ = true;
    bitIndex_ = 0;
    inputByte_ = 0;
    outputByte_ = 0;
  }

  void deselect() {
    if (bitIndex_ != 0 && protocolError_.empty()) {
      protocolError_ = "NSS was released in the middle of an SPI byte";
    }
    selected_ = false;
  }

  void clockRising(byte mosi) {
    if (!selected_) return;
    inputByte_ = static_cast<byte>((inputByte_ << 1) | (mosi ? 1 : 0));
  }

  int miso() const {
    if (!selected_) return LOW;
    return (outputByte_ & static_cast<byte>(0x80 >> bitIndex_)) ? HIGH : LOW;
  }

  void clockFalling() {
    if (!selected_) return;
    ++bitIndex_;
    if (bitIndex_ != 8) return;
    processByte(inputByte_);
    bitIndex_ = 0;
    inputByte_ = 0;
  }

  const std::string& protocolError() const { return protocolError_; }
  std::size_t pendingExchanges() const { return exchanges_.size(); }
  std::size_t completedExchanges() const { return completedExchanges_; }
  unsigned continuedReads() const { return continuedReads_; }
  unsigned terminatedReads() const { return terminatedReads_; }

 private:
  byte readRegister(byte address) {
    if (address == kFifoDataReg) {
      if (fifo_.empty()) {
        if (protocolError_.empty()) protocolError_ = "FIFO read underflow";
        return 0;
      }
      const byte result = fifo_.front();
      fifo_.pop_front();
      registers_[kFifoLevelReg] = static_cast<byte>(fifo_.size());
      return result;
    }
    if (address == kFifoLevelReg) {
      return static_cast<byte>(fifo_.size());
    }
    return registers_[address];
  }

  void writeRegister(byte address, byte value) {
    if (address == kComIrqReg || address == kDivIrqReg) {
      registers_[address] = static_cast<byte>(registers_[address] & ~value);
      return;
    }
    if (address == kFifoLevelReg && (value & 0x80)) {
      fifo_.clear();
      registers_[kFifoLevelReg] = 0;
      return;
    }
    if (address == kFifoDataReg) {
      if (fifo_.size() >= SoftwareMFRC522::FIFO_SIZE) {
        registers_[kErrorReg] |= 0x10;
        return;
      }
      fifo_.push_back(value);
      registers_[kFifoLevelReg] = static_cast<byte>(fifo_.size());
      return;
    }

    registers_[address] = value;
    if (address == kCommandReg && value == 0x0F) {
      const byte version = registers_[kVersionReg];
      registers_.fill(0);
      registers_[kVersionReg] = version;
      fifo_.clear();
      return;
    }
    if (address == kCommandReg && value == 0x03) calculateCrc();
    if (address == kBitFramingReg && (value & 0x80) &&
        registers_[kCommandReg] == 0x0C) {
      runExchange();
    }
  }

  void calculateCrc() {
    const std::vector<byte> data(fifo_.begin(), fifo_.end());
    const auto result = crcA(data);
    registers_[kCrcResultRegL] = result[0];
    registers_[kCrcResultRegH] = result[1];
    registers_[kDivIrqReg] |= 0x04;
  }

  void runExchange() {
    if (exchanges_.empty()) {
      if (protocolError_.empty()) protocolError_ = "unexpected RF exchange";
      registers_[kComIrqReg] = 0x01;
      return;
    }
    const Exchange exchange = exchanges_.front();
    exchanges_.pop_front();
    const std::vector<byte> request(fifo_.begin(), fifo_.end());
    if (request != exchange.request && protocolError_.empty()) {
      protocolError_ = "RF request bytes did not match the scripted frame";
    }
    if ((registers_[kBitFramingReg] & 0x07) != exchange.txLastBits &&
        protocolError_.empty()) {
      protocolError_ = "TxLastBits did not match the scripted frame";
    }

    fifo_.assign(exchange.response.begin(), exchange.response.end());
    registers_[kFifoLevelReg] = static_cast<byte>(fifo_.size());
    registers_[kControlReg] = exchange.rxLastBits;
    registers_[kErrorReg] = exchange.error;
    registers_[kComIrqReg] = exchange.irq;
    ++completedExchanges_;
  }

  void processByte(byte value) {
    if (firstByte_) {
      firstByte_ = false;
      readTransaction_ = (value & 0x80) != 0;
      registerAddress_ = static_cast<byte>(value & 0x7E);
      if (readTransaction_) outputByte_ = readRegister(registerAddress_);
      return;
    }

    if (!readTransaction_) {
      writeRegister(registerAddress_, value);
      outputByte_ = 0;
      return;
    }

    const byte continuationAddress =
        static_cast<byte>(0x80 | registerAddress_);
    if (value == continuationAddress) {
      ++continuedReads_;
      outputByte_ = readRegister(registerAddress_);
    } else if (value == 0) {
      ++terminatedReads_;
      outputByte_ = 0;
    } else if (protocolError_.empty()) {
      protocolError_ = "invalid continuation byte during SPI register read";
    }
  }

  std::array<byte, 128> registers_{};
  std::deque<byte> fifo_;
  std::deque<Exchange> exchanges_;
  bool selected_ = false;
  bool firstByte_ = true;
  bool readTransaction_ = false;
  byte registerAddress_ = 0;
  byte inputByte_ = 0;
  byte outputByte_ = 0;
  byte bitIndex_ = 0;
  std::string protocolError_;
  std::size_t completedExchanges_ = 0;
  unsigned continuedReads_ = 0;
  unsigned terminatedReads_ = 0;
};

FakeMfrc522 fakeChip;
std::array<byte, 32> pinValues{};
unsigned long fakeMillis = 0;

Exchange requestExchange(byte error = 0, byte irq = 0x20) {
  return {{0x26}, {0x04, 0x00}, 7, 0, error, irq};
}

std::vector<byte> withCrc(std::vector<byte> frame) {
  const auto crc = crcA(frame);
  frame.push_back(crc[0]);
  frame.push_back(crc[1]);
  return frame;
}

void queueCascadeLevel(byte selectCommand, const std::array<byte, 4>& blockData,
                       byte sak) {
  std::vector<byte> block(blockData.begin(), blockData.end());
  block.push_back(static_cast<byte>(block[0] ^ block[1] ^ block[2] ^ block[3]));
  fakeChip.queue({{selectCommand, 0x20}, block});

  std::vector<byte> select = {selectCommand, 0x70};
  select.insert(select.end(), block.begin(), block.end());
  select = withCrc(select);
  fakeChip.queue({select, withCrc({sak})});
}

void queueUid(const std::vector<byte>& uid) {
  fakeChip.queue(requestExchange());
  if (uid.size() == 4) {
    queueCascadeLevel(0x93, {uid[0], uid[1], uid[2], uid[3]}, 0x08);
  } else if (uid.size() == 7) {
    queueCascadeLevel(0x93, {0x88, uid[0], uid[1], uid[2]}, 0x04);
    queueCascadeLevel(0x95, {uid[3], uid[4], uid[5], uid[6]}, 0x00);
  } else if (uid.size() == 10) {
    queueCascadeLevel(0x93, {0x88, uid[0], uid[1], uid[2]}, 0x04);
    queueCascadeLevel(0x95, {0x88, uid[3], uid[4], uid[5]}, 0x04);
    queueCascadeLevel(0x97, {uid[6], uid[7], uid[8], uid[9]}, 0x00);
  } else {
    fail("test attempted to queue an unsupported UID length");
  }
}

void assertUidRoundTrip(const std::vector<byte>& expectedUid) {
  fakeChip.reset();
  pinValues.fill(LOW);
  fakeMillis = 0;
  queueUid(expectedUid);
  SoftwareMFRC522 reader(kCsPin, kResetPin, kClockPin, kMosiPin, kMisoPin);
  reader.PCD_Init();
  expect(reader.PCD_ReadRegister(SoftwareMFRC522::VersionReg) == 0x82,
         "SPI register address/read failed");
  expect(reader.PICC_IsNewCardPresent(), "REQA should detect the scripted card");
  expect(reader.PICC_ReadCardSerial(), "UID selection should succeed");
  expect(reader.uid.size == expectedUid.size(), "UID size differs");
  for (std::size_t i = 0; i < expectedUid.size(); ++i) {
    expect(reader.uid.uidByte[i] == expectedUid[i], "UID byte differs");
  }
  expect(fakeChip.pendingExchanges() == 0, "not all RF frames were consumed");
  expect(fakeChip.protocolError().empty(), fakeChip.protocolError());
  expect(fakeChip.continuedReads() > 0,
         "multi-byte FIFO reads did not repeat the read address");

  fakeChip.queue({withCrc({0x50, 0x00}), {}, 0, 0, 0, 0x01});
  reader.PICC_HaltA();
  expect(fakeChip.pendingExchanges() == 0, "HALT frame was not transmitted");
  expect(fakeChip.protocolError().empty(), fakeChip.protocolError());
}

void testInvalidCascadeIsRejected() {
  fakeChip.reset();
  pinValues.fill(LOW);
  fakeMillis = 0;
  fakeChip.queue(requestExchange());
  // CT says another cascade level follows, while SAK says the UID is complete.
  queueCascadeLevel(0x93, {0x88, 0x11, 0x22, 0x33}, 0x00);
  SoftwareMFRC522 reader(kCsPin, kResetPin, kClockPin, kMosiPin, kMisoPin);
  reader.PCD_Init();
  expect(reader.PICC_IsNewCardPresent(), "invalid-cascade REQA failed");
  expect(!reader.PICC_ReadCardSerial(),
         "contradictory CT/SAK cascade response must be rejected");
  expect(reader.uid.size == 0 && reader.uid.sak == 0,
         "failed selection must clear partial UID state");
  expect(fakeChip.protocolError().empty(), fakeChip.protocolError());
}

void testIrqAndErrorHandling() {
  {
    fakeChip.reset();
    pinValues.fill(LOW);
    fakeChip.queue(requestExchange(0x08));
    SoftwareMFRC522 reader(kCsPin, kResetPin, kClockPin, kMosiPin, kMisoPin);
    reader.PCD_Init();
    expect(reader.PICC_IsNewCardPresent(),
           "REQA collision should still report card presence");
  }
  {
    fakeChip.reset();
    pinValues.fill(LOW);
    fakeChip.queue(requestExchange(0x01));
    SoftwareMFRC522 reader(kCsPin, kResetPin, kClockPin, kMosiPin, kMisoPin);
    reader.PCD_Init();
    expect(!reader.PICC_IsNewCardPresent(),
           "protocol error must not report a valid card");
  }
  {
    fakeChip.reset();
    pinValues.fill(LOW);
    fakeChip.queue(requestExchange(0, 0x01));
    SoftwareMFRC522 reader(kCsPin, kResetPin, kClockPin, kMosiPin, kMisoPin);
    reader.PCD_Init();
    expect(!reader.PICC_IsNewCardPresent(),
           "timer IRQ must be reported as a timeout");
  }
}

}  // namespace

void pinMode(byte, byte) {}

void digitalWrite(byte pin, byte value) {
  const byte oldValue = pinValues[pin];
  pinValues[pin] = value;
  if (pin == kCsPin && oldValue == HIGH && value == LOW) fakeChip.select();
  if (pin == kCsPin && oldValue == LOW && value == HIGH) fakeChip.deselect();
  if (pin == kClockPin && oldValue == LOW && value == HIGH) {
    fakeChip.clockRising(pinValues[kMosiPin]);
  }
  if (pin == kClockPin && oldValue == HIGH && value == LOW) {
    fakeChip.clockFalling();
  }
}

int digitalRead(byte pin) {
  return pin == kMisoPin ? fakeChip.miso() : pinValues[pin];
}

void delay(unsigned long milliseconds) { fakeMillis += milliseconds; }
void delayMicroseconds(unsigned int) {}
unsigned long millis() { return fakeMillis++; }

int main() {
  try {
    assertUidRoundTrip({0x83, 0x5C, 0x41, 0x06});
    assertUidRoundTrip({0x04, 0x11, 0x22, 0x33, 0x44, 0x55, 0x66});
    assertUidRoundTrip(
        {0x04, 0xA1, 0xA2, 0xA3, 0xA4, 0xA5, 0xA6, 0xA7, 0xA8, 0xA9});
    testInvalidCascadeIsRejected();
    testIrqAndErrorHandling();
    std::cout << "SoftwareMFRC522 host protocol tests: PASS\n";
    return 0;
  } catch (const std::exception& error) {
    std::cerr << "SoftwareMFRC522 host protocol tests: FAIL: " << error.what()
              << '\n';
    return 1;
  }
}
