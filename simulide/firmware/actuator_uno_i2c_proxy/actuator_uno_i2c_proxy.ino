#include <Arduino.h>
#include <Wire.h>

// SimulIDE용 ActuatorUno 프록시 펌웨어.
// 실제 릴레이는 active-low이지만 회로의 LED는 보기 쉽게 HIGH에서 켠다.
// A4/A5의 I2C 주소, 릴레이 명령, 상태 응답과 제습 단계 시간은 운영 코드와
// 동일하다. LCD는 운영 배선처럼 D5(SDA)/D4(SCL)의 별도 Software I2C로
// 실제 I2CToParallel(PCF8574 0x27) + Hd44780 모델을 구동한다.

constexpr byte I2C_ADDRESS = 0x09;

constexpr byte COMMAND_STOP = 0;
constexpr byte COMMAND_HUMIDIFY = 1;
constexpr byte COMMAND_DEHUMIDIFY = 2;

constexpr byte STATUS_IDLE = 0;
constexpr byte STATUS_RUNNING = 1;
constexpr byte STATUS_DONE = 2;
constexpr byte STATUS_ERROR = 3;

constexpr byte CONTROL_FRAME_MAGIC = 0xA5;
constexpr byte CONTROL_FRAME_SIZE = 4;

// SimulIDE R260501 multi-AVR TWI workaround.  Its slave transmitter returns
// only the first byte of an onRequest payload and fills the rest with 0xFF.
// SensorUno therefore selects one logical response byte with 0xF0..0xF5 and
// reads it in a one-byte request.  The reconstructed payload remains the exact
// production [status, command, appliedSeq, displaySeq, flags, CRC8] frame.
constexpr byte STATUS_BYTE_SELECT_BASE = 0xF0;
constexpr byte STATUS_REPLY_SIZE = 6;

constexpr byte DISPLAY_FRAME_MAGIC = 0xD1;
constexpr byte DISPLAY_FRAME_SIZE = 10;
constexpr byte DISPLAY_STATE_IDLE = 0;
constexpr byte DISPLAY_STATE_MOVING = 1;
constexpr byte DISPLAY_STATE_HUMIDIFY = 2;
constexpr byte DISPLAY_STATE_DEHUMIDIFY = 3;
constexpr byte DISPLAY_STATE_DONE = 4;
constexpr byte DISPLAY_STATE_RETURNING = 5;
constexpr byte DISPLAY_STATE_ERROR = 6;
constexpr byte DISPLAY_INPUT_DHT_VALID = 0x01;

constexpr byte DISPLAY_STATUS_VALID = 0x01;
constexpr byte DISPLAY_STATUS_LCD_READY = 0x02;
constexpr byte DISPLAY_STATUS_LCD_ERROR = 0x04;
constexpr byte DISPLAY_STATUS_STALE = 0x08;

constexpr byte HUMIDIFIER_LED = A0;
constexpr byte PELTIER_LED = A1;
constexpr byte FAN_LED = 7;

constexpr byte LCD_SOFT_SDA_PIN = 5;
constexpr byte LCD_SOFT_SCL_PIN = 4;
constexpr byte LCD_PRIMARY_ADDRESS = 0x27;
constexpr byte LCD_FALLBACK_ADDRESS = 0x3F;
constexpr byte LCD_RS = 0x01;
constexpr byte LCD_ENABLE = 0x04;
constexpr byte LCD_BACKLIGHT = 0x08;

constexpr unsigned long TASK_DURATION_MS = 5000;
constexpr unsigned long FAN_PRESTART_MS = 500;
constexpr unsigned long FAN_COOLDOWN_MS = 2000;
constexpr unsigned long DISPLAY_STALE_MS = 30000;
constexpr unsigned long LCD_RETRY_MS = 5000;
constexpr unsigned long SOFT_I2C_TIMEOUT_US = 1000;
constexpr unsigned int SOFT_I2C_HALF_PERIOD_US = 5;

enum DehumidifyStage : byte {
  STAGE_NONE,
  STAGE_FAN_PRESTART,
  STAGE_PELTIER_RUNNING,
  STAGE_FAN_COOLDOWN
};

volatile byte controlMailbox[CONTROL_FRAME_SIZE] = {0};
volatile bool commandPending = false;
volatile byte displayMailbox[DISPLAY_FRAME_SIZE] = {0};
volatile bool displayMailboxPending = false;
volatile byte actuatorStatus = STATUS_IDLE;
volatile byte activeCommand = COMMAND_STOP;
volatile byte appliedSequence = 0;
volatile byte statusReply[STATUS_REPLY_SIZE] = {
    STATUS_IDLE, COMMAND_STOP, 0, 0, DISPLAY_STATUS_STALE, 0};
volatile byte selectedStatusByte = 0;

DehumidifyStage stage = STAGE_NONE;
unsigned long stageStartedAt = 0;

byte lastDisplaySequence = 0;
byte displayState = DISPLAY_STATE_IDLE;
byte displayZone = 0;
int16_t displayTemperatureTenths = 0;
uint16_t displayHumidityTenths = 0;
byte displayInputFlags = 0;
bool displayFrameValid = false;
bool displayDhtValid = false;
bool displayStale = true;
unsigned long lastDisplayAt = 0;

bool lcdReady = false;
bool lcdError = false;
byte lcdAddress = 0;
byte displayStatusFlags = DISPLAY_STATUS_STALE;
unsigned long lastLcdAttemptAt = 0;
char lcdLine[2][17] = {{0}, {0}};
bool lcdRenderPending = false;
byte lcdRenderRow = 0;
byte lcdRenderColumn = 0;
bool lcdCursorPending = true;

void outputsOff() {
  digitalWrite(HUMIDIFIER_LED, LOW);
  digitalWrite(PELTIER_LED, LOW);
  digitalWrite(FAN_LED, LOW);
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

void receiveCommand(int byteCount) {
  if (byteCount <= 0) return;

  if (byteCount == 1 && Wire.available()) {
    const byte value = Wire.read();
    if (value >= STATUS_BYTE_SELECT_BASE &&
        value < STATUS_BYTE_SELECT_BASE + STATUS_REPLY_SIZE) {
      selectedStatusByte = value - STATUS_BYTE_SELECT_BASE;
      while (Wire.available()) Wire.read();
      return;
    }
    while (Wire.available()) Wire.read();
    controlMailbox[0] = 0xFF;
    commandPending = true;
    return;
  }

  if (byteCount == CONTROL_FRAME_SIZE) {
    byte index = 0;
    while (Wire.available() && index < CONTROL_FRAME_SIZE) {
      controlMailbox[index++] = static_cast<byte>(Wire.read());
    }
    while (Wire.available()) Wire.read();
    if (index != CONTROL_FRAME_SIZE) controlMailbox[0] = 0xFF;
    commandPending = true;
    return;
  }

  if (byteCount != DISPLAY_FRAME_SIZE) {
    while (Wire.available()) Wire.read();
    controlMailbox[0] = 0xFF;
    commandPending = true;
    return;
  }

  byte index = 0;
  while (Wire.available() && index < DISPLAY_FRAME_SIZE) {
    displayMailbox[index++] = static_cast<byte>(Wire.read());
  }
  while (Wire.available()) Wire.read();
  if (index == DISPLAY_FRAME_SIZE) {
    displayMailboxPending = true;
    return;
  }
  controlMailbox[0] = 0xFF;
  commandPending = true;
}

void sendStatus() {
  // R260501은 slave request당 첫 바이트만 안정적으로 반환하므로, 직전
  // 0xF0..0xF5 selector가 가리킨 한 바이트만 보낸다.
  const byte index = selectedStatusByte < STATUS_REPLY_SIZE
                         ? selectedStatusByte
                         : 0;
  Wire.write(statusReply[index]);
}

void publish(byte command, byte status, byte sequence) {
  byte reply[STATUS_REPLY_SIZE] = {
      status, command, sequence, lastDisplaySequence, displayStatusFlags, 0};
  reply[5] = crc8Atm(reply, 5);
  const byte savedSreg = SREG;
  noInterrupts();
  activeCommand = command;
  actuatorStatus = status;
  appliedSequence = sequence;
  for (byte index = 0; index < sizeof(reply); ++index) {
    statusReply[index] = reply[index];
  }
  SREG = savedSreg;
}

void refreshReply() {
  publish(activeCommand, actuatorStatus, appliedSequence);
}

void rebuildDisplayStatusFlags() {
  byte next = 0;
  if (displayFrameValid) next |= DISPLAY_STATUS_VALID;
  if (lcdReady) next |= DISPLAY_STATUS_LCD_READY;
  if (lcdError) next |= DISPLAY_STATUS_LCD_ERROR;
  if (displayStale) next |= DISPLAY_STATUS_STALE;
  if (next == displayStatusFlags) return;
  displayStatusFlags = next;
  refreshReply();
}

void applyCommand(byte command, byte sequence) {
  // 같은 sequence/command 재전송은 멱등 처리한다. 같은 sequence에 다른
  // command가 실리면 오래된 충돌 프레임이므로 모든 출력을 끈다.
  if (sequence == appliedSequence) {
    if (command == activeCommand) return;
    outputsOff();
    stage = STAGE_NONE;
    publish(COMMAND_STOP, STATUS_ERROR, appliedSequence);
    return;
  }

  if (command == COMMAND_STOP) {
    outputsOff();
    stage = STAGE_NONE;
    publish(COMMAND_STOP, STATUS_IDLE, sequence);
    Serial.println(F("[I2C 0x09] STOP -> LEDs OFF"));
    return;
  }

  outputsOff();
  stageStartedAt = millis();
  if (command == COMMAND_HUMIDIFY) {
    digitalWrite(HUMIDIFIER_LED, HIGH);
    stage = STAGE_NONE;
    publish(command, STATUS_RUNNING, sequence);
    Serial.println(F("[I2C 0x09] HUMIDIFY -> A0 LED ON"));
  } else if (command == COMMAND_DEHUMIDIFY) {
    digitalWrite(FAN_LED, HIGH);
    stage = STAGE_FAN_PRESTART;
    publish(command, STATUS_RUNNING, sequence);
    Serial.println(F("[I2C 0x09] DEHUMIDIFY -> fan prestart"));
  } else {
    stage = STAGE_NONE;
    publish(COMMAND_STOP, STATUS_ERROR, sequence);
    Serial.println(F("[I2C 0x09] invalid command -> safe OFF"));
  }
}

void serviceCommand() {
  byte frame[CONTROL_FRAME_SIZE];
  const byte savedSreg = SREG;
  noInterrupts();
  const bool pending = commandPending;
  for (byte index = 0; index < CONTROL_FRAME_SIZE; ++index) {
    frame[index] = controlMailbox[index];
  }
  commandPending = false;
  SREG = savedSreg;
  if (!pending) return;
  if (frame[0] != CONTROL_FRAME_MAGIC || crc8Atm(frame, 3) != frame[3]) {
    outputsOff();
    stage = STAGE_NONE;
    publish(COMMAND_STOP, STATUS_ERROR, appliedSequence);
    Serial.println(F("[I2C 0x09] malformed A5/seq/cmd/CRC -> LEDs OFF"));
    return;
  }
  applyCommand(frame[2], frame[1]);
}

void serviceTask(unsigned long now) {
  if (actuatorStatus != STATUS_RUNNING) return;

  if (activeCommand == COMMAND_HUMIDIFY) {
    if (now - stageStartedAt >= TASK_DURATION_MS) {
      outputsOff();
      publish(COMMAND_HUMIDIFY, STATUS_DONE, appliedSequence);
      Serial.println(F("[TASK] HUMIDIFY DONE"));
    }
    return;
  }

  if (activeCommand != COMMAND_DEHUMIDIFY) return;
  if (stage == STAGE_FAN_PRESTART && now - stageStartedAt >= FAN_PRESTART_MS) {
    digitalWrite(PELTIER_LED, HIGH);
    stage = STAGE_PELTIER_RUNNING;
    stageStartedAt = now;
    Serial.println(F("[TASK] peltier ON"));
  } else if (stage == STAGE_PELTIER_RUNNING &&
             now - stageStartedAt >= TASK_DURATION_MS) {
    digitalWrite(PELTIER_LED, LOW);
    stage = STAGE_FAN_COOLDOWN;
    stageStartedAt = now;
    Serial.println(F("[TASK] peltier OFF, fan cooldown"));
  } else if (stage == STAGE_FAN_COOLDOWN &&
             now - stageStartedAt >= FAN_COOLDOWN_MS) {
    outputsOff();
    stage = STAGE_NONE;
    publish(COMMAND_DEHUMIDIFY, STATUS_DONE, appliedSequence);
    Serial.println(F("[TASK] DEHUMIDIFY DONE"));
  }
}

// ---------------------------------------------------------------------------
// D5/D4 LCD Software I2C. HIGH는 출력하지 않고 INPUT_PULLUP으로 놓는다.
// ---------------------------------------------------------------------------

void releaseSoftLine(byte pin) {
  pinMode(pin, INPUT_PULLUP);
}

void pullSoftLineLow(byte pin) {
  digitalWrite(pin, LOW);
  pinMode(pin, OUTPUT);
}

bool waitSoftClockHigh() {
  const unsigned long startedAt = micros();
  while (digitalRead(LCD_SOFT_SCL_PIN) == LOW) {
    if (micros() - startedAt >= SOFT_I2C_TIMEOUT_US) return false;
  }
  return true;
}

bool softStart() {
  releaseSoftLine(LCD_SOFT_SDA_PIN);
  releaseSoftLine(LCD_SOFT_SCL_PIN);
  if (!waitSoftClockHigh() || digitalRead(LCD_SOFT_SDA_PIN) == LOW) return false;
  delayMicroseconds(SOFT_I2C_HALF_PERIOD_US);
  pullSoftLineLow(LCD_SOFT_SDA_PIN);
  delayMicroseconds(SOFT_I2C_HALF_PERIOD_US);
  pullSoftLineLow(LCD_SOFT_SCL_PIN);
  return true;
}

void softStop() {
  pullSoftLineLow(LCD_SOFT_SDA_PIN);
  delayMicroseconds(SOFT_I2C_HALF_PERIOD_US);
  releaseSoftLine(LCD_SOFT_SCL_PIN);
  waitSoftClockHigh();
  delayMicroseconds(SOFT_I2C_HALF_PERIOD_US);
  releaseSoftLine(LCD_SOFT_SDA_PIN);
  delayMicroseconds(SOFT_I2C_HALF_PERIOD_US);
}

bool softWriteByte(byte value) {
  for (byte mask = 0x80; mask; mask >>= 1) {
    if (value & mask) releaseSoftLine(LCD_SOFT_SDA_PIN);
    else pullSoftLineLow(LCD_SOFT_SDA_PIN);
    delayMicroseconds(SOFT_I2C_HALF_PERIOD_US);
    releaseSoftLine(LCD_SOFT_SCL_PIN);
    if (!waitSoftClockHigh()) return false;
    delayMicroseconds(SOFT_I2C_HALF_PERIOD_US);
    pullSoftLineLow(LCD_SOFT_SCL_PIN);
  }

  releaseSoftLine(LCD_SOFT_SDA_PIN);
  delayMicroseconds(SOFT_I2C_HALF_PERIOD_US);
  releaseSoftLine(LCD_SOFT_SCL_PIN);
  if (!waitSoftClockHigh()) return false;
  delayMicroseconds(SOFT_I2C_HALF_PERIOD_US);
  const bool ack = digitalRead(LCD_SOFT_SDA_PIN) == LOW;
  pullSoftLineLow(LCD_SOFT_SCL_PIN);
  return ack;
}

bool softProbe(byte address) {
  if (!softStart()) return false;
  const bool ack = softWriteByte(static_cast<byte>(address << 1));
  softStop();
  return ack;
}

bool softWrite(byte address, const byte* data, byte length) {
  if (!softStart()) return false;
  bool ok = softWriteByte(static_cast<byte>(address << 1));
  for (byte index = 0; ok && index < length; ++index) {
    ok = softWriteByte(data[index]);
  }
  softStop();
  return ok;
}

bool recoverSoftBus() {
  releaseSoftLine(LCD_SOFT_SDA_PIN);
  releaseSoftLine(LCD_SOFT_SCL_PIN);
  if (!waitSoftClockHigh()) return false;
  for (byte pulse = 0;
       pulse < 9 && digitalRead(LCD_SOFT_SDA_PIN) == LOW;
       ++pulse) {
    pullSoftLineLow(LCD_SOFT_SCL_PIN);
    delayMicroseconds(SOFT_I2C_HALF_PERIOD_US);
    releaseSoftLine(LCD_SOFT_SCL_PIN);
    if (!waitSoftClockHigh()) return false;
    delayMicroseconds(SOFT_I2C_HALF_PERIOD_US);
  }
  softStop();
  return digitalRead(LCD_SOFT_SDA_PIN) == HIGH &&
         digitalRead(LCD_SOFT_SCL_PIN) == HIGH;
}

bool lcdWriteNibble(byte nibble, byte mode) {
  const byte value = static_cast<byte>((nibble & 0xF0) | mode | LCD_BACKLIGHT);
  const byte pulse[3] = {value, static_cast<byte>(value | LCD_ENABLE),
                        static_cast<byte>(value & ~LCD_ENABLE)};
  return softWrite(lcdAddress, pulse, sizeof(pulse));
}

bool lcdSend(byte value, byte mode) {
  const byte high = static_cast<byte>((value & 0xF0) | mode | LCD_BACKLIGHT);
  const byte low = static_cast<byte>(((value << 4) & 0xF0) |
                                      mode | LCD_BACKLIGHT);
  const byte pulse[6] = {
      high, static_cast<byte>(high | LCD_ENABLE),
      static_cast<byte>(high & ~LCD_ENABLE), low,
      static_cast<byte>(low | LCD_ENABLE),
      static_cast<byte>(low & ~LCD_ENABLE)};
  return softWrite(lcdAddress, pulse, sizeof(pulse));
}

bool initializeLcd() {
  if (!recoverSoftBus()) return false;
  if (softProbe(LCD_PRIMARY_ADDRESS)) lcdAddress = LCD_PRIMARY_ADDRESS;
  else if (softProbe(LCD_FALLBACK_ADDRESS)) lcdAddress = LCD_FALLBACK_ADDRESS;
  else return false;

  delay(50);
  if (!lcdWriteNibble(0x30, 0)) return false;
  delay(5);
  if (!lcdWriteNibble(0x30, 0)) return false;
  delay(5);
  if (!lcdWriteNibble(0x30, 0)) return false;
  delay(1);
  if (!lcdWriteNibble(0x20, 0)) return false;
  if (!lcdSend(0x28, 0)) return false;
  if (!lcdSend(0x08, 0)) return false;
  if (!lcdSend(0x01, 0)) return false;
  delay(2);
  if (!lcdSend(0x06, 0)) return false;
  return lcdSend(0x0C, 0);
}

void setLine(byte row, const char* text) {
  byte column = 0;
  while (column < 16 && *text) lcdLine[row][column++] = *text++;
  while (column < 16) lcdLine[row][column++] = ' ';
  lcdLine[row][16] = '\0';
}

void formatLcdLines() {
  char line[17];
  if (displayStale) {
    setLine(0, "TELEMETRY STALE");
  } else if (!displayDhtValid) {
    setLine(0, "DHT22 ERROR");
  } else {
    const long temperature = displayTemperatureTenths;
    const unsigned int absoluteTemperature = static_cast<unsigned int>(
        temperature < 0 ? -temperature : temperature);
    if (temperature < 0) {
      snprintf_P(line, sizeof(line), PSTR("T-%u.%uC H%u.%u%%"),
                 absoluteTemperature / 10, absoluteTemperature % 10,
                 displayHumidityTenths / 10, displayHumidityTenths % 10);
    } else {
      snprintf_P(line, sizeof(line), PSTR("T%u.%uC H%u.%u%%"),
                 absoluteTemperature / 10, absoluteTemperature % 10,
                 displayHumidityTenths / 10, displayHumidityTenths % 10);
    }
    setLine(0, line);
  }

  const char* zone = displayZone == 2 ? "ZONE2" :
                     displayZone == 99 ? "ZONE99" :
                     displayZone == 0 ? "HOME" : "NONE";
  switch (displayState) {
    case DISPLAY_STATE_IDLE: snprintf_P(line, 17, PSTR("IDLE %s"), zone); break;
    case DISPLAY_STATE_MOVING: snprintf_P(line, 17, PSTR("MOVE %s"), zone); break;
    case DISPLAY_STATE_HUMIDIFY: snprintf_P(line, 17, PSTR("HUMIDIFY %s"), zone); break;
    case DISPLAY_STATE_DEHUMIDIFY: snprintf_P(line, 17, PSTR("DEHUM %s"), zone); break;
    case DISPLAY_STATE_DONE: snprintf_P(line, 17, PSTR("DONE %s"), zone); break;
    case DISPLAY_STATE_RETURNING: snprintf_P(line, 17, PSTR("RETURN %s"), zone); break;
    default: snprintf_P(line, 17, PSTR("ERROR %s"), zone); break;
  }
  setLine(1, line);
}

void scheduleLcdRender() {
  lcdRenderPending = true;
  lcdRenderRow = 0;
  lcdRenderColumn = 0;
  lcdCursorPending = true;
}

void serviceDisplayMailbox(unsigned long now) {
  if (!displayMailboxPending) return;
  byte frame[DISPLAY_FRAME_SIZE];
  const byte savedSreg = SREG;
  noInterrupts();
  for (byte index = 0; index < DISPLAY_FRAME_SIZE; ++index) {
    frame[index] = displayMailbox[index];
  }
  displayMailboxPending = false;
  SREG = savedSreg;

  const byte state = frame[2];
  const byte zone = frame[3];
  const uint16_t humidity = static_cast<uint16_t>(frame[6]) |
                            (static_cast<uint16_t>(frame[7]) << 8);
  const bool zoneValid = zone == 0 || zone == 2 || zone == 99 || zone == 0xFF;
  if (frame[0] != DISPLAY_FRAME_MAGIC ||
      crc8Atm(frame, 9) != frame[9] ||
      state > DISPLAY_STATE_ERROR || !zoneValid || humidity > 1000) {
    Serial.println(F("[DISPLAY] invalid 10-byte frame discarded; relays unchanged"));
    return;
  }

  lastDisplaySequence = frame[1];
  displayState = state;
  displayZone = zone;
  displayTemperatureTenths = static_cast<int16_t>(
      static_cast<uint16_t>(frame[4]) |
      (static_cast<uint16_t>(frame[5]) << 8));
  displayHumidityTenths = humidity;
  displayInputFlags = frame[8];
  displayDhtValid = (displayInputFlags & DISPLAY_INPUT_DHT_VALID) != 0;
  displayFrameValid = true;
  displayStale = false;
  lastDisplayAt = now;
  formatLcdLines();
  scheduleLcdRender();
  rebuildDisplayStatusFlags();
  refreshReply();

  Serial.print(F("[DISPLAY] accepted seq="));
  Serial.print(lastDisplaySequence);
  Serial.print(F(" state="));
  Serial.print(displayState);
  Serial.print(F(" zone="));
  Serial.println(displayZone);
}

void serviceDisplayStale(unsigned long now) {
  if (!displayFrameValid || displayStale ||
      now - lastDisplayAt < DISPLAY_STALE_MS) return;
  displayStale = true;
  formatLcdLines();
  scheduleLcdRender();
  rebuildDisplayStatusFlags();
  Serial.println(F("[DISPLAY] STALE >30s"));
}

void serviceLcd(unsigned long now) {
  if (!lcdReady) {
    if (lastLcdAttemptAt != 0 && now - lastLcdAttemptAt < LCD_RETRY_MS) return;
    lastLcdAttemptAt = now;
    lcdReady = initializeLcd();
    lcdError = !lcdReady;
    rebuildDisplayStatusFlags();
    if (!lcdReady) {
      Serial.println(F("[LCD] D5/D4 0x27/0x3F not responding; relay continues"));
      return;
    }
    Serial.print(F("[LCD] D5 SDA / D4 SCL ready address=0x"));
    Serial.println(lcdAddress, HEX);
    scheduleLcdRender();
  }

  // 준비 후 한 loop에 cursor 명령 한 번 또는 문자 한 개만 보낸다.
  if (!lcdRenderPending) return;
  if (lcdCursorPending) {
    if (!lcdSend(lcdRenderRow == 0 ? 0x80 : 0xC0, 0)) {
      lcdReady = false;
      lcdError = true;
      lastLcdAttemptAt = now;
      rebuildDisplayStatusFlags();
      return;
    }
    lcdCursorPending = false;
    return;
  }

  if (!lcdSend(static_cast<byte>(lcdLine[lcdRenderRow][lcdRenderColumn]), LCD_RS)) {
    lcdReady = false;
    lcdError = true;
    lastLcdAttemptAt = now;
    rebuildDisplayStatusFlags();
    return;
  }
  if (++lcdRenderColumn < 16) return;
  if (lcdRenderRow == 0) {
    lcdRenderRow = 1;
    lcdRenderColumn = 0;
    lcdCursorPending = true;
  } else {
    lcdRenderPending = false;
  }
}

void setup() {
  Serial.begin(9600);
  pinMode(HUMIDIFIER_LED, OUTPUT);
  pinMode(PELTIER_LED, OUTPUT);
  pinMode(FAN_LED, OUTPUT);
  outputsOff();

  releaseSoftLine(LCD_SOFT_SDA_PIN);
  releaseSoftLine(LCD_SOFT_SCL_PIN);
  formatLcdLines();
  scheduleLcdRender();

  publish(COMMAND_STOP, STATUS_IDLE, 0);
  Wire.begin(I2C_ADDRESS);
  Wire.onReceive(receiveCommand);
  Wire.onRequest(sendStatus);
  Serial.println(F("ActuatorUno proxy ready: A4/A5 slave=0x09"));
  Serial.println(F("Control: A5/seq/cmd/CRC 4B; status logical 6B"));
  Serial.println(F("LCD proxy: D5=SDA D4=SCL, PCF8574=0x27, D1/10B display"));
}

void loop() {
  const unsigned long now = millis();
  // 릴레이 명령과 종료 타이머가 LCD보다 항상 먼저 처리된다.
  serviceCommand();
  serviceTask(now);
  serviceDisplayMailbox(now);
  serviceDisplayStale(now);
  serviceLcd(now);
}
