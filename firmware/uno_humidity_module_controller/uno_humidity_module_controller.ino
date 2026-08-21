#include <Arduino.h>
#include <Wire.h>

// [ActuatorUno / 하드웨어 I2C 슬레이브 주소 0x09]
// SensorUno A4(SDA) -> ActuatorUno A4(SDA)
// SensorUno A5(SCL) -> ActuatorUno A5(SCL)
// 릴레이: 가습기=A0, 펠티어=A1, 냉각팬=D7
// LCD1602 I2C 백팩: SDA=D5, SCL=D4
//
// A4/A5는 SensorUno 명령 수신용 Wire 슬레이브 버스이고 D5/D4는
// 직접 구현한 LCD 전용 Software I2C 버스다. 두 버스는 연결하지 않는다.
// 모든 UNO와 전원 장치의 제어 GND는 반드시 공통으로 연결한다.

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
constexpr byte STATUS_REPLY_SIZE = 6;

// SensorUno와 공유하는 displayState 값.
constexpr byte DISPLAY_STATE_IDLE = 0;
constexpr byte DISPLAY_STATE_MOVING = 1;
constexpr byte DISPLAY_STATE_HUMIDIFY = 2;
constexpr byte DISPLAY_STATE_DEHUMIDIFY = 3;
constexpr byte DISPLAY_STATE_DONE = 4;
constexpr byte DISPLAY_STATE_RETURNING = 5;
constexpr byte DISPLAY_STATE_ERROR = 6;

constexpr byte DISPLAY_FRAME_MAGIC = 0xD1;
constexpr byte DISPLAY_FRAME_SIZE = 10;
constexpr byte DISPLAY_INPUT_VALID = 0x01;
constexpr byte DISPLAY_INPUT_WIFI_READY = 0x02;
constexpr byte DISPLAY_INPUT_TASK_ACTIVE = 0x04;
constexpr byte DISPLAY_INPUT_FAULT = 0x08;

// onRequest 응답의 displayFlags.
constexpr byte DISPLAY_FLAG_VALID = 0x01;
constexpr byte DISPLAY_FLAG_LCD_READY = 0x02;
constexpr byte DISPLAY_FLAG_LCD_ERROR = 0x04;
constexpr byte DISPLAY_FLAG_STALE = 0x08;

constexpr byte HUMIDIFIER_RELAY_PIN = A0;
constexpr byte PELTIER_RELAY_PIN = A1;
constexpr byte COOLING_FAN_RELAY_PIN = 7;

// LCD는 Wire를 사용하지 않고 아래 두 핀에서 open-drain으로 구동한다.
constexpr byte LCD_SOFT_SDA_PIN = 5;
constexpr byte LCD_SOFT_SCL_PIN = 4;
constexpr byte LCD_PRIMARY_ADDRESS = 0x27;
constexpr byte LCD_FALLBACK_ADDRESS = 0x3F;

// 일반적인 PCF8574 LCD 백팩: P0=RS, P1=RW, P2=E, P3=BL,
// P4..P7=D4..D7.
constexpr byte LCD_RS = 0x01;
constexpr byte LCD_ENABLE = 0x04;
constexpr byte LCD_BACKLIGHT = 0x08;

constexpr bool RELAY_ACTIVE_LOW = true;
constexpr unsigned long TASK_DURATION_MS = 5000;
constexpr unsigned long FAN_PRESTART_MS = 500;
constexpr unsigned long FAN_COOLDOWN_MS = 2000;
// ESP-01 AT/HTTP 재접속이 10초를 넘는 동안 불필요하게 STALE로 바뀌지 않게
// SensorUno의 2초 heartbeat를 넉넉히 감싸는 30초로 판정한다.
constexpr unsigned long DISPLAY_STALE_MS = 30000;
constexpr unsigned long LCD_RETRY_MS = 5000;
constexpr unsigned long LCD_POWER_UP_MS = 50;
constexpr unsigned long SOFT_I2C_CLOCK_TIMEOUT_US = 1000;
constexpr unsigned int SOFT_I2C_HALF_PERIOD_US = 5;
constexpr unsigned long WIRE_TIMEOUT_US = 25000UL;

volatile byte controlMailbox[CONTROL_FRAME_SIZE] = {0};
volatile bool commandPending = false;

// ISR에서는 표시 프레임을 별도 메일박스에 복사만 한다.
volatile byte displayMailbox[DISPLAY_FRAME_SIZE] = {0};
volatile bool displayMailboxPending = false;

volatile byte actuatorStatus = STATUS_IDLE;
volatile byte activeCommand = COMMAND_STOP;
volatile byte appliedSequence = 0;

// loop에서 CRC까지 미리 만든 응답이다. onRequest는 이 6바이트만 보낸다.
volatile byte statusReply[STATUS_REPLY_SIZE] = {
    STATUS_IDLE, COMMAND_STOP, 0, 0, DISPLAY_FLAG_STALE, 0};

unsigned long taskStartedAt = 0;
unsigned long stageStartedAt = 0;

enum DehumidifyStage : byte {
  DEHUM_STAGE_NONE,
  DEHUM_STAGE_FAN_PRESTART,
  DEHUM_STAGE_PELTIER_RUNNING,
  DEHUM_STAGE_FAN_COOLDOWN
};
DehumidifyStage dehumidifyStage = DEHUM_STAGE_NONE;

byte lastDisplaySeq = 0;
byte currentDisplayState = DISPLAY_STATE_IDLE;
byte currentZoneCode = 0;
byte currentInputFlags = 0;
int16_t currentTemperatureTenths = 0;
uint16_t currentHumidityTenths = 0;
bool telemetryReceived = false;
// VALID 회신은 DHT 값 자체가 아니라 10바이트 프레임의 magic/CRC가
// 정상적으로 적용됐다는 ACK다. DHT 유효성은 currentInputFlags bit0로
// 별도 보존해 LCD에 DHT22 ERROR를 표시한다.
bool displayFrameValid = false;
bool telemetryDataValid = false;
bool telemetryStale = true;
unsigned long lastTelemetryAt = 0;
unsigned int displayCrcErrorCount = 0;
unsigned int displayMagicErrorCount = 0;
unsigned int displayPayloadErrorCount = 0;

// LCD 오류는 릴레이 STATUS와 독립적이다.
bool lcdReady = false;
bool lcdError = false;
byte lcdAddress = 0;
byte displayFlags = DISPLAY_FLAG_STALE;
char lcdDesiredLine[2][17] = {{0}, {0}};
bool lcdRenderPending = false;
byte lcdRenderRow = 0;
byte lcdRenderColumn = 0;
bool lcdRenderNeedsCursor = true;

enum LcdInitState : byte {
  LCD_INIT_POWER_WAIT,
  LCD_INIT_RECOVER_BUS,
  LCD_INIT_PROBE_27,
  LCD_INIT_PROBE_3F,
  LCD_INIT_WAKE_1,
  LCD_INIT_WAKE_2,
  LCD_INIT_WAKE_3,
  LCD_INIT_SET_4BIT,
  LCD_INIT_FUNCTION_SET,
  LCD_INIT_DISPLAY_OFF,
  LCD_INIT_CLEAR,
  LCD_INIT_ENTRY_MODE,
  LCD_INIT_DISPLAY_ON,
  LCD_INIT_READY,
  LCD_INIT_RETRY_WAIT
};
LcdInitState lcdInitState = LCD_INIT_POWER_WAIT;
unsigned long lcdNextStepAt = 0;
unsigned long lcdFailureAt = 0;

// CRC-8/ATM: poly=0x07, init=0, refin=false, xorout=0.
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

void writeRelay(byte pin, bool on) {
  const byte level = RELAY_ACTIVE_LOW ? (on ? LOW : HIGH)
                                      : (on ? HIGH : LOW);
  digitalWrite(pin, level);
}

void stopAllOutputs() {
  writeRelay(HUMIDIFIER_RELAY_PIN, false);
  writeRelay(PELTIER_RELAY_PIN, false);
  writeRelay(COOLING_FAN_RELAY_PIN, false);
}

// 상태 응답은 main loop에서 CRC까지 완성한 뒤 원자적으로 공개한다.
void publishStatusReply(byte status, byte command, byte sequence) {
  byte nextReply[STATUS_REPLY_SIZE];
  nextReply[0] = status;
  nextReply[1] = command;
  nextReply[2] = sequence;
  nextReply[3] = lastDisplaySeq;
  nextReply[4] = displayFlags;
  nextReply[5] = crc8Atm(nextReply, 5);

  const byte savedSreg = SREG;
  noInterrupts();
  actuatorStatus = status;
  activeCommand = command;
  appliedSequence = sequence;
  for (byte index = 0; index < sizeof(nextReply); ++index) {
    statusReply[index] = nextReply[index];
  }
  SREG = savedSreg;
}

void refreshStatusReply() {
  byte statusSnapshot;
  byte commandSnapshot;
  byte sequenceSnapshot;
  const byte savedSreg = SREG;
  noInterrupts();
  statusSnapshot = actuatorStatus;
  commandSnapshot = activeCommand;
  sequenceSnapshot = appliedSequence;
  SREG = savedSreg;
  publishStatusReply(statusSnapshot, commandSnapshot, sequenceSnapshot);
}

void rebuildDisplayFlags() {
  byte nextFlags = 0;
  if (displayFrameValid) nextFlags |= DISPLAY_FLAG_VALID;
  if (lcdReady) nextFlags |= DISPLAY_FLAG_LCD_READY;
  if (lcdError) nextFlags |= DISPLAY_FLAG_LCD_ERROR;
  if (telemetryStale) nextFlags |= DISPLAY_FLAG_STALE;
  if (nextFlags == displayFlags) return;
  displayFlags = nextFlags;
  refreshStatusReply();
}

void publishActuatorState(byte command, byte status) {
  publishStatusReply(status, command, appliedSequence);
}

// 4바이트는 릴레이 제어, 정확히 10바이트는 LCD 표시 메일박스다.
// 그 외 비어 있지 않은 프레임은 잘못된 명령으로 넘겨 안전 정지한다.
// 이 ISR 안에는 CRC/LCD/Serial/relay/millis 호출이 없다.
void receiveI2cCommand(int byteCount) {
  if (byteCount <= 0) return;

  if (byteCount != CONTROL_FRAME_SIZE && byteCount != DISPLAY_FRAME_SIZE) {
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

// [status, command, appliedSeq, displaySeq, displayFlags, CRC8] 6바이트.
void sendI2cStatus() {
  byte replySnapshot[STATUS_REPLY_SIZE];
  for (byte index = 0; index < sizeof(replySnapshot); ++index) {
    replySnapshot[index] = statusReply[index];
  }
  Wire.write(replySnapshot, sizeof(replySnapshot));
}

void serviceWireTimeout() {
  if (!Wire.getWireTimeoutFlag()) return;

  // A4/A5 명령 버스 고착만 기존 안전 정책대로 출력을 정지한다.
  // D5/D4 LCD 오류는 릴레이에 영향을 주지 않는다.
  stopAllOutputs();
  const byte savedSreg = SREG;
  noInterrupts();
  commandPending = false;
  SREG = savedSreg;
  dehumidifyStage = DEHUM_STAGE_NONE;
  publishActuatorState(COMMAND_STOP, STATUS_ERROR);
  Wire.clearWireTimeoutFlag();
  Serial.println(F("[I2C ERROR] A4/A5 Wire timeout -> all relays OFF"));
}

void startHumidifier(byte sequence) {
  stopAllOutputs();
  dehumidifyStage = DEHUM_STAGE_NONE;
  writeRelay(HUMIDIFIER_RELAY_PIN, true);
  publishStatusReply(STATUS_RUNNING, COMMAND_HUMIDIFY, sequence);
  taskStartedAt = millis();
  Serial.println(F("[ACTUATOR] humidifier relay A0 -> ON"));
}

void startDehumidifier(byte sequence) {
  stopAllOutputs();
  writeRelay(COOLING_FAN_RELAY_PIN, true);
  publishStatusReply(STATUS_RUNNING, COMMAND_DEHUMIDIFY, sequence);
  dehumidifyStage = DEHUM_STAGE_FAN_PRESTART;
  stageStartedAt = millis();
  Serial.println(F("[ACTUATOR] fan D7 prestart -> ON"));
}

void applyCommand(byte command, byte sequence) {
  // 같은 sequence의 재전송은 멱등 처리한다. 같은 sequence에 다른 명령이
  // 실리면 오래되거나 충돌한 프레임이므로 출력을 안전 정지한다.
  if (sequence == appliedSequence) {
    if (command == activeCommand) return;
    stopAllOutputs();
    dehumidifyStage = DEHUM_STAGE_NONE;
    publishActuatorState(COMMAND_STOP, STATUS_ERROR);
    return;
  }

  switch (command) {
    case COMMAND_STOP:
      stopAllOutputs();
      dehumidifyStage = DEHUM_STAGE_NONE;
      publishStatusReply(STATUS_IDLE, COMMAND_STOP, sequence);
      Serial.println(F("[I2C] STOP -> all outputs OFF"));
      break;
    case COMMAND_HUMIDIFY:
      startHumidifier(sequence);
      break;
    case COMMAND_DEHUMIDIFY:
      startDehumidifier(sequence);
      break;
    default:
      stopAllOutputs();
      dehumidifyStage = DEHUM_STAGE_NONE;
      publishStatusReply(STATUS_ERROR, COMMAND_STOP, sequence);
      Serial.print(F("[I2C] unknown/malformed command="));
      Serial.println(command);
      break;
  }
}

void completeTask() {
  stopAllOutputs();
  dehumidifyStage = DEHUM_STAGE_NONE;
  publishActuatorState(activeCommand, STATUS_DONE);
  Serial.println(F("[ACTUATOR] task complete -> all outputs OFF"));
}

void serviceCommandMailbox() {
  if (!commandPending) return;
  byte frame[CONTROL_FRAME_SIZE];
  const byte savedSreg = SREG;
  noInterrupts();
  for (byte index = 0; index < sizeof(frame); ++index) {
    frame[index] = controlMailbox[index];
  }
  commandPending = false;
  SREG = savedSreg;
  if (frame[0] != CONTROL_FRAME_MAGIC || crc8Atm(frame, 3) != frame[3]) {
    stopAllOutputs();
    dehumidifyStage = DEHUM_STAGE_NONE;
    publishActuatorState(COMMAND_STOP, STATUS_ERROR);
    Serial.println(F("[I2C] malformed control frame -> outputs OFF"));
    return;
  }
  applyCommand(frame[2], frame[1]);
}

// LCD보다 먼저 호출되는 릴레이 시간 상태 머신.
void serviceActuatorTask() {
  if (actuatorStatus != STATUS_RUNNING) return;
  const unsigned long now = millis();

  if (activeCommand == COMMAND_DEHUMIDIFY) {
    if (dehumidifyStage == DEHUM_STAGE_FAN_PRESTART &&
        now - stageStartedAt >= FAN_PRESTART_MS) {
      writeRelay(PELTIER_RELAY_PIN, true);
      dehumidifyStage = DEHUM_STAGE_PELTIER_RUNNING;
      taskStartedAt = now;
      Serial.println(F("[ACTUATOR] peltier A1 -> ON"));
    } else if (dehumidifyStage == DEHUM_STAGE_PELTIER_RUNNING &&
               now - taskStartedAt >= TASK_DURATION_MS) {
      writeRelay(PELTIER_RELAY_PIN, false);
      dehumidifyStage = DEHUM_STAGE_FAN_COOLDOWN;
      stageStartedAt = now;
      Serial.println(F("[ACTUATOR] peltier OFF; fan cooldown"));
    } else if (dehumidifyStage == DEHUM_STAGE_FAN_COOLDOWN &&
               now - stageStartedAt >= FAN_COOLDOWN_MS) {
      completeTask();
    }
  } else if (now - taskStartedAt >= TASK_DURATION_MS) {
    completeTask();
  }
}

// ---------------------------------------------------------------------------
// LCD 전용 open-drain Software I2C (D5=SDA, D4=SCL)
// ---------------------------------------------------------------------------

void releaseSoftI2cLine(byte pin) {
  pinMode(pin, INPUT_PULLUP);  // HIGH 직접 출력 대신 선을 놓는다.
}

void pullSoftI2cLineLow(byte pin) {
  digitalWrite(pin, LOW);      // LOW만 능동 출력한다.
  pinMode(pin, OUTPUT);
}

bool waitForSoftSclHigh() {
  const unsigned long startedAt = micros();
  while (digitalRead(LCD_SOFT_SCL_PIN) == LOW) {
    if (micros() - startedAt >= SOFT_I2C_CLOCK_TIMEOUT_US) return false;
  }
  return true;
}

bool softI2cStart() {
  releaseSoftI2cLine(LCD_SOFT_SDA_PIN);
  releaseSoftI2cLine(LCD_SOFT_SCL_PIN);
  if (!waitForSoftSclHigh() || digitalRead(LCD_SOFT_SDA_PIN) == LOW) return false;
  delayMicroseconds(SOFT_I2C_HALF_PERIOD_US);
  pullSoftI2cLineLow(LCD_SOFT_SDA_PIN);
  delayMicroseconds(SOFT_I2C_HALF_PERIOD_US);
  pullSoftI2cLineLow(LCD_SOFT_SCL_PIN);
  return true;
}

void softI2cStop() {
  pullSoftI2cLineLow(LCD_SOFT_SDA_PIN);
  delayMicroseconds(SOFT_I2C_HALF_PERIOD_US);
  releaseSoftI2cLine(LCD_SOFT_SCL_PIN);
  waitForSoftSclHigh();
  delayMicroseconds(SOFT_I2C_HALF_PERIOD_US);
  releaseSoftI2cLine(LCD_SOFT_SDA_PIN);
  delayMicroseconds(SOFT_I2C_HALF_PERIOD_US);
}

bool softI2cWriteByte(byte value) {
  for (byte mask = 0x80; mask; mask >>= 1) {
    if (value & mask) releaseSoftI2cLine(LCD_SOFT_SDA_PIN);
    else pullSoftI2cLineLow(LCD_SOFT_SDA_PIN);

    delayMicroseconds(SOFT_I2C_HALF_PERIOD_US);
    releaseSoftI2cLine(LCD_SOFT_SCL_PIN);
    if (!waitForSoftSclHigh()) {
      pullSoftI2cLineLow(LCD_SOFT_SCL_PIN);
      releaseSoftI2cLine(LCD_SOFT_SDA_PIN);
      return false;
    }
    delayMicroseconds(SOFT_I2C_HALF_PERIOD_US);
    pullSoftI2cLineLow(LCD_SOFT_SCL_PIN);
  }

  // 아홉 번째 클럭에서 수신 장치의 ACK(LOW)를 읽는다.
  releaseSoftI2cLine(LCD_SOFT_SDA_PIN);
  delayMicroseconds(SOFT_I2C_HALF_PERIOD_US);
  releaseSoftI2cLine(LCD_SOFT_SCL_PIN);
  if (!waitForSoftSclHigh()) {
    pullSoftI2cLineLow(LCD_SOFT_SCL_PIN);
    return false;
  }
  delayMicroseconds(SOFT_I2C_HALF_PERIOD_US);
  const bool acknowledged = digitalRead(LCD_SOFT_SDA_PIN) == LOW;
  pullSoftI2cLineLow(LCD_SOFT_SCL_PIN);
  return acknowledged;
}

bool softI2cProbe(byte address) {
  if (!softI2cStart()) return false;
  const bool acknowledged = softI2cWriteByte(static_cast<byte>(address << 1));
  softI2cStop();
  return acknowledged;
}

bool softI2cWrite(byte address, const byte* data, byte length) {
  if (!softI2cStart()) return false;
  bool ok = softI2cWriteByte(static_cast<byte>(address << 1));
  for (byte index = 0; ok && index < length; ++index) {
    ok = softI2cWriteByte(data[index]);
  }
  softI2cStop();
  return ok;
}

// SDA가 LOW에 묶였으면 최대 9개 SCL 펄스 후 STOP을 만든다.
bool recoverSoftI2cBus() {
  releaseSoftI2cLine(LCD_SOFT_SDA_PIN);
  releaseSoftI2cLine(LCD_SOFT_SCL_PIN);
  if (!waitForSoftSclHigh()) return false;

  for (byte pulse = 0;
       pulse < 9 && digitalRead(LCD_SOFT_SDA_PIN) == LOW;
       ++pulse) {
    pullSoftI2cLineLow(LCD_SOFT_SCL_PIN);
    delayMicroseconds(SOFT_I2C_HALF_PERIOD_US);
    releaseSoftI2cLine(LCD_SOFT_SCL_PIN);
    if (!waitForSoftSclHigh()) return false;
    delayMicroseconds(SOFT_I2C_HALF_PERIOD_US);
  }
  softI2cStop();
  return digitalRead(LCD_SOFT_SDA_PIN) == HIGH &&
         digitalRead(LCD_SOFT_SCL_PIN) == HIGH;
}

bool lcdWriteNibble(byte highNibble, byte mode) {
  const byte value = static_cast<byte>((highNibble & 0xF0) |
                                        mode | LCD_BACKLIGHT);
  const byte pulse[3] = {value, static_cast<byte>(value | LCD_ENABLE),
                        static_cast<byte>(value & ~LCD_ENABLE)};
  return softI2cWrite(lcdAddress, pulse, sizeof(pulse));
}

bool lcdSendByte(byte value, byte mode) {
  const byte high = static_cast<byte>((value & 0xF0) | mode | LCD_BACKLIGHT);
  const byte low = static_cast<byte>(((value << 4) & 0xF0) |
                                      mode | LCD_BACKLIGHT);
  const byte pulse[6] = {
      high, static_cast<byte>(high | LCD_ENABLE),
      static_cast<byte>(high & ~LCD_ENABLE), low,
      static_cast<byte>(low | LCD_ENABLE),
      static_cast<byte>(low & ~LCD_ENABLE)};
  return softI2cWrite(lcdAddress, pulse, sizeof(pulse));
}

bool timeReached(unsigned long now, unsigned long deadline) {
  return static_cast<long>(now - deadline) >= 0;
}

void setPaddedLine(byte row, const char* text) {
  byte index = 0;
  while (index < 16 && *text) lcdDesiredLine[row][index++] = *text++;
  while (index < 16) lcdDesiredLine[row][index++] = ' ';
  lcdDesiredLine[row][16] = '\0';
}

void formatDisplayLines() {
  char line[17];
  if (telemetryStale) {
    setPaddedLine(0, "TELEMETRY STALE");
  } else if (!telemetryDataValid) {
    setPaddedLine(0, "DHT22 ERROR");
  } else {
    const long temperature = currentTemperatureTenths;
    const unsigned int absoluteTemperature = static_cast<unsigned int>(
        temperature < 0 ? -temperature : temperature);
    if (temperature < 0) {
      snprintf_P(line, sizeof(line), PSTR("T:-%u.%uC H:%u.%u%%"),
                 absoluteTemperature / 10, absoluteTemperature % 10,
                 currentHumidityTenths / 10, currentHumidityTenths % 10);
    } else {
      snprintf_P(line, sizeof(line), PSTR("T:%u.%uC H:%u.%u%%"),
                 absoluteTemperature / 10, absoluteTemperature % 10,
                 currentHumidityTenths / 10, currentHumidityTenths % 10);
    }
    setPaddedLine(0, line);
  }

  const char* zone = "NONE";
  if (currentZoneCode == 0) zone = "HOME";
  else if (currentZoneCode == 2) zone = "ZONE2";
  else if (currentZoneCode == 99) zone = "ZONE99";

  if (currentInputFlags & DISPLAY_INPUT_FAULT) {
    snprintf_P(line, sizeof(line), PSTR("ERROR %s"), zone);
  } else if (!(currentInputFlags & DISPLAY_INPUT_WIFI_READY)) {
    strcpy_P(line, PSTR("WIFI OFFLINE"));
  } else switch (currentDisplayState) {
    case DISPLAY_STATE_IDLE:
      snprintf_P(line, sizeof(line), PSTR("IDLE %s"), zone);
      break;
    case DISPLAY_STATE_MOVING:
      snprintf_P(line, sizeof(line), PSTR("MOVE %s"), zone);
      break;
    case DISPLAY_STATE_HUMIDIFY:
      snprintf_P(line, sizeof(line), PSTR("HUMIDIFY %s"), zone);
      break;
    case DISPLAY_STATE_DEHUMIDIFY:
      snprintf_P(line, sizeof(line), PSTR("DEHUM %s"), zone);
      break;
    case DISPLAY_STATE_DONE:
      snprintf_P(line, sizeof(line), PSTR("DONE %s"), zone);
      break;
    case DISPLAY_STATE_RETURNING:
      snprintf_P(line, sizeof(line), PSTR("RETURN %s"), zone);
      break;
    default:
      snprintf_P(line, sizeof(line), PSTR("ERROR %s"), zone);
      break;
  }
  setPaddedLine(1, line);
}

void scheduleFullLcdRender() {
  lcdRenderPending = true;
  lcdRenderRow = 0;
  lcdRenderColumn = 0;
  lcdRenderNeedsCursor = true;
}

void markLcdFailure(const __FlashStringHelper* reason) {
  lcdReady = false;
  lcdError = true;
  lcdAddress = 0;
  lcdRenderPending = false;
  lcdInitState = LCD_INIT_RETRY_WAIT;
  lcdFailureAt = millis();
  releaseSoftI2cLine(LCD_SOFT_SDA_PIN);
  releaseSoftI2cLine(LCD_SOFT_SCL_PIN);
  rebuildDisplayFlags();
  Serial.print(F("[LCD ERROR] "));
  Serial.print(reason);
  Serial.println(F("; relay control continues, retry in 5s"));
}

void finishLcdInitialization() {
  lcdReady = true;
  lcdError = false;
  lcdInitState = LCD_INIT_READY;
  rebuildDisplayFlags();
  scheduleFullLcdRender();
  Serial.print(F("[LCD] Software I2C ready: D5=SDA D4=SCL address=0x"));
  Serial.println(lcdAddress, HEX);
}

// 호출 한 번당 초기화 한 단계 또는 화면 한 글자만 보낸다.
void serviceLcd() {
  const unsigned long now = millis();
  switch (lcdInitState) {
    case LCD_INIT_POWER_WAIT:
      if (!timeReached(now, lcdNextStepAt)) return;
      lcdInitState = LCD_INIT_RECOVER_BUS;
      return;
    case LCD_INIT_RECOVER_BUS:
      if (!recoverSoftI2cBus()) markLcdFailure(F("D5/D4 bus stuck"));
      else lcdInitState = LCD_INIT_PROBE_27;
      return;
    case LCD_INIT_PROBE_27:
      if (softI2cProbe(LCD_PRIMARY_ADDRESS)) {
        lcdAddress = LCD_PRIMARY_ADDRESS;
        lcdInitState = LCD_INIT_WAKE_1;
      } else {
        lcdInitState = LCD_INIT_PROBE_3F;
      }
      return;
    case LCD_INIT_PROBE_3F:
      if (softI2cProbe(LCD_FALLBACK_ADDRESS)) {
        lcdAddress = LCD_FALLBACK_ADDRESS;
        lcdInitState = LCD_INIT_WAKE_1;
      } else {
        markLcdFailure(F("0x27/0x3F no ACK"));
      }
      return;
    case LCD_INIT_WAKE_1:
      if (!lcdWriteNibble(0x30, 0)) markLcdFailure(F("wake step 1 NACK"));
      else {
        lcdNextStepAt = now + 5;
        lcdInitState = LCD_INIT_WAKE_2;
      }
      return;
    case LCD_INIT_WAKE_2:
      if (!timeReached(now, lcdNextStepAt)) return;
      if (!lcdWriteNibble(0x30, 0)) markLcdFailure(F("wake step 2 NACK"));
      else {
        lcdNextStepAt = now + 5;
        lcdInitState = LCD_INIT_WAKE_3;
      }
      return;
    case LCD_INIT_WAKE_3:
      if (!timeReached(now, lcdNextStepAt)) return;
      if (!lcdWriteNibble(0x30, 0)) markLcdFailure(F("wake step 3 NACK"));
      else {
        lcdNextStepAt = now + 1;
        lcdInitState = LCD_INIT_SET_4BIT;
      }
      return;
    case LCD_INIT_SET_4BIT:
      if (!timeReached(now, lcdNextStepAt)) return;
      if (!lcdWriteNibble(0x20, 0)) markLcdFailure(F("4-bit mode NACK"));
      else lcdInitState = LCD_INIT_FUNCTION_SET;
      return;
    case LCD_INIT_FUNCTION_SET:
      if (!lcdSendByte(0x28, 0)) markLcdFailure(F("function set NACK"));
      else lcdInitState = LCD_INIT_DISPLAY_OFF;
      return;
    case LCD_INIT_DISPLAY_OFF:
      if (!lcdSendByte(0x08, 0)) markLcdFailure(F("display off NACK"));
      else lcdInitState = LCD_INIT_CLEAR;
      return;
    case LCD_INIT_CLEAR:
      if (!lcdSendByte(0x01, 0)) markLcdFailure(F("clear NACK"));
      else {
        lcdNextStepAt = now + 2;
        lcdInitState = LCD_INIT_ENTRY_MODE;
      }
      return;
    case LCD_INIT_ENTRY_MODE:
      if (!timeReached(now, lcdNextStepAt)) return;
      if (!lcdSendByte(0x06, 0)) markLcdFailure(F("entry mode NACK"));
      else lcdInitState = LCD_INIT_DISPLAY_ON;
      return;
    case LCD_INIT_DISPLAY_ON:
      if (!lcdSendByte(0x0C, 0)) markLcdFailure(F("display on NACK"));
      else lcdInitState = LCD_INIT_READY;
      return;
    case LCD_INIT_RETRY_WAIT:
      if (now - lcdFailureAt >= LCD_RETRY_MS) lcdInitState = LCD_INIT_RECOVER_BUS;
      return;
    case LCD_INIT_READY:
      if (!lcdReady) {
        finishLcdInitialization();
        return;
      }
      break;
  }

  if (!lcdRenderPending) return;
  if (lcdRenderNeedsCursor) {
    const byte addressCommand = lcdRenderRow == 0 ? 0x80 : 0xC0;
    if (!lcdSendByte(addressCommand, 0)) markLcdFailure(F("cursor write NACK"));
    else lcdRenderNeedsCursor = false;
    return;
  }

  if (!lcdSendByte(static_cast<byte>(
                       lcdDesiredLine[lcdRenderRow][lcdRenderColumn]), LCD_RS)) {
    markLcdFailure(F("character write NACK"));
    return;
  }
  ++lcdRenderColumn;
  if (lcdRenderColumn < 16) return;
  if (lcdRenderRow == 0) {
    lcdRenderRow = 1;
    lcdRenderColumn = 0;
    lcdRenderNeedsCursor = true;
  } else {
    lcdRenderPending = false;
  }
}

void serviceDisplayMailbox() {
  if (!displayMailboxPending) return;
  byte frame[DISPLAY_FRAME_SIZE];
  const byte savedSreg = SREG;
  noInterrupts();
  for (byte index = 0; index < DISPLAY_FRAME_SIZE; ++index) {
    frame[index] = displayMailbox[index];
  }
  displayMailboxPending = false;
  SREG = savedSreg;

  if (frame[0] != DISPLAY_FRAME_MAGIC) {
    ++displayMagicErrorCount;
    Serial.print(F("[DISPLAY ERROR] wrong magic=0x"));
    Serial.println(frame[0], HEX);
    return;
  }
  const byte calculatedCrc = crc8Atm(frame, DISPLAY_FRAME_SIZE - 1);
  if (calculatedCrc != frame[DISPLAY_FRAME_SIZE - 1]) {
    ++displayCrcErrorCount;
    Serial.print(F("[DISPLAY ERROR] CRC received=0x"));
    Serial.print(frame[DISPLAY_FRAME_SIZE - 1], HEX);
    Serial.print(F(" calculated=0x"));
    Serial.println(calculatedCrc, HEX);
    return;
  }

  const byte nextState = frame[2];
  const byte nextZone = frame[3];
  const uint16_t nextHumidityTenths = static_cast<uint16_t>(frame[6]) |
                                      (static_cast<uint16_t>(frame[7]) << 8);
  const bool zoneValid = nextZone == 0 || nextZone == 2 ||
                         nextZone == 99 || nextZone == 0xFF;
  if (nextState > DISPLAY_STATE_ERROR || !zoneValid ||
      nextHumidityTenths > 1000) {
    // payload 오류는 표시 프레임만 폐기한다. 릴레이 명령/상태/타이머는
    // 절대로 바꾸지 않으며 seq ACK도 갱신하지 않아 SensorUno가 재전송한다.
    ++displayPayloadErrorCount;
    Serial.print(F("[DISPLAY ERROR] invalid payload state="));
    Serial.print(nextState);
    Serial.print(F(" zone="));
    Serial.print(nextZone);
    Serial.print(F(" humidity_tenths="));
    Serial.println(nextHumidityTenths);
    return;
  }

  lastDisplaySeq = frame[1];
  currentDisplayState = nextState;
  currentZoneCode = nextZone;
  currentTemperatureTenths = static_cast<int16_t>(
      static_cast<uint16_t>(frame[4]) |
      (static_cast<uint16_t>(frame[5]) << 8));
  currentHumidityTenths = nextHumidityTenths;
  currentInputFlags = frame[8];
  telemetryDataValid = (currentInputFlags & DISPLAY_INPUT_VALID) != 0;
  telemetryReceived = true;
  displayFrameValid = true;
  telemetryStale = false;
  lastTelemetryAt = millis();

  formatDisplayLines();
  scheduleFullLcdRender();
  rebuildDisplayFlags();
  refreshStatusReply();  // flags가 같아도 seq ACK는 새로 공개한다.

  Serial.print(F("[DISPLAY] seq="));
  Serial.print(lastDisplaySeq);
  Serial.print(F(" state="));
  Serial.print(currentDisplayState);
  Serial.print(F(" zone="));
  Serial.print(currentZoneCode);
  Serial.print(F(" T="));
  Serial.print(currentTemperatureTenths / 10);
  Serial.print('.');
  Serial.print(abs(currentTemperatureTenths % 10));
  Serial.print(F(" H="));
  Serial.print(currentHumidityTenths / 10);
  Serial.print('.');
  Serial.print(currentHumidityTenths % 10);
  Serial.print(F(" input_flags=0x"));
  Serial.println(currentInputFlags, HEX);
}

void serviceDisplayStaleness() {
  if (!telemetryReceived || telemetryStale) return;
  if (millis() - lastTelemetryAt < DISPLAY_STALE_MS) return;
  telemetryStale = true;
  formatDisplayLines();
  scheduleFullLcdRender();
  rebuildDisplayFlags();
  Serial.println(F("[DISPLAY] telemetry stale (>30s)"));
}

// 시리얼 모니터 9600 baud에서 1/2/0/?로 단독 점검한다.
void handleSerialTestCommand() {
  if (!Serial.available()) return;
  const char command = static_cast<char>(Serial.read());

  if (command == '1') {
    Serial.println(F("[SERIAL TEST] 1 -> humidifier relay A0 ON"));
    startHumidifier(appliedSequence);
  } else if (command == '2') {
    Serial.println(F("[SERIAL TEST] 2 -> peltier A1 + fan relay D7 ON"));
    startDehumidifier(appliedSequence);
  } else if (command == '0') {
    Serial.println(F("[SERIAL TEST] 0 -> all outputs OFF"));
    stopAllOutputs();
    dehumidifyStage = DEHUM_STAGE_NONE;
    publishActuatorState(COMMAND_STOP, STATUS_IDLE);
  } else if (command == '?') {
    byte commandSnapshot;
    byte statusSnapshot;
    byte replySnapshot[STATUS_REPLY_SIZE];
    const byte savedSreg = SREG;
    noInterrupts();
    commandSnapshot = activeCommand;
    statusSnapshot = actuatorStatus;
    for (byte index = 0; index < sizeof(replySnapshot); ++index) {
      replySnapshot[index] = statusReply[index];
    }
    SREG = savedSreg;

    Serial.println(F("[SERIAL TEST] 1=humidifier 2=peltier+fan 0=off ?=diagnostics"));
    Serial.print(F("[DIAG ACTUATOR] command="));
    Serial.print(commandSnapshot);
    Serial.print(F(" status="));
    Serial.print(statusSnapshot);
    Serial.print(F(" humidifier_A0="));
    Serial.print(digitalRead(HUMIDIFIER_RELAY_PIN) == LOW ? F("ON") : F("OFF"));
    Serial.print(F(" peltier_A1="));
    Serial.print(digitalRead(PELTIER_RELAY_PIN) == LOW ? F("ON") : F("OFF"));
    Serial.print(F(" fan_D7="));
    Serial.println(digitalRead(COOLING_FAN_RELAY_PIN) == LOW ? F("ON") : F("OFF"));

    Serial.print(F("[DIAG DISPLAY] seq="));
    Serial.print(lastDisplaySeq);
    Serial.print(F(" state="));
    Serial.print(currentDisplayState);
    Serial.print(F(" zone="));
    Serial.print(currentZoneCode);
    Serial.print(F(" flags=0x"));
    Serial.print(displayFlags, HEX);
    Serial.print(F(" age_ms="));
    if (telemetryReceived) Serial.println(millis() - lastTelemetryAt);
    else Serial.println(F("NONE"));

    Serial.print(F("[DIAG LCD] ready="));
    Serial.print(lcdReady ? F("YES") : F("NO"));
    Serial.print(F(" error="));
    Serial.print(lcdError ? F("YES") : F("NO"));
    Serial.print(F(" address=0x"));
    Serial.print(lcdAddress, HEX);
    Serial.print(F(" init_state="));
    Serial.print(static_cast<byte>(lcdInitState));
    Serial.print(F(" D5_SDA="));
    Serial.print(digitalRead(LCD_SOFT_SDA_PIN));
    Serial.print(F(" D4_SCL="));
    Serial.println(digitalRead(LCD_SOFT_SCL_PIN));

    Serial.print(F("[DIAG DISPLAY ERRORS] crc="));
    Serial.print(displayCrcErrorCount);
    Serial.print(F(" magic="));
    Serial.print(displayMagicErrorCount);
    Serial.print(F(" payload="));
    Serial.println(displayPayloadErrorCount);
    Serial.print(F("[DIAG I2C REPLY] "));
    for (byte index = 0; index < sizeof(replySnapshot); ++index) {
      if (index) Serial.print(' ');
      if (replySnapshot[index] < 0x10) Serial.print('0');
      Serial.print(replySnapshot[index], HEX);
    }
    Serial.println();
    Serial.print(F("[DIAG LCD LINE1] "));
    Serial.println(lcdDesiredLine[0]);
    Serial.print(F("[DIAG LCD LINE2] "));
    Serial.println(lcdDesiredLine[1]);
  }
}

void setup() {
  // active-low 릴레이의 부팅 순간 오동작을 막는다.
  digitalWrite(COOLING_FAN_RELAY_PIN, HIGH);
  pinMode(COOLING_FAN_RELAY_PIN, OUTPUT);
  digitalWrite(HUMIDIFIER_RELAY_PIN, HIGH);
  pinMode(HUMIDIFIER_RELAY_PIN, OUTPUT);
  digitalWrite(PELTIER_RELAY_PIN, HIGH);
  pinMode(PELTIER_RELAY_PIN, OUTPUT);
  stopAllOutputs();

  releaseSoftI2cLine(LCD_SOFT_SDA_PIN);
  releaseSoftI2cLine(LCD_SOFT_SCL_PIN);
  lcdNextStepAt = millis() + LCD_POWER_UP_MS;
  formatDisplayLines();
  scheduleFullLcdRender();

  Serial.begin(9600);
  Serial.println();
  Serial.println(F("[BOOT STEP 1] ActuatorUno setup entered"));
  Serial.println(F("[BOOT STEP 2] all outputs forced OFF"));

  publishStatusReply(STATUS_IDLE, COMMAND_STOP, 0);
  Wire.begin(I2C_ADDRESS);
  Wire.setWireTimeout(WIRE_TIMEOUT_US, true);
  Wire.onReceive(receiveI2cCommand);
  Wire.onRequest(sendI2cStatus);
  Serial.println(F("[BOOT STEP 3] I2C slave callbacks registered"));

  Serial.println(F("[BOOT] ActuatorUno I2C slave started"));
  Serial.println(F("[BOOT] Sensor bus: address=0x09 A4=SDA A5=SCL"));
  Serial.println(F("[BOOT] relays: humidifier=A0 peltier=A1 fan=D7 (active-low)"));
  Serial.println(F("[BOOT] LCD: Software I2C D5=SDA D4=SCL, 0x27 then 0x3F"));
  Serial.println(F("[BOOT] display frame: D1 seq state zone tempLE humLE flags crc8"));
  Serial.println(F("[BOOT] serial test: 1=humidifier 2=peltier+fan 0=off ?=diagnostics"));
}

void loop() {
  // 안전/릴레이 처리를 LCD보다 항상 먼저 실행한다.
  serviceWireTimeout();
  handleSerialTestCommand();
  serviceCommandMailbox();
  serviceActuatorTask();

  serviceDisplayMailbox();
  serviceDisplayStaleness();
  serviceLcd();
}
