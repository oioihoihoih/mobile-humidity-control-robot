#include <Arduino.h>
#include <DHT.h>
#include <Wire.h>

// SimulIDE용 SensorUno 프록시 펌웨어
//
// 실제 SensorUno의 ESP-01 서버 명령과 RC522 UID 판독은 SimulIDE에서
// 그대로 재현하기 어렵다. 이 스케치는 각각을 순간 버튼으로 대체하지만,
// MotorUno/ActuatorUno와 주고받는 I2C 주소와 프레임은 운영 코드와 같다.

constexpr byte MOTOR_ADDRESS = 0x08;
constexpr byte ACTUATOR_ADDRESS = 0x09;

constexpr byte MOTOR_STOP = 0;
constexpr byte MOTOR_OUTBOUND = 1;
constexpr byte MOTOR_TURN_AROUND = 2;
constexpr byte MOTOR_KEEPALIVE = 5;
constexpr byte MOTOR_HOME_SYNC = 6;

constexpr byte MOTOR_IDLE = 0;
constexpr byte MOTOR_RUNNING = 1;
constexpr byte MOTOR_OBSTACLE = 2;
constexpr byte MOTOR_WATCHDOG_TIMEOUT = 4;
constexpr byte MOTOR_INVALID_COMMAND = 5;
constexpr byte MOTOR_CALIBRATION_REQUIRED = 7;

// MotorUno의 논리 3바이트 상태 응답을 SimulIDE에서 1바이트씩 읽기 위한
// 전용 selector다. 운영 프레임의 status/command/sequence 의미는 같다.
constexpr byte MOTOR_STATUS_BYTE_SELECT_BASE = 0xE0;
constexpr byte MOTOR_STATUS_REPLY_SIZE = 3;

constexpr byte ACTUATOR_STOP = 0;
constexpr byte ACTUATOR_HUMIDIFY = 1;
constexpr byte ACTUATOR_DEHUMIDIFY = 2;

constexpr byte ACTUATOR_IDLE = 0;
constexpr byte ACTUATOR_RUNNING = 1;
constexpr byte ACTUATOR_DONE = 2;
constexpr byte ACTUATOR_ERROR = 3;

constexpr byte ACTUATOR_CONTROL_MAGIC = 0xA5;
constexpr byte ACTUATOR_CONTROL_FRAME_SIZE = 4;

// SimulIDE R260501의 다중 AVR slave transmitter는 request당 첫 바이트만
// 안정적으로 반환한다.  0xF0..0xF5로 logical 6-byte status의 위치를 고른
// 뒤 한 바이트씩 읽어 production frame을 그대로 재구성한다.
constexpr byte STATUS_BYTE_SELECT_BASE = 0xF0;
constexpr byte STATUS_REPLY_SIZE = 6;

// 운영 SensorUno -> ActuatorUno LCD telemetry 프로토콜과 같은 값이다.
constexpr byte DISPLAY_FRAME_MAGIC = 0xD1;
constexpr byte DISPLAY_FRAME_SIZE = 10;
constexpr byte DISPLAY_STATE_IDLE = 0;
constexpr byte DISPLAY_STATE_MOVING = 1;
constexpr byte DISPLAY_STATE_HUMIDIFY = 2;
constexpr byte DISPLAY_STATE_DEHUMIDIFY = 3;
constexpr byte DISPLAY_STATE_DONE = 4;
constexpr byte DISPLAY_STATE_RETURNING = 5;
constexpr byte DISPLAY_STATE_ERROR = 6;
constexpr byte DISPLAY_FLAG_DHT_VALID = 0x01;
constexpr byte DISPLAY_FLAG_SERVER_READY = 0x02;
constexpr byte DISPLAY_FLAG_TASK_ACTIVE = 0x04;
constexpr byte DISPLAY_FLAG_FAULT = 0x08;
constexpr byte DISPLAY_STATUS_VALID = 0x01;

// 실제 차량 DHT22는 운영 배선과 같은 D4에 연결한다. 이 값은 LCD 대신
// SensorUno 시리얼 로그에 출력하며 서버 구역 임무 판정에는 사용하지 않는다.
constexpr byte DHT_PIN = 4;
constexpr byte DHT_TYPE = DHT22;
DHT dht(DHT_PIN, DHT_TYPE);

// 서버 명령 프록시 버튼. 실제 ESP-01 대신 쓰는 시뮬레이션 전용 입력이며,
// A0~A3은 운영 SensorUno에서 비어 있으므로 DHT/RC522/I2C 핀을 침범하지 않는다.
constexpr byte TASK_ZONE2_HUMIDIFY_PIN = A0;
constexpr byte TASK_ZONE2_DEHUMIDIFY_PIN = A1;
constexpr byte TASK_ZONE99_HUMIDIFY_PIN = A2;
constexpr byte TASK_ZONE99_DEHUMIDIFY_PIN = A3;
constexpr byte RETURN_HOME_PIN = 5;
constexpr byte ALL_STOP_PIN = 6;

// RC522/HOME 마커 프록시 버튼. D13은 부팅 직후에는 CALIBRATE_HOME,
// 복귀 중에는 실제 HOME 도착 마커로 해석한다.
constexpr byte RFID_ZONE2_PIN = 8;
constexpr byte RFID_ZONE99_PIN = 12;
constexpr byte HOME_MARKER_PIN = 13;

// SimulIDE Push 컴포넌트는 클릭을 매우 짧게 LOW로 만들 수 있다. 실제
// 스위치의 바운스가 없는 논리 프록시이므로 첫 LOW sample을 즉시 받는다.
constexpr unsigned long KEEPALIVE_MS = 400;
constexpr unsigned long ACTUATOR_POLL_MS = 100;
constexpr unsigned long STATUS_LOG_MS = 1000;
constexpr unsigned long DHT_LOG_MS = 2500;
constexpr unsigned long DISPLAY_HEARTBEAT_MS = 2000;
constexpr unsigned long DISPLAY_RETRY_MS = 500;
constexpr unsigned long DISPLAY_ACK_DELAY_MS = 40;
constexpr unsigned long I2C_APPLY_TIMEOUT_MS = 500;

enum Station : byte { HOME = 0, ZONE2 = 1, ZONE99 = 2 };
enum Action : byte { ACTION_NONE = 0, HUMIDIFY = 1, DEHUMIDIFY = 2 };
enum Mode : byte {
  IDLE,
  MOVING_OUTBOUND,
  TASK_RUNNING,
  WAIT_SERVER_NORMAL,
  RETURNING_HOME,
  SAFE_STOP
};

struct Button {
  byte pin;
  bool stable;
  bool sampled;
  unsigned long changedAt;
};

Button buttons[] = {
  {TASK_ZONE2_HUMIDIFY_PIN, HIGH, HIGH, 0},
  {TASK_ZONE2_DEHUMIDIFY_PIN, HIGH, HIGH, 0},
  {TASK_ZONE99_HUMIDIFY_PIN, HIGH, HIGH, 0},
  {TASK_ZONE99_DEHUMIDIFY_PIN, HIGH, HIGH, 0},
  {RETURN_HOME_PIN, HIGH, HIGH, 0},
  {ALL_STOP_PIN, HIGH, HIGH, 0},
  {RFID_ZONE2_PIN, HIGH, HIGH, 0},
  {RFID_ZONE99_PIN, HIGH, HIGH, 0},
  {HOME_MARKER_PIN, HIGH, HIGH, 0},
};

constexpr byte BUTTON_COUNT = sizeof(buttons) / sizeof(buttons[0]);

Station currentStation = HOME;
Station targetStation = HOME;
Action targetAction = ACTION_NONE;
Mode mode = SAFE_STOP;

// 전원 인가 위치를 HOME이라고 추측하지 않는다. HOME 마커 위에서 D13/h를
// 눌러 MotorUno HOME_SYNC가 exact sequence로 ACK된 뒤에만 경로를 연다.
bool routeCalibrated = false;

byte motorSequence = 0;
byte actuatorSequence = 0;
byte lastMotorCommand = MOTOR_STOP;
byte lastActuatorCommand = ACTUATOR_STOP;
byte lastActuatorSequence = 0;
unsigned long lastKeepaliveAt = 0;
unsigned long lastActuatorPollAt = 0;
unsigned long lastStatusLogAt = 0;
unsigned long lastDhtLogAt = 0;

float carTemperature = NAN;
float carHumidity = NAN;
bool carDhtValid = false;
byte displayPayloadCache[7] = {0};
bool displayPayloadCached = false;
byte displaySequence = 0;
bool displayAckPending = false;
byte lastDisplayAckSequence = 0;
byte lastDisplayStatusFlags = 0;
unsigned long lastDisplaySentAt = 0;
unsigned long lastDisplayHeartbeatAt = 0;

const __FlashStringHelper* stationName(Station station) {
  if (station == ZONE2) return F("ZONE2");
  if (station == ZONE99) return F("ZONE99");
  return F("HOME");
}

const __FlashStringHelper* actionName(Action action) {
  if (action == HUMIDIFY) return F("HUMIDIFY");
  if (action == DEHUMIDIFY) return F("DEHUMIDIFY");
  return F("NONE");
}

const __FlashStringHelper* modeName(Mode value) {
  switch (value) {
    case MOVING_OUTBOUND: return F("MOVE_OUTBOUND");
    case TASK_RUNNING: return F("TASK_RUNNING");
    case WAIT_SERVER_NORMAL: return F("WAIT_SERVER_NORMAL");
    case RETURNING_HOME: return F("RETURN_HOME");
    case SAFE_STOP: return F("SAFE_STOP");
    default: return F("IDLE");
  }
}

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

// 운영 SensorUno와 같은 실제 10 kHz 설정이다. Wire.setClock(10000)은
// AVR에서 계산된 TWBR 값이 8비트를 넘어 되감길 수 있으므로 직접 설정한다.
// 느린 SimulIDE slave와 selector 기반 상태 응답도 이 속도에서 검증한다.
void configureI2c10kHz() {
  TWSR = static_cast<byte>((TWSR & ~(_BV(TWPS0) | _BV(TWPS1))) | _BV(TWPS0));
  TWBR = 198;
}

void restoreI2cClockAfterTimeout() {
  if (!Wire.getWireTimeoutFlag()) return;
  Wire.clearWireTimeoutFlag();
  configureI2c10kHz();
}

byte displayStateForMode() {
  switch (mode) {
    case MOVING_OUTBOUND: return DISPLAY_STATE_MOVING;
    case TASK_RUNNING:
      return targetAction == HUMIDIFY ? DISPLAY_STATE_HUMIDIFY
                                      : DISPLAY_STATE_DEHUMIDIFY;
    case WAIT_SERVER_NORMAL: return DISPLAY_STATE_DONE;
    case RETURNING_HOME: return DISPLAY_STATE_RETURNING;
    case SAFE_STOP: return DISPLAY_STATE_ERROR;
    default: return DISPLAY_STATE_IDLE;
  }
}

byte displayZoneCode() {
  if (!routeCalibrated) return 0xFF;
  if (targetStation == ZONE2) return 2;
  if (targetStation == ZONE99) return 99;
  return 0;
}

void buildDisplayPayload(byte* payload) {
  payload[0] = displayStateForMode();
  payload[1] = displayZoneCode();

  int16_t temperatureTenths = 0;
  uint16_t humidityTenths = 0;
  if (carDhtValid) {
    temperatureTenths = static_cast<int16_t>(
        carTemperature * 10.0f + (carTemperature >= 0 ? 0.5f : -0.5f));
    humidityTenths = static_cast<uint16_t>(carHumidity * 10.0f + 0.5f);
  }
  payload[2] = static_cast<byte>(temperatureTenths & 0xFF);
  payload[3] = static_cast<byte>((temperatureTenths >> 8) & 0xFF);
  payload[4] = static_cast<byte>(humidityTenths & 0xFF);
  payload[5] = static_cast<byte>((humidityTenths >> 8) & 0xFF);

  byte flags = DISPLAY_FLAG_SERVER_READY;
  if (carDhtValid) flags |= DISPLAY_FLAG_DHT_VALID;
  if (mode == MOVING_OUTBOUND || mode == TASK_RUNNING ||
      mode == WAIT_SERVER_NORMAL || mode == RETURNING_HOME) {
    flags |= DISPLAY_FLAG_TASK_ACTIVE;
  }
  if (mode == SAFE_STOP) flags |= DISPLAY_FLAG_FAULT;
  payload[6] = flags;
}

bool sendDisplayTelemetryFrame(unsigned long now) {
  byte frame[DISPLAY_FRAME_SIZE];
  frame[0] = DISPLAY_FRAME_MAGIC;
  frame[1] = displaySequence;
  memcpy(frame + 2, displayPayloadCache, sizeof(displayPayloadCache));
  frame[9] = crc8Atm(frame, 9);

  Wire.beginTransmission(ACTUATOR_ADDRESS);
  Wire.write(frame, sizeof(frame));
  const byte error = Wire.endTransmission();
  restoreI2cClockAfterTimeout();
  lastDisplaySentAt = now;
  lastDisplayHeartbeatAt = now;
  Serial.print(F("[DISPLAY->0x09] D1 seq="));
  Serial.print(displaySequence);
  Serial.print(F(" bytes=10 crc=0x"));
  if (frame[9] < 0x10) Serial.print('0');
  Serial.print(frame[9], HEX);
  Serial.print(F(" result="));
  Serial.println(error);
  if (error != 0) return false;
  displayAckPending = true;
  return true;
}

bool readSelectedStatus(byte address, byte selectorBase, byte replySize,
                        byte* response) {
  for (byte index = 0; index < replySize; ++index) {
    Wire.beginTransmission(address);
    Wire.write(selectorBase + index);
    const byte selectError = Wire.endTransmission();
    restoreI2cClockAfterTimeout();
    if (selectError != 0) return false;

    // SimulIDE의 별도 AVR 이벤트에 selector가 반영되도록 잠시 양보한다.
    delay(2);
    const byte received = Wire.requestFrom(static_cast<int>(address), 1);
    restoreI2cClockAfterTimeout();
    if (received != 1 || Wire.available() < 1) {
      while (Wire.available()) Wire.read();
      return false;
    }
    response[index] = static_cast<byte>(Wire.read());
    while (Wire.available()) Wire.read();
    delay(1);
  }
  return true;
}

bool readMotorState(byte& status, byte& command, byte& sequence) {
  byte response[MOTOR_STATUS_REPLY_SIZE];
  if (!readSelectedStatus(MOTOR_ADDRESS, MOTOR_STATUS_BYTE_SELECT_BASE,
                          MOTOR_STATUS_REPLY_SIZE, response)) return false;
  status = response[0];
  command = response[1];
  sequence = response[2];
  return true;
}

bool readActuatorState(byte& status, byte& command, byte& sequence);

bool sendMotor(byte command) {
  const byte sequence = ++motorSequence;
  Wire.beginTransmission(MOTOR_ADDRESS);
  Wire.write(command);
  Wire.write(sequence);
  const byte error = Wire.endTransmission();
  restoreI2cClockAfterTimeout();

  Serial.print(F("[I2C->0x08] command="));
  Serial.print(command);
  Serial.print(F(" sequence="));
  Serial.print(sequence);
  Serial.print(F(" result="));
  Serial.println(error);

  if (error != 0) return false;

  // KEEPALIVE는 MotorUno의 applied command/sequence를 바꾸지 않는다.
  // 대신 상태 7을 읽어 MotorUno 단독 재부팅도 즉시 interlock으로 되돌린다.
  if (command == MOTOR_KEEPALIVE) {
    delay(3);
    byte status;
    byte appliedCommand;
    byte appliedSequence;
    if (!readMotorState(status, appliedCommand, appliedSequence)) return false;
    if (status == MOTOR_CALIBRATION_REQUIRED) {
      routeCalibrated = false;
      return false;
    }
    return status != MOTOR_WATCHDOG_TIMEOUT &&
           status != MOTOR_INVALID_COMMAND;
  }

  const unsigned long startedAt = millis();
  while (millis() - startedAt < I2C_APPLY_TIMEOUT_MS) {
    byte status;
    byte appliedCommand;
    byte appliedSequence;
    if (!readMotorState(status, appliedCommand, appliedSequence) ||
        appliedCommand != command || appliedSequence != sequence) {
      delay(5);
      continue;
    }

    bool accepted = false;
    if (command == MOTOR_STOP) {
      accepted = status == MOTOR_IDLE ||
                 status == MOTOR_CALIBRATION_REQUIRED;
    } else if (command == MOTOR_HOME_SYNC) {
      accepted = status == MOTOR_IDLE;
    } else {
      accepted = status == MOTOR_RUNNING || status == MOTOR_OBSTACLE;
    }
    if (accepted) {
      lastMotorCommand = command;
      return true;
    }
    if (status == MOTOR_CALIBRATION_REQUIRED) routeCalibrated = false;
    return false;
  }
  return false;
}

bool sendActuator(byte command) {
  const byte sequence = ++actuatorSequence;
  byte frame[ACTUATOR_CONTROL_FRAME_SIZE] = {
      ACTUATOR_CONTROL_MAGIC, sequence, command, 0};
  frame[3] = crc8Atm(frame, 3);
  Wire.beginTransmission(ACTUATOR_ADDRESS);
  Wire.write(frame, sizeof(frame));
  const byte error = Wire.endTransmission();
  restoreI2cClockAfterTimeout();

  Serial.print(F("[I2C->0x09] A5 sequence="));
  Serial.print(sequence);
  Serial.print(F(" command="));
  Serial.print(command);
  Serial.print(F(" result="));
  Serial.println(error);

  if (error != 0) return false;

  const byte expectedStatus = command == ACTUATOR_STOP
      ? ACTUATOR_IDLE : ACTUATOR_RUNNING;
  const unsigned long startedAt = millis();
  while (millis() - startedAt < I2C_APPLY_TIMEOUT_MS) {
    byte status;
    byte appliedCommand;
    byte appliedSequence;
    if (!readActuatorState(status, appliedCommand, appliedSequence) ||
        appliedSequence != sequence) {
      delay(5);
      continue;
    }
    if (status == ACTUATOR_ERROR) return false;
    if (status == expectedStatus && appliedCommand == command) {
      lastActuatorCommand = command;
      lastActuatorSequence = sequence;
      return true;
    }
    return false;
  }
  return false;
}

void stopEverything(const __FlashStringHelper* reason) {
  sendMotor(MOTOR_STOP);
  sendActuator(ACTUATOR_STOP);
  targetStation = HOME;
  targetAction = ACTION_NONE;
  mode = SAFE_STOP;
  Serial.print(F("[SAFE STOP] "));
  Serial.println(reason);
}

bool requireHomeCalibration() {
  if (routeCalibrated) return true;
  sendMotor(MOTOR_STOP);
  sendActuator(ACTUATOR_STOP);
  mode = SAFE_STOP;
  Serial.println(F("[CALIBRATION] TASK/RETURN blocked; press D13 or h at HOME"));
  return false;
}

bool performHomeCalibration() {
  // MotorUno HOME_SYNC가 두 IR=HIGH를 확인하고 같은 sequence로 IDLE을
  // ACK할 때까지 HOME/OUTBOUND를 추측하지 않는다. 이 과정에서 모터 LED는
  // 계속 OFF이며 ActuatorUno도 먼저 STOP/IDLE을 확인한다.
  routeCalibrated = false;
  mode = SAFE_STOP;
  targetStation = HOME;
  targetAction = ACTION_NONE;
  const bool actuatorStopped = sendActuator(ACTUATOR_STOP);
  const bool homeSynced = actuatorStopped && sendMotor(MOTOR_HOME_SYNC);
  if (!homeSynced) {
    sendMotor(MOTOR_STOP);
    Serial.println(F("[CALIBRATION] FAILED: release D9/D10 so both are HIGH"));
    return false;
  }

  routeCalibrated = true;
  currentStation = HOME;
  targetStation = HOME;
  targetAction = ACTION_NONE;
  mode = IDLE;
  Serial.println(F("[CALIBRATION] HOME synced; heading=ZONE2; TASK enabled"));
  return true;
}

void startTask(Station station, Action action) {
  if (!requireHomeCalibration()) return;
  if (mode != IDLE && mode != WAIT_SERVER_NORMAL) {
    Serial.println(F("[SERVER PROXY] task ignored: robot busy"));
    return;
  }

  targetStation = station;
  targetAction = action;
  Serial.print(F("[SERVER PROXY] TASK target="));
  Serial.print(stationName(station));
  Serial.print(F(" action="));
  Serial.println(actionName(action));

  if (currentStation == targetStation) {
    sendMotor(MOTOR_STOP);
    if (sendActuator(static_cast<byte>(targetAction))) mode = TASK_RUNNING;
    else stopEverything(F("ActuatorUno not responding"));
    return;
  }

  if (!sendMotor(MOTOR_OUTBOUND)) {
    stopEverything(F("MotorUno not responding"));
    return;
  }
  mode = MOVING_OUTBOUND;
}

void startReturnHome() {
  if (!requireHomeCalibration()) return;
  if (currentStation == HOME) {
    sendMotor(MOTOR_STOP);
    sendActuator(ACTUATOR_STOP);
    targetStation = HOME;
    targetAction = ACTION_NONE;
    mode = IDLE;
    Serial.println(F("[SERVER PROXY] already HOME -> IDLE"));
    return;
  }

  if (mode == TASK_RUNNING) {
    Serial.println(F("[SERVER PROXY] RETURN ignored until actuator DONE"));
    return;
  }

  sendActuator(ACTUATOR_STOP);
  targetStation = HOME;
  targetAction = ACTION_NONE;
  if (!sendMotor(MOTOR_TURN_AROUND)) {
    stopEverything(F("return command failed"));
    return;
  }
  mode = RETURNING_HOME;
  Serial.println(F("[SERVER PROXY] RETURN_HOME -> turn around"));
}

void arriveAt(Station station) {
  Serial.print(F("[RFID PROXY] detected "));
  Serial.println(stationName(station));

  if (mode == MOVING_OUTBOUND) {
    if (station <= currentStation || station > targetStation) {
      stopEverything(F("unexpected outbound station order"));
      return;
    }

    if (!sendMotor(MOTOR_STOP)) {
      stopEverything(F("RFID stop ACK path failed"));
      return;
    }
    currentStation = station;

    if (station == targetStation) {
      if (!sendActuator(static_cast<byte>(targetAction))) {
        stopEverything(F("actuator task failed"));
        return;
      }
      mode = TASK_RUNNING;
      Serial.println(F("[ROUTE] target reached -> actuator start"));
    } else {
      // ZONE99로 가는 동안 ZONE2는 중간 역이다. 정지 확인 뒤 현재
      // 방향을 유지하는 OUTBOUND 명령으로 다시 출발한다.
      sendMotor(MOTOR_OUTBOUND);
      Serial.println(F("[ROUTE] intermediate ZONE2 -> continue to ZONE99"));
    }
    return;
  }

  if (mode == RETURNING_HOME) {
    if (station != ZONE2 || currentStation != ZONE99) {
      stopEverything(F("unexpected return station order"));
      return;
    }
    sendMotor(MOTOR_STOP);
    currentStation = ZONE2;
    // TURN_AROUND 뒤의 물리 방향을 유지한 채 계속 가는 명령이다.
    sendMotor(MOTOR_OUTBOUND);
    Serial.println(F("[ROUTE] return passed ZONE2 -> continue HOME"));
  }
}

void arriveHomeMarker() {
  Serial.println(F("[HOME MARKER PROXY] detected"));
  if (!routeCalibrated) {
    performHomeCalibration();
    return;
  }
  if (mode != RETURNING_HOME || currentStation != ZONE2) {
    stopEverything(F("HOME marker before return ZONE2"));
    return;
  }
  sendMotor(MOTOR_STOP);
  sendActuator(ACTUATOR_STOP);
  currentStation = HOME;
  targetStation = HOME;
  targetAction = ACTION_NONE;
  mode = IDLE;
  Serial.println(F("[ROUTE] HOME reached -> IDLE"));
}

bool pressedEdge(Button& button, unsigned long now) {
  const bool reading = digitalRead(button.pin);
  button.sampled = reading;
  button.changedAt = now;
  if (reading == button.stable) return false;
  button.stable = reading;
  return reading == LOW;
}

void serviceButtons(unsigned long now) {
  bool pressed[BUTTON_COUNT];
  for (byte i = 0; i < BUTTON_COUNT; ++i) {
    pressed[i] = pressedEdge(buttons[i], now);
  }

  if (pressed[5]) {
    stopEverything(F("ALL_STOP proxy button"));
    return;
  }
  if (pressed[0]) startTask(ZONE2, HUMIDIFY);
  if (pressed[1]) startTask(ZONE2, DEHUMIDIFY);
  if (pressed[2]) startTask(ZONE99, HUMIDIFY);
  if (pressed[3]) startTask(ZONE99, DEHUMIDIFY);
  if (pressed[4]) startReturnHome();
  if (pressed[6]) arriveAt(ZONE2);
  if (pressed[7]) arriveAt(ZONE99);
  if (pressed[8]) arriveHomeMarker();
}

void serviceSerialProxy() {
  while (Serial.available()) {
    const char value = static_cast<char>(Serial.read());
    if (value == '2') startTask(ZONE2, HUMIDIFY);
    else if (value == '3') startTask(ZONE2, DEHUMIDIFY);
    else if (value == '9') startTask(ZONE99, HUMIDIFY);
    else if (value == '0') startTask(ZONE99, DEHUMIDIFY);
    else if (value == 'r' || value == 'R') startReturnHome();
    else if (value == 'a' || value == 'A') stopEverything(F("serial ALL_STOP"));
    else if (value == 'z' || value == 'Z') arriveAt(ZONE2);
    else if (value == 'x' || value == 'X') arriveAt(ZONE99);
    else if (value == 'h' || value == 'H') arriveHomeMarker();
  }
}

bool readActuatorState(byte& status, byte& command, byte& sequence) {
  byte response[STATUS_REPLY_SIZE];
  if (!readSelectedStatus(ACTUATOR_ADDRESS, STATUS_BYTE_SELECT_BASE,
                          STATUS_REPLY_SIZE, response)) return false;
  if (crc8Atm(response, 5) != response[5]) {
    const byte expectedCrc = crc8Atm(response, 5);
    // SimulIDE의 세 AVR 동시 실행은 긴 연속 Serial.print 호출을 화면에서
    // 누락시킬 수 있다. 한 번에 짧은 진단 문자열을 보내 실제 수신 6바이트와
    // 계산 CRC를 온전히 남기고, 1초에 한 번으로 제한해 UART를 포화시키지 않는다.
    static unsigned long lastCrcLogAt = 0;
    const unsigned long now = millis();
    if (lastCrcLogAt == 0 || now - lastCrcLogAt >= 1000) {
      lastCrcLogAt = now;
      char line[40];
      snprintf_P(line, sizeof(line),
                 PSTR("I2C9 CRC %02X%02X%02X%02X%02X%02X/%02X"),
                 response[0], response[1], response[2], response[3],
                 response[4], response[5], expectedCrc);
      Serial.println(line);
    }
    return false;
  }
  status = response[0];
  command = response[1];
  sequence = response[2];
  lastDisplayAckSequence = response[3];
  lastDisplayStatusFlags = response[4];
  if ((lastDisplayStatusFlags & DISPLAY_STATUS_VALID) &&
      lastDisplayAckSequence == displaySequence) {
    displayAckPending = false;
  }
  return true;
}

void serviceDisplayTelemetry(unsigned long now) {
  byte payload[sizeof(displayPayloadCache)];
  buildDisplayPayload(payload);
  if (!displayPayloadCached ||
      memcmp(payload, displayPayloadCache, sizeof(payload)) != 0) {
    memcpy(displayPayloadCache, payload, sizeof(payload));
    displayPayloadCached = true;
    ++displaySequence;
    displayAckPending = true;
    lastDisplaySentAt = now - DISPLAY_RETRY_MS;
  }

  const bool heartbeatDue =
      now - lastDisplayHeartbeatAt >= DISPLAY_HEARTBEAT_MS;
  const bool retryDue =
      displayAckPending && now - lastDisplaySentAt >= DISPLAY_RETRY_MS;
  if (heartbeatDue || retryDue) {
    sendDisplayTelemetryFrame(now);
    return;
  }

  if (displayAckPending &&
      now - lastDisplaySentAt >= DISPLAY_ACK_DELAY_MS) {
    byte status;
    byte command;
    byte sequence;
    if (!readActuatorState(status, command, sequence)) return;
    if (!displayAckPending) {
      Serial.print(F("[DISPLAY<-0x09] seq ACK="));
      Serial.print(lastDisplayAckSequence);
      Serial.print(F(" flags=0x"));
      Serial.println(lastDisplayStatusFlags, HEX);
    }
  }
}

void pollActuator() {
  if (mode != TASK_RUNNING) return;
  byte status;
  byte command;
  byte sequence;
  if (!readActuatorState(status, command, sequence)) {
    stopEverything(F("ActuatorUno status missing"));
    return;
  }
  if (status == ACTUATOR_ERROR) {
    stopEverything(F("ActuatorUno ERROR"));
    return;
  }
  if (sequence != lastActuatorSequence ||
      command != static_cast<byte>(targetAction)) {
    stopEverything(F("ActuatorUno stale/mismatched sequence"));
    return;
  }
  if (status == ACTUATOR_RUNNING) return;
  if (status == ACTUATOR_DONE &&
      command == static_cast<byte>(targetAction)) {
    sendActuator(ACTUATOR_STOP);
    mode = WAIT_SERVER_NORMAL;
    Serial.println(F("[TASK] DONE -> press NORMAL/RETURN proxy"));
    return;
  }
  stopEverything(F("ActuatorUno unexpected non-running state"));
}

void printStatus() {
  Serial.print(F("[STATE] mode="));
  Serial.print(modeName(mode));
  Serial.print(F(" current="));
  if (routeCalibrated) Serial.print(stationName(currentStation));
  else Serial.print(F("UNKNOWN"));
  Serial.print(F(" target="));
  Serial.print(stationName(targetStation));
  Serial.print(F(" action="));
  Serial.print(actionName(targetAction));
  Serial.print(F(" motorCmd="));
  Serial.print(lastMotorCommand);
  Serial.print(F(" actuatorCmd="));
  Serial.println(lastActuatorCommand);
}

void printCarDht(unsigned long now) {
  if (now - lastDhtLogAt < DHT_LOG_MS) return;
  lastDhtLogAt = now;
  carHumidity = dht.readHumidity();
  carTemperature = dht.readTemperature();
  carDhtValid = !isnan(carHumidity) && !isnan(carTemperature) &&
                carHumidity >= 0.0f && carHumidity <= 100.0f;
  Serial.print(F("[CAR DHT22 D4] "));
  if (!carDhtValid) {
    Serial.println(F("READ ERROR"));
    return;
  }
  Serial.print(F("T="));
  Serial.print(carTemperature, 1);
  Serial.print(F("C H="));
  Serial.print(carHumidity, 1);
  Serial.println(F("%"));
}

void setup() {
  // 운영 SensorUno와 같은 115200bps를 사용해 I2C/RFID 경로 로그가
  // 송신 버퍼를 오래 점유하거나 SimulIDE 터미널에서 섞이지 않게 한다.
  Serial.begin(115200);
  dht.begin();
  for (byte i = 0; i < BUTTON_COUNT; ++i) pinMode(buttons[i].pin, INPUT_PULLUP);

  Wire.begin();
  // SimulIDE 또는 실제 버스에서 slave 응답이 끊겨도 주 루프와 LCD heartbeat가
  // 영구 정지하지 않도록 운영 코드와 같은 25 ms Wire timeout을 사용한다.
  Wire.setWireTimeout(25000UL, true);
  configureI2c10kHz();
  delay(100);
  sendMotor(MOTOR_STOP);
  sendActuator(ACTUATOR_STOP);
  const unsigned long now = millis();
  lastDisplaySentAt = now - DISPLAY_RETRY_MS;
  lastDisplayHeartbeatAt = now - DISPLAY_HEARTBEAT_MS;
  serviceDisplayTelemetry(now);

  Serial.println(F("SensorUno 3-Uno proxy ready"));
  Serial.println(F("Route: HOME -> ZONE2 -> ZONE99"));
  Serial.println(F("I2C: Motor=0x08 Actuator=0x09, A5 control + D1 display"));
  Serial.println(F("CALIBRATION REQUIRED: HOME marker + h/D13; motor stays OFF"));
  Serial.println(F("Serial: h=CALIBRATE/HOME, 2/3=Z2 H/D, 9/0=Z99 H/D, z/x=RFID, r=RETURN, a=STOP"));
}

void loop() {
  const unsigned long now = millis();
  serviceButtons(now);
  serviceSerialProxy();
  printCarDht(now);
  serviceDisplayTelemetry(now);

  if ((mode == MOVING_OUTBOUND || mode == RETURNING_HOME) &&
      now - lastKeepaliveAt >= KEEPALIVE_MS) {
    lastKeepaliveAt = now;
    if (!sendMotor(MOTOR_KEEPALIVE)) {
      stopEverything(routeCalibrated
          ? F("MotorUno status/keepalive missing")
          : F("MotorUno rebooted: HOME calibration required"));
    }
  }

  if (mode == TASK_RUNNING && now - lastActuatorPollAt >= ACTUATOR_POLL_MS) {
    lastActuatorPollAt = now;
    pollActuator();
  }

  if (now - lastStatusLogAt >= STATUS_LOG_MS) {
    lastStatusLogAt = now;
    printStatus();
  }
}
