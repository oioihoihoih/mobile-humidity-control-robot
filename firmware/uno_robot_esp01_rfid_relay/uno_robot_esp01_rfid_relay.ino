#include <Arduino.h>
#include <Wire.h>
#include <SoftwareSerial.h>
#include "SoftwareMFRC522.h"
#include "robot_network_config.h"

// [메인 제어 Arduino Uno]
// D5(UNO TX) -> ESP-01 RX(5V->3.3V 분압 필수)
// D6(UNO RX) <- ESP-01 TX
// RC522 SDA/SCK/MOSI/MISO/RST -> D8/D9/D10/D11/D12 (소프트웨어 SPI)
// D2/D3/D4는 주변 센서를 다른 Uno로 분배해 비워 둔다.
// DHT22 DATA -> ActuatorUno D2
// 뒤쪽 HC-SR04 ECHO/TRIG -> MotorUno D2/A1
// 주행 Uno와 환경 Uno는 A4(SDA), A5(SCL), GND의 I2C 버스로 연결
// 세 Uno와 전원들의 제어 GND는 반드시 공통으로 연결한다.
constexpr byte UNO_ESP_RX_PIN = 6;  // ESP-01 TX가 연결되는 Uno 수신 핀
constexpr byte UNO_ESP_TX_PIN = 5;  // ESP-01 RX가 연결되는 Uno 송신 핀
constexpr byte RFID_SS_PIN = 8;
constexpr byte RFID_SCK_PIN = 9;
constexpr byte RFID_MOSI_PIN = 10;
constexpr byte RFID_MISO_PIN = 11;
constexpr byte RFID_RST_PIN = 12;
constexpr unsigned long USB_SERIAL_BAUD = 115200;

// UNO 세 대 사이에서 사용하는 I2C 주소와 명령값이다. MotorUno 명령은
// 오래된 상태를 ACK로 오인하지 않도록 command+sequence 2바이트로 보낸다.
constexpr byte MOTOR_UNO_ADDRESS = 0x08;
constexpr byte ACTUATOR_UNO_ADDRESS = 0x09;
constexpr byte MOTOR_COMMAND_STOP = 0;
// 구형 1/2와 값 자체를 분리해 SensorUno/MotorUno 부분 업로드나 downgrade가
// 반대 방향 주행으로 이어지지 않게 한다.
constexpr byte MOTOR_COMMAND_OUTBOUND = 0x11;
constexpr byte MOTOR_COMMAND_RETURN = 0x12;
constexpr byte MOTOR_COMMAND_PAUSE = 3;
constexpr byte MOTOR_COMMAND_RESUME = 4;
constexpr byte MOTOR_COMMAND_KEEPALIVE = 5;
constexpr byte MOTOR_COMMAND_HOME_SYNC = 6;
constexpr byte MOTOR_COMMAND_PROTOCOL_SYNC = 7;
constexpr byte MOTOR_STATUS_IDLE = 0;
constexpr byte MOTOR_STATUS_RUNNING = 1;
constexpr byte MOTOR_STATUS_OBSTACLE = 2;
constexpr byte MOTOR_STATUS_STOP_LINE = 3;
constexpr byte MOTOR_STATUS_WATCHDOG_TIMEOUT = 4;
constexpr byte MOTOR_STATUS_INVALID_COMMAND = 5;
constexpr byte MOTOR_STATUS_UNEXPECTED_MARKER = 6;
constexpr byte MOTOR_STATUS_CALIBRATION_REQUIRED = 7;
constexpr byte MOTOR_STATUS_PROTOCOL_REQUIRED = 8;
constexpr byte ACTUATOR_COMMAND_STOP = 0;
constexpr byte ACTUATOR_COMMAND_HUMIDIFY = 1;
constexpr byte ACTUATOR_COMMAND_DEHUMIDIFY = 2;
constexpr byte ACTUATOR_STATUS_IDLE = 0;
constexpr byte ACTUATOR_STATUS_RUNNING = 1;
constexpr byte ACTUATOR_STATUS_DONE = 2;
constexpr byte ACTUATOR_STATUS_ERROR = 3;
constexpr byte ACTUATOR_CONTROL_MAGIC = 0xA5;
constexpr byte ACTUATOR_CONTROL_FRAME_SIZE = 4;
constexpr byte ACTUATOR_STATUS_REPLY_SIZE = 6;

// LCD1602는 ActuatorUno의 D5(SDA)/D4(SCL) 소프트웨어 I2C로 이동했다.
// SensorUno는 문자열을 만들지 않고 아래 10바이트 고정 프레임만 0x09로
// 전달한다. 4바이트 CRC 제어 프레임과 길이로 구분한다.
constexpr byte DISPLAY_FRAME_MAGIC = 0xD1;
constexpr byte DISPLAY_FRAME_SIZE = 10;
constexpr byte DISPLAY_STATE_IDLE = 0;
constexpr byte DISPLAY_STATE_MOVING = 1;
constexpr byte DISPLAY_STATE_HUMIDIFY = 2;
constexpr byte DISPLAY_STATE_DEHUMIDIFY = 3;
constexpr byte DISPLAY_STATE_DONE = 4;
constexpr byte DISPLAY_STATE_RETURNING = 5;
constexpr byte DISPLAY_STATE_ERROR = 6;
constexpr byte DISPLAY_FLAG_WIFI_READY = 0x02;
constexpr byte DISPLAY_FLAG_TASK_ACTIVE = 0x04;
constexpr byte DISPLAY_FLAG_FAULT = 0x08;
constexpr byte DISPLAY_STATUS_VALID = 0x01;
constexpr byte DISPLAY_STATUS_LCD_READY = 0x02;
constexpr byte DISPLAY_STATUS_LCD_ERROR = 0x04;
constexpr byte DISPLAY_STATUS_STALE = 0x08;

const char WIFI_SSID[] = ROBOT_WIFI_SSID;
const char WIFI_PASSWORD[] = ROBOT_WIFI_PASSWORD;
const char SERVER_HOST[] = ROBOT_SERVER_HOST;
constexpr uint16_t SERVER_PORT = ROBOT_SERVER_PORT;

// RC522 테스트에서 출력된 실제 UID로 교체한다.
const char RFID_ZONE2_UID[] PROGMEM = ROBOT_ZONE2_UID;
const char RFID_ZONE99_UID[] PROGMEM = ROBOT_ZONE99_UID;

// 실제 모터·가습/제습 모듈을 연결하기 전의 통합 로직 시험 설정입니다.
// false이면 목표 구역에 등록된 UID가 일치할 때만 도착으로 인정합니다.
constexpr bool RFID_PLACEHOLDER_ACCEPT_ANY_CARD = false;
// 운영 빌드에서는 USB 문자 명령을 제거해 우발 주행을 막고 프로토콜
// handshake·후진 안전 로직에 필요한 플래시 여유를 확보한다.
constexpr bool ENABLE_SERIAL_TEST_COMMANDS = false;
// 이전 2모터 벤치 시험용 z one-shot은 운영 빌드에서 끈다. 필요할 때만 true로
// 바꾸며, 일반 운전은 서버 명령과 실제 RFID 경로를 사용한다.
constexpr bool ENABLE_USB_ZONE2_MISSION_TEST = false;
// Uno 플래시 여유를 확보하기 위해 긴 AT/HTTP 원문 덤프는 기본 OFF다.
// 오류, 명령, RFID, I2C, 임무 상태 로그는 이 설정과 무관하게 계속 출력한다.
#define VERBOSE_NETWORK_LOGS 0
// 안전 오류와 '?' 통합 진단은 항상 남기고, 정상 진행 중 반복되는 설명 로그는
// 플래시가 작은 Uno 빌드에서 제거한다. 1로 바꾸면 상세 시연 로그를 복원한다.
#define VERBOSE_OPERATION_LOGS 0
#if VERBOSE_OPERATION_LOGS
#define TRACE_PRINT(...) Serial.print(__VA_ARGS__)
#define TRACE_PRINTLN(...) Serial.println(__VA_ARGS__)
#else
#define TRACE_PRINT(...)
#define TRACE_PRINTLN(...)
#endif
constexpr unsigned long MOVE_SAFETY_TIMEOUT_MS = 30000;
constexpr unsigned long RETURN_SAFETY_TIMEOUT_MS = 30000;
constexpr unsigned long MODULE_TIMEOUT_MS = 60000;
constexpr unsigned long I2C_APPLY_TIMEOUT_MS = 750;
constexpr unsigned long MOTOR_KEEPALIVE_MS = 400;
// 뒤쪽 HC-SR04는 MotorUno가 직접 읽고 네 바퀴를 로컬 정지시킨다.
// SensorUno는 Motor status=OBSTACLE만 받아 경로/RFID 상태를 보존한다.

SoftwareSerial esp8266(UNO_ESP_RX_PIN, UNO_ESP_TX_PIN);
SoftwareMFRC522 rfid(RFID_SS_PIN, RFID_RST_PIN, RFID_SCK_PIN,
                     RFID_MOSI_PIN, RFID_MISO_PIN);

// HTTP 헤더 전체를 보관하지 않고 본문 '{'부터 다시 담으므로 명령 JSON에
// 충분한 크기만 사용한다. Uno의 HTTP 처리 중 스택 충돌 재부팅을 피한다.
char espBuffer[128];
// HTTP/1.0은 기본 close이므로 Host/Connection 헤더를 생략해 최악 173B의
// 상태 요청도 여유 있게 담고, Uno SRAM을 절약한다.
char requestBuffer[180];
// 현재 사용하는 가장 긴 AT 명령도 64바이트 미만이므로 SRAM을 절약한다.
char atBuffer[48];
char targetZone[8] = "HOME";
char targetAction[16] = "NONE";
// MFRC522가 지원하는 최대 10바이트 UID는 "AA BB ..." 형식으로 30바이트가 필요하다.
char uidText[30];
char pendingEvent[24] = "BOOT";
char deferredEvent[24] = "";
char commandResult[16] = "BOOT";

enum RobotPhase : byte {
  PHASE_IDLE,
  PHASE_MOVING,
  PHASE_MODULE_RUNNING,
  PHASE_TASK_COMPLETE,
  PHASE_RETURNING
};

// STOP 명령의 I2C ACK가 한 번 누락됐다고 정지 완료로 간주하지 않는다.
// 서버 명령 revision과 무관하게 로컬에서 재시도하기 위한 상태다.
enum StopRetryMode : byte {
  STOP_RETRY_NONE,
  STOP_RETRY_TO_IDLE,
  STOP_RETRY_SERVER_RECOVERY,
  STOP_RETRY_KEEP_FAULT
};

// 직선 트랙의 물리적 순서다. 새 4륜 차체는 제자리 회전하지 않는다.
// OUTBOUND는 차체 앞쪽으로 전진, HOMEBOUND는 같은 자세로 뒤쪽 후진이며,
// 어느 역을 통과/정차할지는 SensorUno가 관리한다.
enum RouteStation : int8_t {
  STATION_UNKNOWN = -1,
  STATION_HOME = 0,
  STATION_ZONE2 = 1,
  STATION_ZONE99 = 2
};

enum RouteHeading : int8_t {
  HEADING_HOMEBOUND = -1,
  HEADING_OUTBOUND = 1
};

bool wifiReady = false;
// ESP-01이 이전 TCP 전송 처리에 묶이면 구형 AT 펌웨어는 `busy s...`,
// 신형 ESP-AT는 `busy p...`를 반환한다. 이때 AT 명령을 연속 전송하면
// 고착이 길어지므로 busy를 별도로 기억해 15초 reconnect backoff로 넘긴다.
bool espBusySeen = false;
bool rfidReady = false;
// 전원 인가 위치/방향을 HOME으로 추측하지 않는다. 사용자가 차를 HOME의
// 넓은 검은 마커 위에서 ZONE2 방향으로 놓고 HOME_SYNC를 성공시킨 뒤에만
// TASK/RETURN/수동 주행을 허용한다.
bool routeCalibrated = false;
bool taskActive = false;
bool manualForwardActive = false;
bool usbZone2MissionActive = false;
bool retrySameRevisionAllowed = false;
StopRetryMode stopRetryMode = STOP_RETRY_NONE;
bool stopMotorRetryNeeded = false;
bool stopActuatorRetryNeeded = false;
unsigned long lastStopRetryAt = 0;
bool robotReportPending = true;
unsigned long lastPollAt = 0;
unsigned long lastRfidLogAt = 0;
unsigned long phaseStartedAt = 0;
RobotPhase robotPhase = PHASE_IDLE;
RouteStation confirmedStation = STATION_UNKNOWN;
RouteStation expectedStation = STATION_UNKNOWN;
RouteStation targetStation = STATION_HOME;
RouteHeading routeHeading = HEADING_OUTBOUND;
bool routeAtStation = false;
constexpr unsigned long POLL_INTERVAL_MS = 3000;
constexpr unsigned long STOP_RETRY_INTERVAL_MS = 500;
// ESP-01 또는 AP가 응답하지 않을 때 매 서버 폴링마다 긴 초기화를
// 반복하면 RFID/I2C 처리가 수십 초씩 멈춘다. 실패 뒤에는 정상
// loop()를 계속 돌리고, 이 간격이 지난 뒤에만 다시 접속을 시도한다.
constexpr unsigned long WIFI_RECONNECT_INTERVAL_MS = 15000;
// 주행 중 태그를 지나치지 않도록 RC522를 짧은 간격으로 확인한다. 같은
// 태그가 리더 아래에 남아 있어도 두 번 역으로 처리하지 않도록 별도 보호한다.
constexpr unsigned long RFID_SCAN_INTERVAL_MS = 40;
// 전진/후진을 바꾼 직후에는 방금 지나온 카드가 RC522 아래에 남아 있을 수
// 있다. 짧은 기계 안정 시간과 최소 한 번의 no-card 관측 뒤에만 새 도착을
// 허용한다. 이는 회전 시간이 아니라 태그 이탈 확인용 가드다.
// 1:298 N20 저속 후륜이 판독 영역을 빠져나갈 시간을 고려한 초기 안전값이다.
// 실차 속도를 측정한 뒤 줄일 수 있지만 no-card 조건 자체는 유지한다.
constexpr unsigned long RFID_DIRECTION_SETTLE_MS = 850;
constexpr unsigned long RFID_LOG_INTERVAL_MS = 5000;
constexpr unsigned long MODULE_STATUS_POLL_MS = 100;
unsigned long lastModuleStatusPollAt = 0;
byte lastActuatorStatus = 0xFF;
byte expectedActuatorCommand = ACTUATOR_COMMAND_STOP;
byte actuatorSequenceCounter = 0;
byte expectedActuatorSequence = 0;
bool actuatorRunningObserved = false;
unsigned long lastMotorKeepaliveAt = 0;
byte lastMotorStatus = 0xFF;
byte motorSequenceCounter = 0;
byte acknowledgedMotorCommand = MOTOR_COMMAND_STOP;
byte acknowledgedMotorSequence = 0;
bool motorLinkFaultPending = false;
long lastCommandRevision = -1;
long acknowledgedRevision = -1;
byte consecutiveServerFailures = 0;
constexpr byte SERVER_FAILURES_BEFORE_WIFI_CHECK = 3;
// MotorUno의 로컬 후방 초음파가 정지시킨 동안 방향 변경 직후의 RFID를
// 새 역으로 오인하지 않도록 status=OBSTACLE만 기억한다.
bool obstaclePauseActive = false;
constexpr unsigned long DISPLAY_HEARTBEAT_MS = 2000;
constexpr unsigned long DISPLAY_RETRY_MS = 500;
constexpr unsigned long DISPLAY_ACK_DELAY_MS = 40;
byte displaySequence = 0;
byte displayPayloadCache[7] = {0};
bool displayPayloadCached = false;
bool displayAckPending = false;
byte lastDisplayAckSequence = 0;
byte lastDisplayStatusFlags = 0;
unsigned long lastDisplaySentAt = 0;
unsigned long lastDisplayHeartbeatAt = 0;
unsigned long lastWifiReconnectAttemptAt = 0;
unsigned long lastHeartbeatAt = 0;
constexpr unsigned long HEARTBEAT_INTERVAL_MS = 9000;
const char HEARTBEAT_EVENT[] = "HEARTBEAT";
unsigned long lastRfidScanAt = 0;
RouteStation lastAcceptedRfidStation = STATION_UNKNOWN;
unsigned long rfidDirectionGuardStartedAt = 0;
bool rfidDirectionGuardActive = false;
bool rfidDirectionClearSeen = false;

// Arduino 자동 프로토타입 생성기가 기본 인자를 안정적으로 처리하지 못하는
// 경우가 있어, 경로 함수보다 뒤에 정의된 모듈 시작 함수를 명시한다.
bool startPlaceholderModule(bool reportArrival = true);
bool reportRobotStatus(bool heartbeatOnly = false);
bool extractJsonLong(const char* key, long& output);
void checkRfidArrival();
void processRouteRfid(RouteStation scannedStation);
void serviceStopRetry();
void armStopRetry(StopRetryMode mode, bool motorStopped, bool moduleStopped);

// Wire의 자동 타임아웃 복구는 TWBR은 보존하지만 prescaler를 1로
// 초기화한다. 세 Uno의 제어 버스(A4/A5)를 항상 실제 10kHz로 맞춘다.
// LCD는 ActuatorUno D5/D4의 별도 소프트웨어 I2C 버스에 있다.
void configureI2c10kHz() {
  TWSR = static_cast<byte>((TWSR & ~(_BV(TWPS0) | _BV(TWPS1))) | _BV(TWPS0));
  TWBR = 198;
}

bool restoreI2cClockAfterTimeout() {
  if (!Wire.getWireTimeoutFlag()) return false;
  Wire.clearWireTimeoutFlag();
  configureI2c10kHz();
  return true;
}

bool finishI2cTransmission(byte address) {
  const byte result = Wire.endTransmission();
  restoreI2cClockAfterTimeout();
  if (result != 0) {
    Serial.print(F("[I2C] transmit failed address=0x"));
    Serial.print(address, HEX);
    Serial.print(F(" result="));
    Serial.println(result);
    Serial.print(F("[I2C] bus levels SDA(A4)="));
    Serial.print(digitalRead(SDA));
    Serial.print(F(" SCL(A5)="));
    Serial.println(digitalRead(SCL));
    return false;
  }
  return true;
}

// MotorUno에는 명령과 8비트 순번을 한 프레임으로 전송한다.
bool sendMotorFrame(byte command, byte sequence) {
  Wire.beginTransmission(MOTOR_UNO_ADDRESS);
  Wire.write(command);
  Wire.write(sequence);
  return finishI2cTransmission(MOTOR_UNO_ADDRESS);
}

// MotorUno가 실제 적용한 명령/순번까지 3바이트로 확인한다.
bool readMotorState(byte& status, byte& appliedCommand,
                    byte& appliedSequence) {
  const byte received =
      Wire.requestFrom(static_cast<int>(MOTOR_UNO_ADDRESS), 3);
  restoreI2cClockAfterTimeout();
  if (received < 3 || Wire.available() < 3) {
    while (Wire.available()) Wire.read();
    status = 0xFF;
    appliedCommand = 0xFF;
    appliedSequence = 0xFF;
    return false;
  }
  status = static_cast<byte>(Wire.read());
  appliedCommand = static_cast<byte>(Wire.read());
  appliedSequence = static_cast<byte>(Wire.read());
  return true;
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

// 제어 명령마다 새 순번을 발급하고 magic/CRC가 있는 4바이트 프레임으로 보낸다.
bool sendActuatorFrame(byte command, byte& sequence) {
  sequence = ++actuatorSequenceCounter;
  byte frame[ACTUATOR_CONTROL_FRAME_SIZE] = {
      ACTUATOR_CONTROL_MAGIC, sequence, command, 0};
  frame[3] = crc8Atm(frame, 3);
  Wire.beginTransmission(ACTUATOR_UNO_ADDRESS);
  Wire.write(frame, sizeof(frame));
  return finishI2cTransmission(ACTUATOR_UNO_ADDRESS);
}

// status/command/appliedSeq/displaySeq/displayFlags/CRC를 모두 검증한다.
bool readActuatorState(byte& status, byte& appliedCommand,
                       byte& appliedSequence) {
  const byte received = Wire.requestFrom(
      static_cast<int>(ACTUATOR_UNO_ADDRESS),
      static_cast<int>(ACTUATOR_STATUS_REPLY_SIZE));
  restoreI2cClockAfterTimeout();
  if (received < ACTUATOR_STATUS_REPLY_SIZE ||
      Wire.available() < ACTUATOR_STATUS_REPLY_SIZE) {
    while (Wire.available()) Wire.read();
    status = 0xFF;
    appliedCommand = 0xFF;
    appliedSequence = 0xFF;
    return false;
  }
  byte response[ACTUATOR_STATUS_REPLY_SIZE];
  for (byte index = 0; index < sizeof(response); ++index) {
    response[index] = static_cast<byte>(Wire.read());
  }
  if (crc8Atm(response, 5) != response[5]) {
    status = 0xFF;
    appliedCommand = 0xFF;
    appliedSequence = 0xFF;
    return false;
  }
  status = response[0];
  appliedCommand = response[1];
  appliedSequence = response[2];
  lastDisplayAckSequence = response[3];
  lastDisplayStatusFlags = response[4];
  if ((lastDisplayStatusFlags & DISPLAY_STATUS_VALID) &&
      lastDisplayAckSequence == displaySequence) {
    displayAckPending = false;
  }
  return true;
}

// 전원을 켰을 때 슬레이브 UNO가 실제로 I2C 버스에 연결됐는지 확인한다.
bool i2cDevicePresent(byte address) {
  Wire.beginTransmission(address);
  const byte result = Wire.endTransmission();
  restoreI2cClockAfterTimeout();
  return result == 0;
}

void probeI2cDevice(byte address, const __FlashStringHelper* name) {
  const bool connected = i2cDevicePresent(address);
  Serial.print(F("[I2C] "));
  Serial.print(name);
  Serial.print(F(" address=0x"));
  Serial.print(address, HEX);
  Serial.println(connected ? F(" connected") : F(" not found"));
}

byte currentDisplayState() {
  if (!strcmp_P(commandResult, PSTR("FAILED"))) return DISPLAY_STATE_ERROR;
  switch (robotPhase) {
    case PHASE_MOVING: return DISPLAY_STATE_MOVING;
    case PHASE_MODULE_RUNNING:
      if (!strcmp_P(targetAction, PSTR("HUMIDIFY"))) {
        return DISPLAY_STATE_HUMIDIFY;
      }
      if (!strcmp_P(targetAction, PSTR("DEHUMIDIFY"))) {
        return DISPLAY_STATE_DEHUMIDIFY;
      }
      return DISPLAY_STATE_ERROR;
    case PHASE_TASK_COMPLETE: return DISPLAY_STATE_DONE;
    case PHASE_RETURNING: return DISPLAY_STATE_RETURNING;
    default: return DISPLAY_STATE_IDLE;
  }
}

byte currentDisplayZoneCode() {
  switch (targetStation) {
    case STATION_HOME: return 0;
    case STATION_ZONE2: return 2;
    case STATION_ZONE99: return 99;
    default: return 0xFF;
  }
}

void buildDisplayPayload(byte* payload) {
  payload[0] = currentDisplayState();
  payload[1] = currentDisplayZoneCode();

  // 10바이트 wire 형식은 이전 Sensor/Actuator 조합과 호환되게 유지한다.
  // 환경값은 ActuatorUno D2의 로컬 DHT22가 직접 측정하므로 이 네 바이트와
  // bit0(DHT valid)은 예약값 0으로 보낸다.
  payload[2] = 0;
  payload[3] = 0;
  payload[4] = 0;
  payload[5] = 0;

  byte flags = 0;
  if (wifiReady) flags |= DISPLAY_FLAG_WIFI_READY;
  if (taskActive) flags |= DISPLAY_FLAG_TASK_ACTIVE;
  if (!strcmp_P(commandResult, PSTR("FAILED"))) flags |= DISPLAY_FLAG_FAULT;
  payload[6] = flags;
}

bool sendDisplayTelemetryFrame() {
  byte frame[DISPLAY_FRAME_SIZE];
  frame[0] = DISPLAY_FRAME_MAGIC;
  frame[1] = displaySequence;
  memcpy(frame + 2, displayPayloadCache, sizeof(displayPayloadCache));
  frame[9] = crc8Atm(frame, 9);

  Wire.beginTransmission(ACTUATOR_UNO_ADDRESS);
  Wire.write(frame, sizeof(frame));
  const bool sent = finishI2cTransmission(ACTUATOR_UNO_ADDRESS);
  lastDisplaySentAt = millis();
  lastDisplayHeartbeatAt = lastDisplaySentAt;
  if (!sent) {
    Serial.println(F("[DISPLAY] telemetry send failed"));
    return false;
  }
  displayAckPending = true;
  return true;
}

void serviceDisplayTelemetry() {
  byte payload[sizeof(displayPayloadCache)];
  buildDisplayPayload(payload);
  if (!displayPayloadCached ||
      memcmp(payload, displayPayloadCache, sizeof(payload)) != 0) {
    memcpy(displayPayloadCache, payload, sizeof(payload));
    displayPayloadCached = true;
    ++displaySequence;
    displayAckPending = true;
    // 상태 변화는 다음 loop에서 즉시 전송한다.
    lastDisplaySentAt = millis() - DISPLAY_RETRY_MS;
  }

  const unsigned long currentMs = millis();
  const bool heartbeatDue =
      currentMs - lastDisplayHeartbeatAt >= DISPLAY_HEARTBEAT_MS;
  const bool retryDue =
      displayAckPending && currentMs - lastDisplaySentAt >= DISPLAY_RETRY_MS;
  if (heartbeatDue || retryDue) {
    sendDisplayTelemetryFrame();
    return;
  }

  // onReceive ISR가 telemetry를 적용할 시간을 준 뒤 상태 응답의 seq를
  // 확인한다. LCD 준비 실패는 표시 문제일 뿐 주행/릴레이 안전정지 조건은
  // 아니므로 여기서는 상태만 보존한다.
  if (displayAckPending &&
      currentMs - lastDisplaySentAt >= DISPLAY_ACK_DELAY_MS) {
    byte actuatorState;
    byte appliedCommand;
    byte appliedSequence;
    if (!readActuatorState(actuatorState, appliedCommand, appliedSequence)) return;
    if ((lastDisplayStatusFlags & DISPLAY_STATUS_VALID) &&
        lastDisplayAckSequence == displaySequence) {
      displayAckPending = false;
    }
  }
}

void copyFlashText(char* output, byte outputSize,
                   const __FlashStringHelper* text) {
  strncpy_P(output, reinterpret_cast<PGM_P>(text), outputSize - 1);
  output[outputSize - 1] = '\0';
}

void setCommandResult(const __FlashStringHelper* result) {
  copyFlashText(commandResult, sizeof(commandResult), result);
}

// 가장 오래된 미전송 이벤트와 가장 최근 이벤트를 각각 보존한다.
// Uno SRAM을 고려한 2단 큐라서 RFID 도착 직후의 완료/오류가 앞 이벤트를
// 바로 덮어쓰는 문제를 줄인다.
void queueRobotReport(const __FlashStringHelper* eventName) {
  char nextEvent[24];
  copyFlashText(nextEvent, sizeof(nextEvent), eventName);
  if (!robotReportPending) {
    strncpy(pendingEvent, nextEvent, sizeof(pendingEvent));
    pendingEvent[sizeof(pendingEvent) - 1] = '\0';
    robotReportPending = true;
    return;
  }
  if (!strcmp(pendingEvent, nextEvent) || !strcmp(deferredEvent, nextEvent)) return;
  strncpy(deferredEvent, nextEvent, sizeof(deferredEvent));
  deferredEvent[sizeof(deferredEvent) - 1] = '\0';
}

// 첫 이벤트 전송이 끝나면 보류 중이던 두 번째 이벤트를 다음 전송 대상으로
// 승격한다. RFID_ARRIVAL 직후 MODULE_COMPLETE가 덮어써지는 것을 막는다.
void finishRobotReport() {
  if (deferredEvent[0]) {
    strncpy(pendingEvent, deferredEvent, sizeof(pendingEvent));
    pendingEvent[sizeof(pendingEvent) - 1] = '\0';
    deferredEvent[0] = '\0';
    robotReportPending = true;
  } else {
    robotReportPending = false;
  }
}

// 현재 내부 상태를 서버가 이해하는 영문 상태명으로 바꾼다.
void copyPhaseName(char* output, byte outputSize) {
  PGM_P name = PSTR("IDLE");
  switch (robotPhase) {
    case PHASE_MOVING: name = PSTR("MOVING"); break;
    case PHASE_MODULE_RUNNING: name = PSTR("MODULE_RUNNING"); break;
    case PHASE_TASK_COMPLETE: name = PSTR("TASK_COMPLETE"); break;
    case PHASE_RETURNING: name = PSTR("RETURNING"); break;
    default: break;
  }
  strncpy_P(output, name, outputSize - 1);
  output[outputSize - 1] = '\0';
}

// 출발 직전에 RC522를 다시 읽어 부팅 후 빠진 전원/배선도 감지한다.
// 0x00/0xFF는 통신 불가 상태이며, 해당 상태로는 RFID 기반 구역 주행을
// 시작하지 않는다.
bool refreshRfidHealth() {
  const byte version = rfid.PCD_ReadRegister(SoftwareMFRC522::VersionReg);
  const bool readyNow = version != 0x00 && version != 0xFF;
  if (readyNow != rfidReady || !readyNow) {
    Serial.print(F("[RFID] health VersionReg=0x"));
    if (version < 0x10) Serial.print('0');
    Serial.println(version, HEX);
  }
  rfidReady = readyNow;
  return rfidReady;
}

bool waitForMotorCommand(byte command, byte sequence) {
  const unsigned long startedAt = millis();
  while (millis() - startedAt < I2C_APPLY_TIMEOUT_MS) {
    byte status;
    byte appliedCommand;
    byte appliedSequence;
    if (!readMotorState(status, appliedCommand, appliedSequence)) {
      delay(10);
      continue;
    }
    lastMotorStatus = status;
    obstaclePauseActive = status == MOTOR_STATUS_OBSTACLE;

    // MotorUno가 이 순번을 실제 적용하기 전의 RUNNING/IDLE/STOP_LINE은
    // 이전 명령의 상태이므로 ACK로 인정하지 않는다.
    if (appliedCommand != command || appliedSequence != sequence) {
      delay(10);
      continue;
    }

    if (command == MOTOR_COMMAND_PROTOCOL_SYNC) {
      return status == MOTOR_STATUS_PROTOCOL_REQUIRED;
    }

    if (status == MOTOR_STATUS_WATCHDOG_TIMEOUT ||
        status == MOTOR_STATUS_INVALID_COMMAND ||
        status == MOTOR_STATUS_UNEXPECTED_MARKER) return false;

    // Calibration 전 STOP도 물리 출력은 안전하게 꺼진 상태다. HOME_SYNC만
    // STATUS_IDLE을 받아야 성공이며, 나머지 이동 명령의 status 7은 거절이다.
    if (command == MOTOR_COMMAND_STOP) {
      return status == MOTOR_STATUS_IDLE ||
             status == MOTOR_STATUS_CALIBRATION_REQUIRED ||
             status == MOTOR_STATUS_PROTOCOL_REQUIRED;
    }
    if (status == MOTOR_STATUS_PROTOCOL_REQUIRED) return false;
    if (command == MOTOR_COMMAND_HOME_SYNC) {
      return status == MOTOR_STATUS_IDLE;
    }
    if (status == MOTOR_STATUS_CALIBRATION_REQUIRED) return false;
    if (command == MOTOR_COMMAND_PAUSE) {
      return status == MOTOR_STATUS_OBSTACLE ||
             status == MOTOR_STATUS_STOP_LINE;
    }
    return status == MOTOR_STATUS_RUNNING ||
           status == MOTOR_STATUS_OBSTACLE ||
           status == MOTOR_STATUS_STOP_LINE;
  }
  return false;
}

bool sendMotorCommandChecked(byte command) {
  motorSequenceCounter = static_cast<byte>(motorSequenceCounter + 1);
  const byte sequence = motorSequenceCounter;
  if (!sendMotorFrame(command, sequence)) return false;
  if (!waitForMotorCommand(command, sequence)) return false;
  acknowledgedMotorCommand = command;
  acknowledgedMotorSequence = sequence;
  return true;
}

void latchRouteUnknown() {
  routeCalibrated = false;
  confirmedStation = STATION_UNKNOWN;
  expectedStation = STATION_UNKNOWN;
  routeAtStation = false;
  rfidDirectionGuardActive = false;
  rfidDirectionClearSeen = false;
}

// 모든 raw STOP 호출이 ACK 실패를 놓치지 않도록 두 STOP 함수에서 사용하는
// 중앙 재시도 등록 함수의 전방 선언이다.
void armStopRetry(StopRetryMode mode, bool motorStopped, bool moduleStopped);

bool stopMotorController() {
  const bool stopped = sendMotorCommandChecked(MOTOR_COMMAND_STOP);
  obstaclePauseActive = false;
  Serial.println(stopped ? F("[I2C MOTOR] STOP confirmed")
                         : F("[I2C MOTOR] STOP failed/no ACK"));
  if (!stopped) armStopRetry(STOP_RETRY_KEEP_FAULT, false, true);
  return stopped;
}

bool startMotorController(RouteHeading heading) {
  obstaclePauseActive = false;
  // command 1은 차체 앞쪽 전진, command 2는 네 바퀴 모두 반대로 돌리는
  // 실제 후진이다. 이전처럼 180도 회전 명령으로 사용하지 않는다.
  const bool reversing = heading == HEADING_HOMEBOUND;
  const byte command = reversing ? MOTOR_COMMAND_RETURN : MOTOR_COMMAND_OUTBOUND;
  const bool started = sendMotorCommandChecked(command);
  Serial.print(F("[I2C MOTOR] command="));
  Serial.print(reversing ? F("REVERSE_HOMEBOUND") : F("FORWARD_OUTBOUND"));
  Serial.println(started ? F(" confirmed") : F(" failed/no ACK"));
  return started;
}

RouteStation stationForZone(const char* zone) {
  if (!strcmp_P(zone, PSTR("HOME"))) return STATION_HOME;
  if (!strcmp_P(zone, PSTR("ZONE2"))) return STATION_ZONE2;
  if (!strcmp_P(zone, PSTR("ZONE99"))) return STATION_ZONE99;
  return STATION_UNKNOWN;
}

bool validRouteStation(RouteStation station) {
  return station >= STATION_HOME && station <= STATION_ZONE99;
}

// 현재 위치/방향에서 목적지까지 계속 직진할 수 있는지 계산한다.
// routeAtStation=false일 때 confirmedStation은 방금 떠난 역,
// expectedStation은 현재 진행 방향에서 만날 다음 역이다.
bool targetAheadOnCurrentSegment(RouteStation destination) {
  if (routeHeading == HEADING_OUTBOUND) return destination >= expectedStation;
  return destination <= expectedStation;
}

// 목적지까지 이동을 시작하거나, 이미 같은 방향으로 주행 중이면 현재 주행을
// 유지한다. 차체는 회전하지 않는다. 목적지가 큰 역 번호 쪽이면 실제 전진,
// 작은 역 번호 쪽이면 네 바퀴를 반대로 돌려 실제 후진한다.
bool startRouteTravel(RouteStation destination) {
  if (!validRouteStation(destination) || !validRouteStation(confirmedStation)) {
    stopMotorController();
    stopModuleController();
    latchRouteUnknown();
    robotPhase = PHASE_TASK_COMPLETE;
    taskActive = false;
    setCommandResult(F("FAILED"));
    queueRobotReport(F("ROUTE_STATE_ERROR"));
    return false;
  }

  const bool movingNow =
      robotPhase == PHASE_MOVING || robotPhase == PHASE_RETURNING;
  RouteHeading nextHeading = routeHeading;
  RouteStation nextExpected = expectedStation;

  if (routeAtStation) {
    if (destination == confirmedStation) return true;
    const RouteHeading desired = destination > confirmedStation
        ? HEADING_OUTBOUND : HEADING_HOMEBOUND;
    nextHeading = desired;
    nextExpected = static_cast<RouteStation>(confirmedStation + nextHeading);
  } else if (targetAheadOnCurrentSegment(destination)) {
    // 복귀 중 ZONE2처럼 지금 진행 방향 앞에 새 목표가 생기면 회전하지 않는다.
    if (movingNow) {
      robotPhase = destination == STATION_HOME ? PHASE_RETURNING : PHASE_MOVING;
      phaseStartedAt = millis();
      queueRobotReport(F("ROUTE_UPDATED"));
      return true;
    }
  } else {
    nextHeading = routeHeading == HEADING_OUTBOUND
        ? HEADING_HOMEBOUND : HEADING_OUTBOUND;
    // 구간 중간에서 되돌아가면 다음 역은 방금 떠난 confirmedStation이다.
    nextExpected = confirmedStation;
  }

  const bool directionChanged = nextHeading != routeHeading;
  if (!startMotorController(nextHeading)) {
    // START 프레임만 적용되고 ACK가 유실됐을 수도 있으므로 즉시 STOP을
    // 재전송한다. 실제 이동 여부를 확신할 수 없어 경로 위치도 UNKNOWN으로 잠근다.
    stopMotorController();
    stopModuleController();
    latchRouteUnknown();
    robotPhase = PHASE_TASK_COMPLETE;
    taskActive = false;
    setCommandResult(F("I2C_ERROR"));
    queueRobotReport(F("MOTOR_START_ERROR"));
    return false;
  }

  routeHeading = nextHeading;
  expectedStation = nextExpected;
  if (directionChanged) {
    rfidDirectionGuardStartedAt = millis();
    rfidDirectionGuardActive = true;
    rfidDirectionClearSeen = false;
  }
  routeAtStation = false;
  robotPhase = destination == STATION_HOME ? PHASE_RETURNING : PHASE_MOVING;
  phaseStartedAt = millis();
  queueRobotReport(F("DISPATCHED"));
  return true;
}

// MotorUno의 2초 독립 watchdog이 동작하지 않도록 주행 중 400ms마다
// KEEPALIVE를 보내고 실제 상태도 함께 읽는다. 이 함수는 ESP 응답 대기
// 루프 안에서도 호출되므로 지연이나 문자열 할당을 하지 않는다.
void serviceMotorLink() {
  if (robotPhase != PHASE_MOVING && robotPhase != PHASE_RETURNING) return;
  if (millis() - lastMotorKeepaliveAt < MOTOR_KEEPALIVE_MS) return;
  lastMotorKeepaliveAt = millis();
  if (!sendMotorFrame(MOTOR_COMMAND_KEEPALIVE,
                      acknowledgedMotorSequence)) {
    motorLinkFaultPending = true;
    return;
  }
  byte status;
  byte appliedCommand;
  byte appliedSequence;
  if (!readMotorState(status, appliedCommand, appliedSequence)) {
    lastMotorStatus = 0xFF;
    motorLinkFaultPending = true;
    return;
  }
  lastMotorStatus = status;
  // 후방 HC-SR04는 MotorUno가 직접 처리한다. 로컬 장애물 정지 상태만
  // 받아 방향 변경 직후 RFID guard의 경과 시간을 멈춘다.
  obstaclePauseActive = status == MOTOR_STATUS_OBSTACLE;
  if (appliedCommand != acknowledgedMotorCommand ||
      appliedSequence != acknowledgedMotorSequence ||
      status == MOTOR_STATUS_IDLE ||
      status == MOTOR_STATUS_WATCHDOG_TIMEOUT ||
      status == MOTOR_STATUS_INVALID_COMMAND ||
      status == MOTOR_STATUS_UNEXPECTED_MARKER ||
      status == MOTOR_STATUS_CALIBRATION_REQUIRED ||
      status == MOTOR_STATUS_PROTOCOL_REQUIRED) {
    motorLinkFaultPending = true;
  }
}

void applyMotorLinkState() {
  serviceMotorLink();
  if (motorLinkFaultPending) {
    motorLinkFaultPending = false;
    const byte faultStatus = lastMotorStatus;
    stopMotorController();
    stopModuleController();
    latchRouteUnknown();
    taskActive = false;
    robotPhase = PHASE_TASK_COMPLETE;
    setCommandResult(F("FAILED"));
    if (faultStatus == MOTOR_STATUS_CALIBRATION_REQUIRED ||
        faultStatus == MOTOR_STATUS_PROTOCOL_REQUIRED) {
      queueRobotReport(F("CALIBRATION_REQUIRED"));
      Serial.println(F("[I2C MOTOR] HOME calibration lost -> safe stop"));
    } else if (faultStatus == MOTOR_STATUS_UNEXPECTED_MARKER) {
      queueRobotReport(F("MOTOR_MARKER_ERROR"));
      Serial.println(F("[I2C MOTOR] unexpected outbound marker -> safe stop"));
    } else {
      queueRobotReport(F("MOTOR_I2C_ERROR"));
      Serial.println(F("[I2C MOTOR] link/status fault -> safe stop"));
    }
    return;
  }
  if ((robotPhase == PHASE_MOVING || robotPhase == PHASE_RETURNING) &&
      lastMotorStatus == MOTOR_STATUS_STOP_LINE) {
    stopMotorController();

    if (manualForwardActive) {
      manualForwardActive = false;
      robotPhase = PHASE_IDLE;
      setCommandResult(F("COMPLETED"));
      queueRobotReport(F("MOTOR_STOP_LINE"));
      Serial.println(F("[MOTOR] manual stop line -> IDLE"));
    // 직선 트랙에서 HOME은 ZONE2보다 안쪽에 있는 종점이다. ZONE2를
    // HOMEBOUND 방향으로 확인한 뒤 만나는 다음 정지선만 HOME으로 인정한다.
    } else if (expectedStation == STATION_HOME && targetStation == STATION_HOME) {
      confirmedStation = STATION_HOME;
      routeAtStation = true;
      taskActive = false;
      robotPhase = PHASE_IDLE;
      setCommandResult(F("COMPLETED"));
      queueRobotReport(F("HOME_ARRIVAL"));
    } else if (targetStation == STATION_HOME &&
               routeHeading == HEADING_HOMEBOUND) {
      // 복귀 중 ZONE2 UID를 놓쳤더라도 MotorUno의 homebound 전용 마커는
      // 물리 HOME을 뜻한다. 위치는 복구하되 누락은 실패 이벤트로 남긴다.
      confirmedStation = STATION_HOME;
      expectedStation = STATION_HOME;
      routeAtStation = true;
      taskActive = false;
      robotPhase = PHASE_IDLE;
      setCommandResult(F("FAILED"));
      queueRobotReport(F("HOME_RFID_MISSED"));
    } else if (taskActive || targetStation == STATION_HOME) {
      // ZONE2/ZONE99는 정지선이 아니라 주행 중 RFID로만 확정한다. HOME 이외의
      // STOP_LINE은 태그를 놓쳤거나 트랙을 잘못 읽은 것이므로 재출발하지 않는다.
      stopModuleController();
      latchRouteUnknown();
      taskActive = false;
      robotPhase = PHASE_TASK_COMPLETE;
      setCommandResult(F("FAILED"));
      queueRobotReport(F("UNEXPECTED_STOP_LINE"));
    } else {
      robotPhase = PHASE_IDLE;
      setCommandResult(F("COMPLETED"));
      queueRobotReport(F("MOTOR_STOP_LINE"));
    }
    phaseStartedAt = millis();
  }
}

void stopSafelyForServerLoss() {
  if (robotPhase == PHASE_IDLE || robotPhase == PHASE_TASK_COMPLETE) return;
  const bool motorStopped = stopMotorController();
  const bool moduleStopped = stopModuleController();
  // Motor STOP ACK가 확인되고 위치와 다음 역이 모두 유효할 때만 같은 서버
  // 명령을 1회 재개한다. 전진/후진 방향은 routeHeading으로 그대로 복구한다.
  retrySameRevisionAllowed = motorStopped && moduleStopped &&
                             validRouteStation(confirmedStation) &&
                             validRouteStation(expectedStation);
  if (!motorStopped) latchRouteUnknown();
  if (!motorStopped || !moduleStopped) {
    retrySameRevisionAllowed = false;
    armStopRetry(STOP_RETRY_SERVER_RECOVERY, motorStopped, moduleStopped);
  }
  taskActive = false;
  manualForwardActive = false;
  robotPhase = PHASE_TASK_COMPLETE;
  // 서버 allowlist가 이해하는 실패 결과를 사용하고 원인은 event로 보존한다.
  setCommandResult(F("FAILED"));
  queueRobotReport(F("SERVER_OFFLINE_STOP"));
  Serial.println(F("[SAFETY] server unavailable during task -> all outputs stopped"));
}

bool waitForActuatorCommand(byte command, byte sequence, byte expectedStatus) {
  const unsigned long startedAt = millis();
  while (millis() - startedAt < I2C_APPLY_TIMEOUT_MS) {
    byte status;
    byte appliedCommand;
    byte appliedSequence;
    if (readActuatorState(status, appliedCommand, appliedSequence)) {
      lastActuatorStatus = status;
      if (appliedSequence != sequence) {
        delay(10);
        continue;
      }
      if (status == ACTUATOR_STATUS_ERROR) return false;
      if (status == expectedStatus && appliedCommand == command) return true;
    }
    delay(10);
  }
  return false;
}

bool stopModuleController() {
  byte sequence;
  const bool sent = sendActuatorFrame(ACTUATOR_COMMAND_STOP, sequence);
  const bool stopped = sent &&
      waitForActuatorCommand(ACTUATOR_COMMAND_STOP, sequence,
                             ACTUATOR_STATUS_IDLE);
  Serial.println(stopped ? F("[I2C ACTUATOR] STOP confirmed")
                         : F("[I2C ACTUATOR] STOP failed/no ACK"));
  if (!stopped) armStopRetry(STOP_RETRY_KEEP_FAULT, true, false);
  return stopped;
}

// MotorUno와 ActuatorUno가 모두 STOP/IDLE을 ACK할 때까지 로컬 재시도를
// 유지한다. ACK 전에는 서버에 COMPLETED를 보고하지 않는다.
void armStopRetry(StopRetryMode mode, bool motorStopped, bool moduleStopped) {
  const bool alreadyRetrying = stopRetryMode != STOP_RETRY_NONE;
  // raw STOP의 KEEP_FAULT는 뒤따르는 호출부의 구체적인 완료 정책으로
  // 승격할 수 있지만, 재시도 중의 KEEP_FAULT가 기존 정책을 덮어쓰면 안 된다.
  if (!alreadyRetrying || mode != STOP_RETRY_KEEP_FAULT) stopRetryMode = mode;
  stopMotorRetryNeeded = stopMotorRetryNeeded || !motorStopped;
  stopActuatorRetryNeeded = stopActuatorRetryNeeded || !moduleStopped;
  // Actuator만 미확인이면 차량 위치는 유지할 수 있지만, Motor STOP ACK가
  // 없으면 계속 움직였을 수 있으므로 경로 위치를 즉시 폐기한다.
  if (!motorStopped) latchRouteUnknown();
  if (!alreadyRetrying) lastStopRetryAt = millis() - STOP_RETRY_INTERVAL_MS;
  robotPhase = PHASE_TASK_COMPLETE;
  setCommandResult(F("FAILED"));
}

void serviceStopRetry() {
  if (stopRetryMode == STOP_RETRY_NONE ||
      millis() - lastStopRetryAt < STOP_RETRY_INTERVAL_MS) return;

  lastStopRetryAt = millis();
  Serial.println(F("[SAFETY] retrying Motor/Actuator STOP confirmation"));
  if (stopMotorRetryNeeded && stopMotorController()) {
    stopMotorRetryNeeded = false;
  }
  if (stopActuatorRetryNeeded && stopModuleController()) {
    stopActuatorRetryNeeded = false;
  }
  if (stopMotorRetryNeeded) latchRouteUnknown();
  if (stopMotorRetryNeeded || stopActuatorRetryNeeded) return;

  const StopRetryMode completedMode = stopRetryMode;
  stopRetryMode = STOP_RETRY_NONE;
  if (completedMode == STOP_RETRY_SERVER_RECOVERY) {
    robotPhase = PHASE_TASK_COMPLETE;
    setCommandResult(F("FAILED"));
    retrySameRevisionAllowed = validRouteStation(confirmedStation) &&
                               validRouteStation(expectedStation);
    queueRobotReport(F("SERVER_OFFLINE_STOP"));
  } else if (completedMode == STOP_RETRY_TO_IDLE) {
    robotPhase = PHASE_IDLE;
    setCommandResult(F("COMPLETED"));
    queueRobotReport(F("STOP_CONFIRMED"));
  } else {
    // 오류 원인은 해소된 것이 아니므로 출력 정지만 확인하고 FAILED를 유지한다.
    robotPhase = PHASE_TASK_COMPLETE;
    setCommandResult(F("FAILED"));
    queueRobotReport(F("STOP_CONFIRMED"));
  }
  Serial.println(F("[SAFETY] all STOP outputs confirmed"));
}

bool requireHomeCalibration() {
  if (routeCalibrated) return true;
  stopMotorController();
  stopModuleController();
  latchRouteUnknown();
  taskActive = false;
  robotPhase = PHASE_TASK_COMPLETE;
  setCommandResult(F("FAILED"));
  queueRobotReport(F("CALIBRATION_REQUIRED"));
  Serial.println(F("[CALIBRATION] place at HOME facing ZONE2, then sync"));
  return false;
}

bool performHomeCalibration() {
  taskActive = false;
  manualForwardActive = false;
  strcpy_P(targetZone, PSTR("HOME"));
  strcpy_P(targetAction, PSTR("NONE"));

  // 보정 명령이 주행 중 들어와도 Actuator ACK를 기다리는 동안 차가 더 가지
  // 않도록 Motor STOP을 가장 먼저 확인한다. 두 출력 보드가 모두 정지한 뒤에만
  // HOME 마커를 읽는 stationary HOME_SYNC를 보낸다.
  // versioned 이동 명령 의미가 같은 4모터 v2인지 먼저 확인한다. 구형 MotorUno는
  // command 7을 invalid로 정지하므로 혼합 펌웨어로는 보정이 열리지 않는다.
  const bool protocolReady =
      sendMotorCommandChecked(MOTOR_COMMAND_PROTOCOL_SYNC);
  const bool motorStopped = stopMotorController();
  const bool moduleStopped = stopModuleController();
  const bool homeSynced = protocolReady && motorStopped && moduleStopped &&
      sendMotorCommandChecked(MOTOR_COMMAND_HOME_SYNC);
  if (!homeSynced) {
    stopMotorController();
    latchRouteUnknown();
    robotPhase = PHASE_TASK_COMPLETE;
    setCommandResult(F("FAILED"));
    queueRobotReport(F("HOME_CALIBRATION_FAILED"));
    Serial.println(F("[CALIBRATION] failed: HOME marker needs IR HIGH/HIGH"));
    return false;
  }

  routeCalibrated = true;
  confirmedStation = STATION_HOME;
  expectedStation = STATION_ZONE2;
  targetStation = STATION_HOME;
  routeHeading = HEADING_OUTBOUND;
  routeAtStation = true;
  lastAcceptedRfidStation = STATION_UNKNOWN;
  rfidDirectionGuardActive = false;
  rfidDirectionClearSeen = false;
  obstaclePauseActive = false;
  robotPhase = PHASE_IDLE;
  phaseStartedAt = millis();
  setCommandResult(F("COMPLETED"));
  queueRobotReport(F("HOME_CALIBRATED"));
  Serial.println(F("[CALIBRATION] HOME synced; heading=ZONE2"));
  return true;
}

bool startPlaceholderMovement() {
  if (!requireHomeCalibration()) return false;
  if (!stopModuleController()) {
    stopMotorController();
    robotPhase = PHASE_TASK_COMPLETE;
    setCommandResult(F("I2C_ERROR"));
    queueRobotReport(F("ACTUATOR_STOP_ERROR"));
    return false;
  }
  targetStation = stationForZone(targetZone);
  if (targetStation == STATION_UNKNOWN) {
    stopMotorController();
    taskActive = false;
    robotPhase = PHASE_TASK_COMPLETE;
    setCommandResult(F("FAILED"));
    queueRobotReport(F("INVALID_TARGET"));
    return false;
  }

  // 이미 확인된 목표 역에 정차 중이면 다시 이동/RFID 확인을 하지 않는다.
  if (routeAtStation && confirmedStation == targetStation) {
    return startPlaceholderModule(false);
  }

  // 구역은 라인이 아니라 RFID UID로 확정하므로 리더가 부팅 진단에
  // 실패한 상태에서는 자동 임무를 출발시키지 않는다.
  if (!refreshRfidHealth()) {
    stopMotorController();
    taskActive = false;
    robotPhase = PHASE_TASK_COMPLETE;
    setCommandResult(F("FAILED"));
    queueRobotReport(F("RFID_NOT_READY"));
    return false;
  }

  return startRouteTravel(targetStation);
}

bool startPlaceholderReturn() {
  if (!requireHomeCalibration()) return false;
  if (!stopModuleController()) {
    stopMotorController();
    robotPhase = PHASE_TASK_COMPLETE;
    setCommandResult(F("I2C_ERROR"));
    queueRobotReport(F("ACTUATOR_STOP_ERROR"));
    return false;
  }
  targetStation = STATION_HOME;
  if (routeAtStation && confirmedStation == STATION_HOME) {
    const bool motorStopped = stopMotorController();
    const bool moduleStopped = stopModuleController();
    robotPhase = PHASE_IDLE;
    taskActive = false;
    if (!motorStopped) latchRouteUnknown();
    if (motorStopped && moduleStopped) {
      setCommandResult(F("COMPLETED"));
    } else {
      armStopRetry(STOP_RETRY_TO_IDLE, motorStopped, moduleStopped);
    }
    queueRobotReport(F("HOME_ALREADY"));
    return motorStopped && moduleStopped;
  }

  // ZONE99에서 HOME으로 갈 때 중간 ZONE2를 RFID로 확인해야 한다.
  // 부팅 후 리더가 분리된 경우에도 출발 직전 여기서 다시 차단한다.
  if (!refreshRfidHealth()) {
    stopMotorController();
    taskActive = false;
    robotPhase = PHASE_TASK_COMPLETE;
    setCommandResult(F("FAILED"));
    queueRobotReport(F("RFID_NOT_READY"));
    return false;
  }

  // 후진 시작/장애물 정지는 MotorUno D2/A1의 로컬 HC-SR04가 담당한다.
  // SensorUno는 I2C ACK와 status=OBSTACLE만 관찰한다.
  return startRouteTravel(STATION_HOME);
}

bool startPlaceholderModule(bool reportArrival) {
  if (!stopMotorController()) {
    stopModuleController();
    latchRouteUnknown();
    robotPhase = PHASE_TASK_COMPLETE;
    setCommandResult(F("I2C_ERROR"));
    queueRobotReport(F("MOTOR_STOP_ERROR"));
    return false;
  }
  if (reportArrival) queueRobotReport(F("RFID_ARRIVAL"));
  phaseStartedAt = millis();
  TRACE_PRINT(F("[MODULE] START action="));
  TRACE_PRINT(targetAction);
  TRACE_PRINT(F(", zone="));
  TRACE_PRINTLN(targetZone);

  if (!strcmp(targetAction, "NONE")) {
    const bool stopped = stopModuleController();
    robotPhase = PHASE_TASK_COMPLETE;
    if (stopped) {
      setCommandResult(F("COMPLETED"));
      queueRobotReport(F("MOVE_ONLY_COMPLETE"));
      TRACE_PRINTLN(F("[MODULE] move-only complete"));
      return true;
    }
    setCommandResult(F("I2C_ERROR"));
    queueRobotReport(F("ACTUATOR_STOP_ERROR"));
    return false;
  }

  byte actuatorCommand = ACTUATOR_COMMAND_STOP;
  if (!strcmp(targetAction, "HUMIDIFY")) {
    actuatorCommand = ACTUATOR_COMMAND_HUMIDIFY;
    TRACE_PRINTLN(F("[ACT] HUMIDIFY"));
  } else if (!strcmp(targetAction, "DEHUMIDIFY")) {
    actuatorCommand = ACTUATOR_COMMAND_DEHUMIDIFY;
    TRACE_PRINTLN(F("[ACT] DEHUMIDIFY"));
  } else {
    Serial.println(F("[MODULE PLACEHOLDER] unknown action -> request cancelled"));
    robotPhase = PHASE_TASK_COMPLETE;
    setCommandResult(F("INVALID_ACTION"));
    queueRobotReport(F("INVALID_ACTION"));
    return false;
  }

  lastActuatorStatus = 0xFF;
  lastModuleStatusPollAt = 0;
  expectedActuatorCommand = actuatorCommand;
  actuatorRunningObserved = false;
  byte actuatorSequence;
  if (!sendActuatorFrame(actuatorCommand, actuatorSequence)) {
    // endTransmission 오류는 START가 슬레이브에 전혀 적용되지 않았다는
    // 보장이 없다. 즉시 STOP+IDLE ACK를 요구하며, 확인 실패는
    // stopModuleController()가 STOP_RETRY_KEEP_FAULT로 계속 재시도한다.
    stopModuleController();
    robotPhase = PHASE_TASK_COMPLETE;
    setCommandResult(F("I2C_ERROR"));
    queueRobotReport(F("ACTUATOR_I2C_ERROR"));
    Serial.println(F("[I2C ACTUATOR] command transmission failed"));
    return false;
  }
  expectedActuatorSequence = actuatorSequence;
  if (!waitForActuatorCommand(actuatorCommand, actuatorSequence,
                              ACTUATOR_STATUS_RUNNING)) {
    // START ACK를 읽지 못한 경우도 실제 릴레이 상태가 모호하므로 같은
    // STOP 확인/로컬 재시도 경로를 사용한다.
    stopModuleController();
    robotPhase = PHASE_TASK_COMPLETE;
    setCommandResult(F("ACT_START_ERROR"));
    queueRobotReport(F("ACTUATOR_START_ERROR"));
    Serial.println(F("[I2C ACTUATOR] RUNNING ACK missing or ERROR"));
    return false;
  }
  actuatorRunningObserved = true;
  robotPhase = PHASE_MODULE_RUNNING;
  setCommandResult(F("EXECUTING"));
  TRACE_PRINTLN(F("[ACT] RUNNING ACK"));
  return true;
}

// 0=계속 실행, 1=정상 완료, 2=즉시 오류
byte pollActuatorTask() {
  if (millis() - lastModuleStatusPollAt < MODULE_STATUS_POLL_MS) return 0;
  lastModuleStatusPollAt = millis();
  byte status;
  byte appliedCommand;
  byte appliedSequence;
  if (!readActuatorState(status, appliedCommand, appliedSequence)) {
    Serial.println(F("[I2C ACTUATOR] NO RESPONSE -> immediate fault"));
    return 2;
  }
  if (appliedSequence != expectedActuatorSequence) return 2;
  if (status != lastActuatorStatus) {
    TRACE_PRINT(F("[ACT] status="));
    TRACE_PRINT(status);
    TRACE_PRINT(F(" cmd="));
    TRACE_PRINTLN(appliedCommand);
    lastActuatorStatus = status;
  }
  if (status == ACTUATOR_STATUS_ERROR) return 2;
  if (status == ACTUATOR_STATUS_RUNNING) {
    if (appliedCommand != expectedActuatorCommand) return 2;
    actuatorRunningObserved = true;
    return 0;
  }
  if (status == ACTUATOR_STATUS_DONE) {
    // 새 명령에서 같은 command의 RUNNING을 실제 관찰했고, DONE 응답도
    // 같은 command를 echo할 때만 완료로 인정한다.
    return actuatorRunningObserved && appliedCommand == expectedActuatorCommand
        ? 1 : 2;
  }
  // 실행 도중 IDLE이나 알 수 없는 값으로 바뀌면 출력이 예기치 않게 꺼진 것이다.
  return 2;
}

void updatePlaceholderStateMachine() {
  const unsigned long elapsed = millis() - phaseStartedAt;

  if (robotPhase == PHASE_MOVING && elapsed >= MOVE_SAFETY_TIMEOUT_MS) {
    stopMotorController();
    if (taskActive) {
      stopModuleController();
      // 목표 RFID를 못 찾았다면 이미 예상 역을 지나쳤을 수 있다.
      latchRouteUnknown();
      taskActive = false;
      robotPhase = PHASE_TASK_COMPLETE;
      setCommandResult(F("FAILED"));
      queueRobotReport(F("RFID_NOT_FOUND"));
    } else {
      // 수동 직진은 경로 위치를 보장하지 않으며 시작 시 이미 UNKNOWN이다.
      latchRouteUnknown();
      robotPhase = PHASE_IDLE;
      setCommandResult(F("COMPLETED"));
      queueRobotReport(F("MANUAL_MOVE_TIMEOUT"));
    }
    phaseStartedAt = millis();
    Serial.println(F("[RFID] moving scan timeout -> safe stop"));
  } else if (robotPhase == PHASE_MODULE_RUNNING) {
    const byte actuatorPoll = pollActuatorTask();
    if (actuatorPoll == 1) {
      stopModuleController();
      robotPhase = PHASE_TASK_COMPLETE;
      setCommandResult(F("COMPLETED"));
      queueRobotReport(F("MODULE_COMPLETE"));
      phaseStartedAt = millis();
      TRACE_PRINT(F("[MODULE] COMPLETE action="));
      TRACE_PRINT(targetAction);
      TRACE_PRINT(F(", zone="));
      TRACE_PRINTLN(targetZone);
    } else if (actuatorPoll == 2 || elapsed >= MODULE_TIMEOUT_MS) {
      stopModuleController();
      robotPhase = PHASE_TASK_COMPLETE;
      setCommandResult(F("FAILED"));
      queueRobotReport(actuatorPoll == 2 ? F("ACTUATOR_STATUS_ERROR")
                                         : F("MODULE_TIMEOUT"));
      phaseStartedAt = millis();
      Serial.println(F("[I2C ACTUATOR] task aborted by status/timeout fault"));
    }
  } else if (robotPhase == PHASE_RETURNING &&
             elapsed >= RETURN_SAFETY_TIMEOUT_MS) {
    stopMotorController();
    // HOME 표식을 못 찾았으므로 이후 자동 재출발에 이전 위치를 쓰지 않는다.
    latchRouteUnknown();
    robotPhase = PHASE_TASK_COMPLETE;
    setCommandResult(F("FAILED"));
    queueRobotReport(F("RETURN_TIMEOUT"));
    phaseStartedAt = millis();
    Serial.println(F("[MOTOR] HOME stop line not found before safety timeout"));
  }
}

void checkUsbTestCommands() {
  if (!ENABLE_SERIAL_TEST_COMMANDS || !Serial.available()) return;

  const char command = static_cast<char>(Serial.read());
  if ((command == 'X' || command == 'x') &&
      ENABLE_USB_ZONE2_MISSION_TEST) {
    const bool wasMoving = robotPhase == PHASE_MOVING ||
                           robotPhase == PHASE_RETURNING;
    usbZone2MissionActive = false;
    taskActive = false;
    manualForwardActive = false;
    const bool motorStopped = stopMotorController();
    const bool moduleStopped = stopModuleController();
    if (wasMoving || !motorStopped) latchRouteUnknown();
    robotPhase = PHASE_TASK_COMPLETE;
    setCommandResult(motorStopped && moduleStopped ? F("COMPLETED")
                                                   : F("FAILED"));
    lastPollAt = millis();
    Serial.println(F("[USB MISSION] ABORT -> outputs STOP"));
    return;
  }
  if (command == 'C' || command == 'c') {
    if (usbZone2MissionActive) {
      Serial.println(F("[CALIBRATION] rejected: USB mission active"));
      return;
    }
    Serial.println(F("[CALIBRATION] USB HOME_SYNC requested"));
    performHomeCalibration();
    return;
  }
  if ((command == 'Z' || command == 'z') &&
      ENABLE_USB_ZONE2_MISSION_TEST) {
    if (usbZone2MissionActive || stopRetryMode != STOP_RETRY_NONE) {
      Serial.println(F("[USB MISSION] rejected: busy"));
      return;
    }
    if (!routeCalibrated) {
      Serial.println(F("[USB MISSION] rejected: run c at HOME first"));
      return;
    }

    manualForwardActive = false;
    taskActive = true;
    strcpy_P(targetZone, PSTR("ZONE2"));
    strcpy_P(targetAction, PSTR("HUMIDIFY"));
    targetStation = STATION_ZONE2;
    setCommandResult(F("EXECUTING"));
    usbZone2MissionActive = true;
    lastPollAt = millis();
    Serial.println(F("[USB MISSION] ZONE2 HUMIDIFY start"));
    if (!startPlaceholderMovement()) {
      usbZone2MissionActive = false;
      taskActive = false;
      Serial.println(F("[USB MISSION] start failed"));
    }
    return;
  }
  if (command == '?') {
    char phaseName[20];
    copyPhaseName(phaseName, sizeof(phaseName));
    byte motorStatus;
    byte motorCommand;
    byte motorSequence;
    byte actuatorState;
    byte actuatorCommand;
    byte actuatorSequence;
    const bool motorOk =
        readMotorState(motorStatus, motorCommand, motorSequence);
    const bool actuatorOk = readActuatorState(
        actuatorState, actuatorCommand, actuatorSequence);
    const byte rfidVersion =
        rfid.PCD_ReadRegister(SoftwareMFRC522::VersionReg);

    Serial.print(F("[DIAG] P="));
    Serial.print(phaseName);
    Serial.print(F(" Z="));
    Serial.print(targetZone);
    Serial.print(F(" A="));
    Serial.print(targetAction);
    Serial.print(F(" W="));
    Serial.print(wifiReady ? 1 : 0);
    Serial.print(F(" C="));
    Serial.print(routeCalibrated ? 1 : 0);
    Serial.print(F(" U="));
    Serial.print(usbZone2MissionActive ? 1 : 0);
    Serial.print(F(" M="));
    Serial.print(motorOk ? 1 : 0);
    Serial.print(',');
    Serial.print(motorStatus);
    Serial.print(',');
    Serial.print(motorCommand);
    Serial.print(',');
    Serial.print(motorSequence);

    Serial.print(F(" X="));
    Serial.print(actuatorOk ? 1 : 0);
    Serial.print(',');
    Serial.print(actuatorState);
    Serial.print(',');
    Serial.print(actuatorCommand);
    Serial.print(',');
    Serial.print(actuatorSequence);
    Serial.print(F(" L="));
    Serial.print(lastDisplayAckSequence);
    Serial.print(',');
    Serial.print(lastDisplayStatusFlags, HEX);
    Serial.print(F(" R="));
    Serial.println(rfidVersion, HEX);
    return;
  }
  if (command != 'T' && command != 't') return;

  Serial.println(F("[RFID TEST] UI requested a placeholder RFID tag"));
  if (!manualForwardActive && validRouteStation(expectedStation) &&
      (robotPhase == PHASE_MOVING || robotPhase == PHASE_RETURNING)) {
    processRouteRfid(expectedStation);
  } else {
    Serial.println(F("[RFID TEST] ignored: route is not moving"));
  }
}

void clearEsp() {
  while (esp8266.available()) esp8266.read();
}

bool waitFor(const char* expected, unsigned long timeoutMs,
             const char* alternative = nullptr) {
  uint16_t index = 0;
  espBusySeen = false;
  espBuffer[0] = '\0';
  const unsigned long startedAt = millis();

  while (millis() - startedAt < timeoutMs) {
    while (esp8266.available()) {
      const char c = static_cast<char>(esp8266.read());
      // reset/접속 응답이 128바이트를 넘어도 마지막 ready/OK를 계속 찾는다.
      // 가득 찬 이전 조각은 버리고 새 조각부터 다시 수집한다.
      if (index >= sizeof(espBuffer) - 1) index = 0;
      espBuffer[index++] = c;
      espBuffer[index] = '\0';
      if (strstr(espBuffer, expected) ||
          (alternative && strstr(espBuffer, alternative))) return true;
      if (strstr(espBuffer, "busy s") || strstr(espBuffer, "busy p")) {
        espBusySeen = true;
        return false;
      }
      if (strstr(espBuffer, "ERROR") || strstr(espBuffer, "FAIL")) return false;
    }
    applyMotorLinkState();
    updatePlaceholderStateMachine();
    checkRfidArrival();
    // 위 로컬 처리 중 안전 STOP 재시도가 새로 걸릴 수 있다. ESP 응답을
    // 계속 기다리면 500ms 재시도가 수 초씩 밀리므로 즉시 로컬 정지를
    // 서비스하고 네트워크 대기를 빠져나간다.
    if (stopRetryMode != STOP_RETRY_NONE) {
      serviceStopRetry();
      // 이번 반복에서 fault를 본 HTTP/AT 응답은 정지가 곧 확인됐더라도
      // 폐기한다. fault 이전 command를 같은 call stack에서 실행하지 않는다.
      return false;
    }
  }
  return false;
}

bool sendAt(const char* command, const char* expected, unsigned long timeoutMs,
             const char* alternative = nullptr) {
  clearEsp();

#if VERBOSE_NETWORK_LOGS
  Serial.print(F("[ESP TX] "));
  // 시리얼 로그에 Wi-Fi 비밀번호가 그대로 노출되지 않도록 접속 명령만 가린다.
  if (!strncmp(command, "AT+CWJAP=", 9)) {
    Serial.print(F("AT+CWJAP=\""));
    Serial.print(WIFI_SSID);
    Serial.println(F("\",\"********\""));
  } else {
    Serial.println(command);
  }
#endif

  esp8266.print(command);
  esp8266.print(F("\r\n"));
  const bool success = waitFor(expected, timeoutMs, alternative);

#if VERBOSE_NETWORK_LOGS
  Serial.print(F("[ESP RX] "));
  Serial.println(espBuffer[0] ? espBuffer : "(no response)");
  Serial.print(F("[ESP RESULT] "));
  Serial.println(success ? F("OK") : F("FAIL"));
#else
  if (!success) Serial.println(F("[ESP] AT command failed"));
#endif
  return success;
}

// TCP 단계가 중간에 실패하면 ESP-01 내부 연결이 남아 다음 CIPSTART가
// ALREADY CONNECTED로 고착될 수 있다. 성공/이미 닫힘을 모두 정상 정리로 본다.
bool closeTcpAfterFailure() {
  // 직전 단계가 이미 busy를 확인했다면 CIPCLOSE조차 추가 전송하지 않는다.
  // busy가 아니었던 실패에서만 stale socket 정리를 한 번 시도한다.
  if (!espBusySeen) sendAt("AT+CIPCLOSE", "OK", 1500, "ERROR");
  if (!espBusySeen) return true;

  wifiReady = false;
  lastWifiReconnectAttemptAt = millis();
  Serial.println(F("[ESP] send busy -> quiet reconnect backoff"));
  return false;
}

bool connectWifi() {
  Serial.println(F("[WIFI] ===== connection start ====="));
  bool atReady = sendAt("AT", "OK", 2500);
  if (!atReady && espBusySeen) {
    // busy 상태의 AT+RST/CIPCLOSE/+++는 복구 명령이 아니라 추가 입력일 뿐이다.
    // 다른 로컬 제어를 계속 서비스하며 다음 15초 reconnect 때 다시 확인한다.
    Serial.println(F("[WIFI] ESP send busy -> quiet backoff"));
    return false;
  }

  if (!atReady) {
    // 응답이 전혀 없을 때에만 transparent mode 탈출을 한 번 시도한다.
    // +++ 뒤에는 AT 명령 전 최소 1초 guard time을 둔다.
    clearEsp();
    delay(25);
    esp8266.print(F("+++"));
    delay(1200);
    atReady = sendAt("AT", "OK", 2500);
  }

  if (!atReady) {
    Serial.println(F("[WIFI] ESP-01 AT communication failed"));
    return false;
  }

  // 정상 ESP를 매 reconnect마다 재부팅하지 않는다. echo와 transparent mode를
  // 정리한 뒤 남은 단일 TCP 소켓만 닫아 재사용 가능한 상태로 만든다.
  if (!sendAt("ATE0", "OK", 2000) && espBusySeen) return false;
  if (!sendAt("AT+CIPMODE=0", "OK", 2000, "no change") && espBusySeen) {
    return false;
  }
  if (!closeTcpAfterFailure()) return false;
  if (!sendAt("AT+CIPMUX=0", "OK", 3000, "no change")) {
    Serial.println(F("[WIFI] single TCP mode setup failed"));
    return false;
  }
  if (!sendAt("AT+CWMODE=1", "OK", 3000, "no change")) {
    Serial.println(F("[WIFI] station mode setup failed"));
    return false;
  }

  snprintf_P(atBuffer, sizeof(atBuffer), PSTR("AT+CWJAP=\"%s\",\"%s\""),
             WIFI_SSID, WIFI_PASSWORD);
  Serial.print(F("[WIFI] connecting SSID -> "));
  Serial.println(WIFI_SSID);
  if (!sendAt(atBuffer, "OK", 30000)) {
    Serial.println(F("[WIFI] AP connection failed"));
    return false;
  }
  Serial.println(F("[WIFI] AP connection successful"));

  if (!sendAt("AT+CIFSR", "OK", 4000)) {
    Serial.println(F("[WIFI] IP address query failed"));
    return false;
  }
  Serial.println(F("[WIFI] IP address is shown in AT+CIFSR response above"));
  Serial.println(F("[WIFI] ===== connection complete ====="));
  return true;
}

bool collectHttpResponse() {
  uint16_t index = 0;
  bool received = false;
  bool httpResponseStarted = false;
  bool httpOk = false;
  bool headersComplete = false;
  bool bodyStarted = false;
  bool bodyComplete = false;
  byte headerEndMatch = 0;
  unsigned long lastDataAt = millis();
  const unsigned long startedAt = millis();
  espBusySeen = false;
  espBuffer[0] = '\0';
  // 이전 AT 거래에서 남은 sticky overflow flag를 지운다. 이번 HTTP 응답에서
  // 한 바이트라도 유실되면 부분 JSON을 명령으로 실행하지 않는다.
  (void)esp8266.overflow();

  while (millis() - startedAt < 7000) {
    while (esp8266.available()) {
      const char c = static_cast<char>(esp8266.read());
      received = true;
      lastDataAt = millis();
      if (!bodyStarted) {
        if (!httpOk) {
          // SEND OK/+IPD와 HTTP 상태 줄까지만 임시 보관한다. 200 OK를 찾은
          // 뒤에는 긴 헤더를 계속 strstr 하지 않아 SoftwareSerial을 즉시 비운다.
          if (index < sizeof(espBuffer) - 1) {
            espBuffer[index++] = c;
            espBuffer[index] = '\0';
          }
          if (strstr(espBuffer, "busy s") || strstr(espBuffer, "busy p")) {
            espBusySeen = true;
            wifiReady = false;
            lastWifiReconnectAttemptAt = millis();
            return false;
          }
          // Python 기본 서버는 HTTP/1.0, 다른 서버는 HTTP/1.1을 사용할 수 있다.
          if (strstr(espBuffer, "HTTP/1.")) {
            httpResponseStarted = true;
            if (strstr(espBuffer, "HTTP/1.0 200") ||
                strstr(espBuffer, "HTTP/1.1 200")) {
              httpOk = true;
              index = 0;
              espBuffer[0] = '\0';
            }
          }
        } else if (!headersComplete) {
          // JSON의 첫 '{' 한 바이트에 의존하지 않고 HTTP 헤더 끝을 찾는다.
          // 헤더와 본문이 서로 다른 +IPD 조각이어도 본문 경계를 잃지 않는다.
          if ((headerEndMatch == 0 && c == '\r') ||
              (headerEndMatch == 2 && c == '\r')) {
            ++headerEndMatch;
          } else if ((headerEndMatch == 1 || headerEndMatch == 3) && c == '\n') {
            ++headerEndMatch;
          } else {
            headerEndMatch = c == '\r' ? 1 : 0;
          }
          if (headerEndMatch == 4) {
            headersComplete = true;
            index = 0;
            espBuffer[0] = '\0';
          }
        } else if (c == '{') {
          // 헤더 완료와 JSON 시작을 별도로 확인한다. '{'가 유실된 응답의
          // key/value 조각만 우연히 남아도 자동차 명령으로 실행하지 않는다.
          bodyStarted = true;
          index = 0;
          espBuffer[0] = '\0';
          espBuffer[index++] = c;
          espBuffer[index] = '\0';
        }
      } else if (!bodyComplete) {
        // command JSON은 128바이트 안에 들어온다. status 응답은 더 길 수
        // 있으므로 앞부분은 보존하고, 저장 공간이 차도 stream의 '}'는 확인한다.
        if (index < sizeof(espBuffer) - 1) {
          espBuffer[index++] = c;
          espBuffer[index] = '\0';
        }
        if (c == '}') bodyComplete = true;
      }
    }
    if (esp8266.overflow()) {
      Serial.println(F("[HTTP] ESP RX overflow -> discard response"));
      return false;
    }
    applyMotorLinkState();
    updatePlaceholderStateMachine();
    checkRfidArrival();
    // HTTP 수신 중에도 Motor/I2C/RFID 오류가 STOP latch를 만들 수 있다.
    // 안전 출력 확인을 네트워크 타임아웃보다 항상 우선한다.
    if (stopRetryMode != STOP_RETRY_NONE) {
      serviceStopRetry();
      // 안전 fault 전후의 TCP 응답을 섞지 않고 다음 poll에서 새로 요청한다.
      return false;
    }
    // SEND OK 뒤 실제 HTTP 응답이 늦게 올 수 있으므로 HTTP가 시작된 뒤에만 종료한다.
    if (httpResponseStarted && millis() - lastDataAt > 500) break;
  }
  // 200 헤더만 받고 JSON이 없거나 잘린 경우를 성공으로 오판하지 않는다.
  if (httpOk && (!bodyStarted || !bodyComplete)) {
    Serial.println(F("[HTTP] JSON body incomplete"));
  }
  return received && httpOk && bodyStarted && bodyComplete;
}

bool fetchCommandResponse() {
#if VERBOSE_NETWORK_LOGS
  Serial.println(F("[SERVER] ===== command request start ====="));
  Serial.print(F("[SERVER] destination -> http://"));
  Serial.print(SERVER_HOST);
  Serial.print(':');
  Serial.println(SERVER_PORT);
#endif

  snprintf_P(atBuffer, sizeof(atBuffer), PSTR("AT+CIPSTART=\"TCP\",\"%s\",%u"),
             SERVER_HOST, SERVER_PORT);
  if (!sendAt(atBuffer, "CONNECT", 10000, "Linked")) {
    Serial.println(F("[SERVER] TCP CONNECT ERROR"));
    closeTcpAfterFailure();
    return false;
  }
#if VERBOSE_NETWORK_LOGS
  Serial.println(F("[SERVER] TCP connected"));
#endif

  const int requestLength = snprintf_P(
      requestBuffer, sizeof(requestBuffer),
      PSTR("GET /api/robot/command HTTP/1.0\r\n\r\n"));
  if (requestLength < 0 || requestLength >= static_cast<int>(sizeof(requestBuffer))) {
    Serial.println(F("[SERVER] command request buffer overflow"));
    closeTcpAfterFailure();
    return false;
  }
  snprintf_P(atBuffer, sizeof(atBuffer), PSTR("AT+CIPSEND=%u"),
             static_cast<unsigned int>(requestLength));
  if (!sendAt(atBuffer, ">", 5000)) {
    Serial.println(F("[SERVER] CIPSEND ERROR"));
    closeTcpAfterFailure();
    return false;
  }
#if VERBOSE_NETWORK_LOGS
  Serial.println(F("[HTTP TX] GET /api/robot/command"));
#endif
  esp8266.print(requestBuffer);
  const bool httpOk = collectHttpResponse();
#if VERBOSE_NETWORK_LOGS
  Serial.println(F("[HTTP RX RAW]"));
  Serial.println(espBuffer[0] ? espBuffer : "(no response)");
#endif
  if (!httpOk) {
    Serial.println(F("[SERVER] HTTP RESPONSE ERROR"));
    closeTcpAfterFailure();
    return false;
  }
#if VERBOSE_NETWORK_LOGS
  Serial.println(F("[SERVER] HTTP 200 OK"));
  Serial.println(F("[SERVER] ===== command request complete ====="));
#endif
  return true;
}

// RFID 도착, 릴레이/모듈 가동, 복귀 같은 자동차 상태를 PC 서버로 보낸다.
// 별도 JSON 라이브러리 없이 짧은 GET 요청을 사용해 Uno 메모리 사용량을 줄인다.
bool reportRobotStatus(bool heartbeatOnly) {
  char phaseName[20];
  copyPhaseName(phaseName, sizeof(phaseName));
  const char* eventName = heartbeatOnly ? HEARTBEAT_EVENT : pendingEvent;

#if VERBOSE_NETWORK_LOGS
  Serial.println(F("[STATUS] ===== robot status report start ====="));
  Serial.print(F("[STATUS] phase="));
  Serial.print(phaseName);
  Serial.print(F(", event="));
  Serial.print(eventName);
  Serial.print(F(", zone="));
  Serial.print(targetZone);
  Serial.print(F(", action="));
  Serial.print(targetAction);
  Serial.print(F(", ack_revision="));
  Serial.print(acknowledgedRevision);
  Serial.print(F(", result="));
  Serial.println(commandResult);
#endif

  snprintf_P(atBuffer, sizeof(atBuffer), PSTR("AT+CIPSTART=\"TCP\",\"%s\",%u"),
             SERVER_HOST, SERVER_PORT);
  if (!sendAt(atBuffer, "CONNECT", 10000, "Linked")) {
    Serial.println(F("[STATUS] TCP CONNECT ERROR"));
    closeTcpAfterFailure();
    return false;
  }

  const int requestLength = snprintf_P(
      requestBuffer, sizeof(requestBuffer),
      PSTR("GET /api/robot/status?phase=%s&event=%s&zone=%s&action=%s&ack_revision=%ld&result=%s HTTP/1.0\r\n\r\n"),
      phaseName, eventName, targetZone, targetAction,
      acknowledgedRevision, commandResult);
  if (requestLength < 0 || requestLength >= static_cast<int>(sizeof(requestBuffer))) {
    Serial.println(F("[STATUS] request buffer overflow"));
    closeTcpAfterFailure();
    return false;
  }
  snprintf_P(atBuffer, sizeof(atBuffer), PSTR("AT+CIPSEND=%u"),
             static_cast<unsigned int>(requestLength));
  if (!sendAt(atBuffer, ">", 5000)) {
    Serial.println(F("[STATUS] CIPSEND ERROR"));
    closeTcpAfterFailure();
    return false;
  }

#if VERBOSE_NETWORK_LOGS
  Serial.println(F("[HTTP TX] GET /api/robot/status"));
#endif
  esp8266.print(requestBuffer);
  const bool httpOk = collectHttpResponse();
#if VERBOSE_NETWORK_LOGS
  Serial.println(F("[HTTP RX RAW]"));
  Serial.println(espBuffer[0] ? espBuffer : "(no response)");
#endif
  if (!httpOk) {
    Serial.println(F("[STATUS] HTTP RESPONSE ERROR"));
    closeTcpAfterFailure();
    return false;
  }

  // 서버는 ACK를 거부해도 HTTP 200과 ack_revision:null을 반환할 수 있다.
  // 실제로 인정된 revision이 현재 명령과 같을 때만 이벤트를 큐에서 지운다.
  long serverAckRevision = -1;
  if (!extractJsonLong("ack_revision", serverAckRevision) ||
      serverAckRevision != acknowledgedRevision) {
    Serial.println(F("[STATUS] ACK REJECTED -> keep pending event"));
    return false;
  }

  if (!heartbeatOnly) finishRobotReport();
  lastHeartbeatAt = millis();
  Serial.println(F("[STATUS] report complete"));
  return true;
}

bool extractJsonText(const char* key, char* output, byte outputSize) {
  char token[30];
  snprintf_P(token, sizeof(token), PSTR("\"%s\":\""), key);
  char* start = strstr(espBuffer, token);
  if (!start) return false;
  start += strlen(token);
  char* end = strchr(start, '"');
  if (!end) return false;
  byte length = min(static_cast<int>(end - start), static_cast<int>(outputSize - 1));
  strncpy(output, start, length);
  output[length] = '\0';
  return true;
}

bool extractJsonLong(const char* key, long& output) {
  char token[24];
  snprintf_P(token, sizeof(token), PSTR("\"%s\":"), key);
  char* start = strstr(espBuffer, token);
  if (!start) return false;
  start += strlen(token);
  while (*start == ' ') ++start;
  if (*start < '0' || *start > '9') return false;
  output = atol(start);
  return true;
}

bool pollServerCommand() {
  Serial.println(F("[COMMAND] polling server"));
  if (!fetchCommandResponse()) return false;

  char command[16];
  char nextTarget[8];
  char nextAction[16];
  long nextRevision = -1;
  if (!extractJsonLong("revision", nextRevision)) {
    Serial.println(F("[COMMAND] JSON REVISION ERROR"));
    return false;
  }
  if (!extractJsonText("command", command, sizeof(command))) {
    Serial.println(F("[COMMAND] JSON COMMAND ERROR"));
    return false;
  }
  if (!extractJsonText("target_zone", nextTarget, sizeof(nextTarget))) {
    Serial.println(F("[COMMAND] JSON TARGET ERROR"));
    return false;
  }
  if (!extractJsonText("action", nextAction, sizeof(nextAction))) {
    Serial.println(F("[COMMAND] JSON ACTION ERROR"));
    return false;
  }

  Serial.print(F("[COMMAND] command="));
  Serial.print(command);
  Serial.print(F(", target_zone="));
  Serial.print(nextTarget);
  Serial.print(F(", action="));
  Serial.println(nextAction);

  // 정지 ACK 재확인 중에는 어떤 새 이동/모듈 명령도 소비하지 않는다.
  // 로컬 STOP이 모두 확인된 뒤 다음 서버 poll에서 같은 revision을 처리한다.
  if (stopRetryMode != STOP_RETRY_NONE) {
    Serial.println(F("[COMMAND] deferred until STOP confirmation"));
    return true;
  }

  if (nextRevision == lastCommandRevision) {
    Serial.print(F("[COMMAND] revision "));
    Serial.print(nextRevision);
    Serial.println(F(" already handled"));
    // 서버가 잠시 끊겨 안전 정지했더라도 경로 위치가 확실하면, 실패 이벤트를
    // 먼저 보고한 뒤 동일 revision의 자동 명령을 딱 한 번 재개한다.
    if (retrySameRevisionAllowed && robotPhase == PHASE_TASK_COMPLETE &&
        !robotReportPending && !deferredEvent[0] &&
        (!strcmp_P(command, PSTR("TASK")) ||
         !strcmp_P(command, PSTR("RETURN_HOME")))) {
      retrySameRevisionAllowed = false;
      setCommandResult(F("EXECUTING"));
      if (!strcmp_P(command, PSTR("TASK"))) {
        taskActive = true;
        startPlaceholderMovement();
      } else {
        taskActive = false;
        startPlaceholderReturn();
      }
      return true;
    }

    // 같은 revision은 완전한 no-op이다. 모듈 완료 뒤에는 서버가 완료 이후의
    // 새 구역 측정을 확인하고 새 revision을 발급해야만 다음 1회가 시작된다.
    // 따라서 5/15초 센서 주기 사이에 가습·제습이 연속 재가동되지 않는다.
    return true;
  }

  Serial.print(F("[COMMAND] new revision="));
  Serial.println(nextRevision);
  acknowledgedRevision = nextRevision;
  setCommandResult(F("EXECUTING"));
  manualForwardActive = false;
  retrySameRevisionAllowed = false;

  if (!strcmp_P(command, PSTR("CALIBRATE_HOME"))) {
    performHomeCalibration();
  } else if (!strcmp_P(command, PSTR("MOTOR_FWD"))) {
    if (requireHomeCalibration()) {
      manualForwardActive = true;
      taskActive = false;
      strcpy_P(targetZone, PSTR("HOME"));
      strcpy_P(targetAction, PSTR("NONE"));
      const bool moduleStopped = stopModuleController();
      // 수동 직진은 RFID 경로 상태와 무관하므로 이후 자동 임무에 오래된 위치를
      // 재사용하지 않도록 출발 전에 경로를 UNKNOWN으로 잠근다.
      latchRouteUnknown();
      const bool motorStarted = moduleStopped &&
          startMotorController(HEADING_OUTBOUND);
      if (motorStarted) {
        routeHeading = HEADING_OUTBOUND;
        robotPhase = PHASE_MOVING;
        phaseStartedAt = millis();
        queueRobotReport(F("MANUAL_MOTOR_FWD"));
      } else {
        manualForwardActive = false;
        stopMotorController();
        robotPhase = PHASE_TASK_COMPLETE;
        setCommandResult(F("FAILED"));
        queueRobotReport(F("MANUAL_MOTOR_ERROR"));
      }
    }
  } else if (!strcmp_P(command, PSTR("MOTOR_RETURN"))) {
    taskActive = false;
    strcpy_P(targetZone, PSTR("HOME"));
    strcpy_P(targetAction, PSTR("NONE"));
    startPlaceholderReturn();
    queueRobotReport(F("MANUAL_RETURN"));
  } else if (!strcmp_P(command, PSTR("ALL_STOP"))) {
    taskActive = false;
    const bool motorStopped = stopMotorController();
    const bool moduleStopped = stopModuleController();
    if (!motorStopped) latchRouteUnknown();
    if (motorStopped && moduleStopped) {
      robotPhase = PHASE_IDLE;
      setCommandResult(F("COMPLETED"));
    } else {
      armStopRetry(STOP_RETRY_TO_IDLE, motorStopped, moduleStopped);
    }
    queueRobotReport(F("MANUAL_ALL_STOP"));
  } else if (!strcmp_P(command, PSTR("MOTOR_STOP"))) {
    taskActive = false;
    const bool motorStopped = stopMotorController();
    const bool moduleStopped = stopModuleController();
    if (!motorStopped) latchRouteUnknown();
    if (motorStopped && moduleStopped) {
      robotPhase = PHASE_IDLE;
      setCommandResult(F("COMPLETED"));
    } else {
      armStopRetry(STOP_RETRY_TO_IDLE, motorStopped, moduleStopped);
    }
    queueRobotReport(F("MANUAL_MOTOR_STOP"));
  } else if (!strcmp_P(command, PSTR("ACT_HUMIDIFY")) ||
             !strcmp_P(command, PSTR("ACT_DEHUMID"))) {
    taskActive = false;
    strcpy_P(targetZone, PSTR("HOME"));
    if (!strcmp_P(command, PSTR("ACT_HUMIDIFY"))) {
      strcpy_P(targetAction, PSTR("HUMIDIFY"));
    } else {
      strcpy_P(targetAction, PSTR("DEHUMIDIFY"));
    }
    startPlaceholderModule(false);
    queueRobotReport(F("MANUAL_ACTUATOR"));
  } else if (!strcmp_P(command, PSTR("ACT_STOP"))) {
    taskActive = false;
    const bool motorStopped = stopMotorController();
    const bool moduleStopped = stopModuleController();
    if (!motorStopped) latchRouteUnknown();
    if (motorStopped && moduleStopped) {
      robotPhase = PHASE_IDLE;
      setCommandResult(F("COMPLETED"));
    } else {
      armStopRetry(STOP_RETRY_TO_IDLE, motorStopped, moduleStopped);
    }
    queueRobotReport(F("MANUAL_ACT_STOP"));
  } else if (!strcmp_P(command, PSTR("RFID_TEST"))) {
    if (!manualForwardActive && validRouteStation(expectedStation) &&
        (robotPhase == PHASE_MOVING || robotPhase == PHASE_RETURNING)) {
      processRouteRfid(expectedStation);
      queueRobotReport(F("WEB_RFID_TEST"));
    } else {
      setCommandResult(F("IGNORED"));
      queueRobotReport(F("RFID_TEST_IGNORED"));
    }
  } else if (!strcmp_P(command, PSTR("I2C_CHECK"))) {
    const bool motorOk = i2cDevicePresent(MOTOR_UNO_ADDRESS);
    const bool actuatorOk = i2cDevicePresent(ACTUATOR_UNO_ADDRESS);
    probeI2cDevice(MOTOR_UNO_ADDRESS, F("MotorUno"));
    probeI2cDevice(ACTUATOR_UNO_ADDRESS, F("ActuatorUno"));
    // LCD는 ActuatorUno의 별도 D5/D4 소프트웨어 I2C에 있으므로 SensorUno의
    // A4/A5 주소 probe 대상이 아니다. 준비 상태는 0x09 응답 flags로 받는다.
    setCommandResult(motorOk && actuatorOk ? F("COMPLETED") : F("FAILED"));
    queueRobotReport(motorOk && actuatorOk ? F("I2C_ALL_OK")
                                          : F("I2C_DEVICE_ERROR"));
  } else if (!strcmp_P(command, PSTR("TASK"))) {
    // 새 revision은 payload가 같더라도 사용자가 명시적으로 다시 실행한
    // 명령이므로 언제나 새 임무로 시작한다.
    taskActive = true;
    strncpy(targetZone, nextTarget, sizeof(targetZone) - 1);
    targetZone[sizeof(targetZone) - 1] = '\0';
    strncpy(targetAction, nextAction, sizeof(targetAction) - 1);
    targetAction[sizeof(targetAction) - 1] = '\0';
    Serial.print(F("[CAR] NEW TASK target="));
    Serial.print(targetZone);
    Serial.print(F(", action="));
    Serial.println(targetAction);
    startPlaceholderMovement();
  } else if (!strcmp_P(command, PSTR("RETURN_HOME"))) {
    const bool alreadyReturning = targetStation == STATION_HOME &&
        robotPhase == PHASE_RETURNING;
    strcpy_P(targetZone, PSTR("HOME"));
    strcpy_P(targetAction, PSTR("NONE"));
    taskActive = false;
    if (routeAtStation && confirmedStation == STATION_HOME) {
      const bool motorStopped = stopMotorController();
      const bool moduleStopped = stopModuleController();
      targetStation = STATION_HOME;
      if (!motorStopped) latchRouteUnknown();
      if (motorStopped && moduleStopped) {
        robotPhase = PHASE_IDLE;
        setCommandResult(F("COMPLETED"));
      } else {
        armStopRetry(STOP_RETRY_TO_IDLE, motorStopped, moduleStopped);
      }
      queueRobotReport(F("HOME_ALREADY"));
    } else if (alreadyReturning) {
      setCommandResult(F("EXECUTING"));
      queueRobotReport(F("RETURN_CONTINUING"));
    } else {
      startPlaceholderReturn();
    }
  } else {
    taskActive = false;
    stopMotorController();
    stopModuleController();
    robotPhase = PHASE_TASK_COMPLETE;
    setCommandResult(F("FAILED"));
    queueRobotReport(F("INVALID_COMMAND"));
  }

  lastCommandRevision = nextRevision;
  return true;
}

void makeUidText() {
  byte position = 0;
  for (byte i = 0; i < rfid.uid.size; ++i) {
    if (rfid.uid.uidByte[i] < 0x10) uidText[position++] = '0';
    char hex[3];
    itoa(rfid.uid.uidByte[i], hex, 16);
    uidText[position++] = toupper(hex[0]);
    if (hex[1]) uidText[position++] = toupper(hex[1]);
    if (i + 1 < rfid.uid.size) uidText[position++] = ' ';
  }
  uidText[position] = '\0';
}

void processRouteRfid(RouteStation scannedStation) {
  const bool routeMoving =
      robotPhase == PHASE_MOVING || robotPhase == PHASE_RETURNING;
  if (!routeMoving || manualForwardActive ||
      scannedStation == STATION_UNKNOWN) return;

  // 웹/USB 합성 RFID도 실제 리더와 같은 방향변경 가드를 통과해야 한다.
  // 차체 회전은 없지만, 직전 태그가 안테나 아래에 남은 채 후진을 시작하면
  // 같은 UID를 새 도착으로 오인할 수 있다.
  if (rfidDirectionGuardActive &&
      (obstaclePauseActive || !rfidDirectionClearSeen ||
       millis() - rfidDirectionGuardStartedAt <
           RFID_DIRECTION_SETTLE_MS)) return;
  rfidDirectionGuardActive = false;

  const unsigned long now = millis();
  // 방금 태그 위에서 미끄러지거나 오래 정체되어도 같은 역을 다시 오류로
  // 처리하지 않는다. 반대 방향 재라우팅으로 그 역이 expected가 되면 허용한다.
  if (scannedStation == lastAcceptedRfidStation &&
      scannedStation != expectedStation) return;
  lastAcceptedRfidStation = scannedStation;

  // 구역 태그를 읽은 순간 MotorUno에 STOP+sequence를 보내고 ACK까지 확인한
  // 뒤 위치를 확정한다. 정지선은 HOME에서만 사용한다.
  if (!stopMotorController()) {
    stopModuleController();
    latchRouteUnknown();
    taskActive = false;
    robotPhase = PHASE_TASK_COMPLETE;
    setCommandResult(F("I2C_ERROR"));
    queueRobotReport(F("MOTOR_STOP_ERROR"));
    return;
  }

  confirmedStation = scannedStation;
  routeAtStation = true;
  phaseStartedAt = now;
  if (scannedStation != expectedStation) {
    stopModuleController();
    taskActive = false;
    robotPhase = PHASE_TASK_COMPLETE;
    setCommandResult(F("FAILED"));
    queueRobotReport(F("ROUTE_UID_ERROR"));
    return;
  }

  if (confirmedStation == targetStation && taskActive) {
    startPlaceholderModule();
    return;
  }

  // ZONE99 출동 중의 ZONE2, 또는 HOME 복귀 중의 ZONE2는 중간역이다.
  // 잠깐 정지해 위치를 확정한 뒤 같은 방향의 라인트레이싱을 다시 시작한다.
  queueRobotReport(F("RFID_PASS"));
  if (targetStation == STATION_HOME) startPlaceholderReturn();
  else if (taskActive) startPlaceholderMovement();
}

void checkRfidArrival() {
  const bool routeMoving =
      robotPhase == PHASE_MOVING || robotPhase == PHASE_RETURNING;
  // IDLE/모듈 실행 중에는 최대 36ms 걸리는 RC522 transceive를 실행하지 않아
  // ESP-01 SoftwareSerial 수신과 actuator 상태 확인을 방해하지 않는다.
  if (!routeMoving || manualForwardActive) return;
  if (rfidDirectionGuardActive && obstaclePauseActive) {
    // 장애물 때문에 실제 이동하지 못한 시간은 안정 시간으로 세지 않는다.
    rfidDirectionGuardStartedAt = millis();
    rfidDirectionClearSeen = false;
  }
  if (millis() - lastRfidScanAt < RFID_SCAN_INTERVAL_MS) return;
  lastRfidScanAt = millis();

  if (!rfid.PICC_IsNewCardPresent()) {
    if (rfidDirectionGuardActive && !obstaclePauseActive) {
      rfidDirectionClearSeen = true;
    }
    if (millis() - lastRfidLogAt >= RFID_LOG_INTERVAL_MS) {
      lastRfidLogAt = millis();
      Serial.println(F("[RFID] scanning... no card"));
    }
    return;
  }

  if (rfidDirectionGuardActive &&
      (obstaclePauseActive || !rfidDirectionClearSeen ||
       millis() - rfidDirectionGuardStartedAt <
           RFID_DIRECTION_SETTLE_MS)) {
    // 태그가 계속 보이면 no-card 조건을 다시 요구한다.
    rfidDirectionClearSeen = false;
    return;
  }

  Serial.println(F("[RFID] card detected"));
  if (!rfid.PICC_ReadCardSerial()) {
    Serial.println(F("[RFID] card UID read failed"));
    return;
  }
  makeUidText();

  RouteStation scannedStation = STATION_UNKNOWN;
  if (!strcmp_P(uidText, RFID_ZONE2_UID)) {
    scannedStation = STATION_ZONE2;
  } else if (!strcmp_P(uidText, RFID_ZONE99_UID)) {
    scannedStation = STATION_ZONE99;
  }

  if (scannedStation == STATION_UNKNOWN && RFID_PLACEHOLDER_ACCEPT_ANY_CARD &&
      routeMoving) {
    scannedStation = expectedStation;
    Serial.println(F("[RFID] PLACEHOLDER MODE -> expected station accepted"));
  }

  // 먼저 모터를 정지시킨 뒤 로그를 출력해 고속 주행에서도 제동을 늦추지 않는다.
  processRouteRfid(scannedStation);
  if (scannedStation == STATION_ZONE2) {
    Serial.println(F("[RFID] ARRIVAL CONFIRMED station=ZONE2"));
  } else if (scannedStation == STATION_ZONE99) {
    Serial.println(F("[RFID] ARRIVAL CONFIRMED station=ZONE99"));
  } else {
    Serial.println(F("[RFID] UNREGISTERED"));
  }

  rfid.PICC_HaltA();
  rfid.PCD_StopCrypto1();
  Serial.println(F("[RFID] card processing complete"));
}

void setup() {
  Serial.begin(USB_SERIAL_BAUD);
  esp8266.begin(9600);

  Serial.println();
  Serial.println(F("[BOOT] UNO CAR USB=115200 ESP=9600"));
  Serial.println(F("[BOOT] ESP=5/6 RFID=8..12 I2C=A4/A5"));
  Serial.println(F("[BOOT] DHT=Actuator D2 / rear HC=Motor D2,A1"));
  Serial.println(F("[BOOT] route HOME>ZONE2>ZONE99"));

  // 전원/배선 진단: 정상 대기 중인 ESP TX는 보통 HIGH다. SensorUno의
  // D2/D3/D4는 분배 후 비워 두며 ESP TX가 계속 LOW이면 전원/EN을 확인한다.
  Serial.print(F("[PIN LEVEL] ESP_TX@D6="));
  Serial.println(digitalRead(UNO_ESP_RX_PIN));

  // 저속 소프트 ACK 프로브는 버스 진단 스케치에서만 실행한다.
  // 정상 운전에서는 하드웨어 Wire 타임아웃과 슬레이브 상태 ACK를 사용한다.

  // 이 보드는 I2C 마스터이며 A4/A5로 두 슬레이브 UNO에 명령을 보낸다.
  Wire.begin();
  // AVR Wire.setClock(10000)은 계산 결과가 8비트 TWBR을 넘어 약 250kHz로
  // 되감기는 문제가 있다. prescaler=4, TWBR=198을 직접 설정하면
  // 16MHz Uno에서 실제 약 10kHz가 된다: 16M/(16+2*198*4)=10kHz.
  configureI2c10kHz();
  Wire.setWireTimeout(25000UL, true);
  delay(50);
  probeI2cDevice(MOTOR_UNO_ADDRESS, F("MotorUno"));
  probeI2cDevice(ACTUATOR_UNO_ADDRESS, F("ActuatorUno"));
  const bool bootMotorStopped = stopMotorController();
  const bool bootModuleStopped = stopModuleController();
  if (!bootMotorStopped || !bootModuleStopped) {
    queueRobotReport(F("BOOT_STOP_ERROR"));
  }
  Serial.println(F("[RFID] software SPI initialization"));
  rfid.PCD_Init();
  const byte rfidVersion = rfid.PCD_ReadRegister(SoftwareMFRC522::VersionReg);
  rfidReady = rfidVersion != 0x00 && rfidVersion != 0xFF;
  Serial.print(F("[RFID] RC522 version=0x"));
  if (rfidVersion < 0x10) Serial.print('0');
  Serial.println(rfidVersion, HEX);
  if (!rfidReady) {
    Serial.println(F("[RFID] ERROR: RC522 communication failed"));
  } else {
    Serial.println(F("[RFID] RC522 communication OK, scanning started"));
  }

  // ActuatorUno가 긴 Wi-Fi 초기화 중에도 로컬 DHT22와 기본 상태 화면을
  // 표시할 수 있도록 첫 상태 telemetry를 먼저 보낸다.
  lastDisplaySentAt = millis() - DISPLAY_RETRY_MS;
  lastDisplayHeartbeatAt = millis() - DISPLAY_HEARTBEAT_MS;
  serviceDisplayTelemetry();

  // 부팅 STOP이 확인되지 않았다면 긴 Wi-Fi 초기화보다 로컬 STOP 재시도를
  // 먼저 수행한다. 확인 뒤 loop의 reconnect 경로로 서버에 연결한다.
  wifiReady = stopRetryMode == STOP_RETRY_NONE && connectWifi();
  // 최초 접속이 실패해도 setup 직후 다시 긴 접속 절차를 반복하지 않는다.
  // 15초 동안 센서/RFID/표시 telemetry와 안전 제어 loop를 먼저 실행한다.
  lastWifiReconnectAttemptAt = millis();
  Serial.println(wifiReady ? F("[BOOT] WIFI OK") : F("[BOOT] WIFI ERROR"));
  Serial.println(F("[BOOT] setup complete"));
  Serial.println(F("================================"));
  lastPollAt = millis() - POLL_INTERVAL_MS;
}

void loop() {
  applyMotorLinkState();
  serviceStopRetry();
  updatePlaceholderStateMachine();
  checkUsbTestCommands();
  checkRfidArrival();
  serviceDisplayTelemetry();

  // USB one-shot 시험 중에는 Wi-Fi reconnect/poll이 실제 주행과 RFID 처리를
  // 끊지 않게 한다. 모듈 완료나 오류로 TASK_COMPLETE가 되면 시험을 종료하고
  // 다음 loop부터 정상 네트워크 동작으로 돌아간다.
  if (usbZone2MissionActive) {
    if (robotPhase != PHASE_TASK_COMPLETE) return;
    usbZone2MissionActive = false;
    taskActive = false;
    lastPollAt = millis();
    Serial.println(F("[USB MISSION] finished; network polling restored"));
    return;
  }

  if (millis() - lastPollAt >= POLL_INTERVAL_MS) {
    // 정지 확인이 최우선이다. 긴 ESP/HTTP 대기로 500ms 로컬 재시도를
    // 지연시키거나, 완료 전의 상태를 서버 ACK로 보내지 않는다.
    if (stopRetryMode != STOP_RETRY_NONE) return;
    lastPollAt = millis();
    Serial.println(F("[LOOP] 3-second server poll"));
    if (!wifiReady) {
      if (millis() - lastWifiReconnectAttemptAt >= WIFI_RECONNECT_INTERVAL_MS) {
        lastWifiReconnectAttemptAt = millis();
        Serial.println(F("[LOOP] Wi-Fi reconnect backoff elapsed -> reconnect"));
        wifiReady = connectWifi();
        Serial.println(wifiReady ? F("[LOOP] Wi-Fi reconnect OK")
                                 : F("[LOOP] Wi-Fi reconnect ERROR"));
      } else {
        Serial.println(F("[LOOP] Wi-Fi reconnect backoff; local control remains active"));
      }
    }
    if (wifiReady && !pollServerCommand()) {
      if (consecutiveServerFailures < 255) ++consecutiveServerFailures;
      stopSafelyForServerLoss();
      Serial.print(F("[LOOP] SERVER ERROR count="));
      Serial.println(consecutiveServerFailures);

      // STOP ACK가 빠졌다면 AP 상태 확인보다 출력 정지 재시도가 먼저다.
      if (stopRetryMode != STOP_RETRY_NONE) return;

      // TCP 서버가 꺼진 것과 AP 연결이 끊긴 것은 다르다. 매번 ESP를
      // 리셋하지 않고 3회 실패했을 때만 AP 연결 상태를 확인한다.
      if (consecutiveServerFailures >= SERVER_FAILURES_BEFORE_WIFI_CHECK) {
        const bool stillAssociated = sendAt("AT+CWJAP?", "+CWJAP:", 3000);
        wifiReady = stillAssociated;
        consecutiveServerFailures = 0;
        if (!stillAssociated) lastWifiReconnectAttemptAt = millis();
        Serial.println(stillAssociated
            ? F("[LOOP] Wi-Fi is still associated; server will be retried")
            : F("[LOOP] Wi-Fi association lost; reconnect required"));
      }
    } else if (wifiReady) {
      consecutiveServerFailures = 0;
      Serial.println(F("[LOOP] command processing OK"));
      if (stopRetryMode != STOP_RETRY_NONE) {
        Serial.println(F("[LOOP] STOP confirmation has priority over reporting"));
      } else if (robotReportPending) {
        // 실제 이벤트가 heartbeat보다 항상 먼저 전송된다.
        if (!reportRobotStatus()) {
          Serial.println(F("[LOOP] STATUS REPORT ERROR -> retry on next poll"));
        }
      } else if (millis() - lastHeartbeatAt >= HEARTBEAT_INTERVAL_MS &&
                 !reportRobotStatus(true)) {
        // 성공할 때만 lastHeartbeatAt이 갱신되므로 다음 폴링에서 다시 시도한다.
        Serial.println(F("[LOOP] HEARTBEAT ERROR -> retry on next poll"));
      }
    }
  }

  // 릴레이는 상태 전환 함수에서만 켜고 끈다. loop에서 계속 덮어쓰지 않는다.
}
