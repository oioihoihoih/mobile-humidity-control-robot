#include <Arduino.h>
#include <Wire.h>

// SimulIDE용 MotorUno 프록시 펌웨어.
// I2C 주소와 command+sequence/status+command+sequence 프레임은 실제
// MotorUno와 같고, AFMotor 실드의 M1~M4 정·역방향 출력은 여덟 LED로
// 표시한다. M1/M3는 왼쪽, M2/M4는 오른쪽이며 M3/M4는 N20 1:298이다.

constexpr byte I2C_ADDRESS = 0x08;

constexpr byte COMMAND_STOP = 0;
constexpr byte COMMAND_OUTBOUND = 0x11;
constexpr byte COMMAND_RETURN = 0x12;
constexpr byte COMMAND_PAUSE = 3;
constexpr byte COMMAND_RESUME = 4;
constexpr byte COMMAND_KEEPALIVE = 5;
constexpr byte COMMAND_HOME_SYNC = 6;
constexpr byte COMMAND_PROTOCOL_SYNC = 7;

constexpr byte STATUS_IDLE = 0;
constexpr byte STATUS_RUNNING = 1;
constexpr byte STATUS_OBSTACLE = 2;
constexpr byte STATUS_WATCHDOG_TIMEOUT = 4;
constexpr byte STATUS_INVALID_COMMAND = 5;
constexpr byte STATUS_CALIBRATION_REQUIRED = 7;
constexpr byte STATUS_PROTOCOL_REQUIRED = 8;

// SimulIDE R260501은 다중 AVR slave가 한 request에서 여러 바이트를
// 안정적으로 반환하지 못한다. SensorUno가 0xE0..0xE2로 논리 응답
// [status, appliedCommand, appliedSequence]의 위치를 고른 뒤 1바이트씩 읽는다.
// 운영 펌웨어의 논리 3바이트 응답 의미는 그대로 유지한다.
constexpr byte STATUS_BYTE_SELECT_BASE = 0xE0;
constexpr byte STATUS_REPLY_SIZE = 3;

// 한 모터 채널마다 FORWARD/REVERSE LED를 따로 둔다. 실제 AFMotor 실드의
// 제어 핀을 뜻하지 않으며, SimulIDE에서 네 채널의 논리 방향만 보여준다.
constexpr byte M1_FORWARD_LED = 2;
constexpr byte M1_REVERSE_LED = 3;
constexpr byte M2_FORWARD_LED = 4;
constexpr byte M2_REVERSE_LED = 5;
constexpr byte M3_FORWARD_LED = 6;
constexpr byte M3_REVERSE_LED = 7;
constexpr byte M4_FORWARD_LED = 8;
constexpr byte M4_REVERSE_LED = 11;

// 실제 핀과 같은 D9/D10 라인센서. 버튼은 편차 검출 프록시다.
constexpr byte LEFT_IR_PIN = 9;
constexpr byte RIGHT_IR_PIN = 10;

// 명시적 STOP 경계를 포함해 마지막 실제 구동과 반대 방향이면 RELEASE 시간을
// 보장한다. RETURN은 회전이 아니라 네 바퀴의 직선 후진이다.
constexpr unsigned long DIRECTION_CHANGE_DEAD_TIME_MS = 120;
constexpr unsigned long WATCHDOG_MS = 2000;

volatile byte pendingCommand = COMMAND_STOP;
volatile byte pendingSequence = 0;
volatile bool commandPending = false;
volatile bool keepalivePending = false;
volatile byte motorStatus = STATUS_PROTOCOL_REQUIRED;
volatile byte appliedCommand = COMMAND_STOP;
volatile byte appliedSequence = 0;
volatile byte selectedStatusByte = 0;

byte activeCommand = COMMAND_STOP;
byte resumeCommand = COMMAND_STOP;
bool headingHomebound = false;
bool calibrated = false;
bool protocolValidated = false;
bool directionChangeDeadTimeActive = false;
bool outputsActive = false;
bool motorDirectionKnown = false;
bool lastDrivenHomebound = false;
unsigned long directionChangeDeadTimeStartedAt = 0;
unsigned long lastOutputsOffAt = 0;
unsigned long lastControlAt = 0;

void outputsOff() {
  digitalWrite(M1_FORWARD_LED, LOW);
  digitalWrite(M1_REVERSE_LED, LOW);
  digitalWrite(M2_FORWARD_LED, LOW);
  digitalWrite(M2_REVERSE_LED, LOW);
  digitalWrite(M3_FORWARD_LED, LOW);
  digitalWrite(M3_REVERSE_LED, LOW);
  digitalWrite(M4_FORWARD_LED, LOW);
  digitalWrite(M4_REVERSE_LED, LOW);
  if (outputsActive) {
    outputsActive = false;
    lastOutputsOffAt = millis();
  }
}

void markDrive(bool homebound) {
  outputsActive = true;
  motorDirectionKnown = true;
  lastDrivenHomebound = homebound;
}

void driveOutbound() {
  outputsOff();
  digitalWrite(M1_FORWARD_LED, HIGH);
  digitalWrite(M2_FORWARD_LED, HIGH);
  digitalWrite(M3_FORWARD_LED, HIGH);
  digitalWrite(M4_FORWARD_LED, HIGH);
  markDrive(false);
}

void driveHomebound() {
  outputsOff();
  digitalWrite(M1_REVERSE_LED, HIGH);
  digitalWrite(M2_REVERSE_LED, HIGH);
  digitalWrite(M3_REVERSE_LED, HIGH);
  digitalWrite(M4_REVERSE_LED, HIGH);
  markDrive(true);
}

void driveForCurrentDirection() {
  if (headingHomebound) driveHomebound();
  else driveOutbound();
}

// 차체를 제자리 회전시키지 않는다. 편차가 있으면 현재 이동 방향을 유지한
// 채 한쪽 두 채널만 잠시 쉬게 한다. 후진에서는 보정 좌우를 반대로 본다.
void correctTowardLeft() {
  outputsOff();
  if (headingHomebound) {
    digitalWrite(M1_REVERSE_LED, HIGH);
    digitalWrite(M3_REVERSE_LED, HIGH);
  } else {
    digitalWrite(M2_FORWARD_LED, HIGH);
    digitalWrite(M4_FORWARD_LED, HIGH);
  }
  markDrive(headingHomebound);
}

void correctTowardRight() {
  outputsOff();
  if (headingHomebound) {
    digitalWrite(M2_REVERSE_LED, HIGH);
    digitalWrite(M4_REVERSE_LED, HIGH);
  } else {
    digitalWrite(M1_FORWARD_LED, HIGH);
    digitalWrite(M3_FORWARD_LED, HIGH);
  }
  markDrive(headingHomebound);
}

void receiveCommand(int byteCount) {
  if (byteCount == 1 && Wire.available()) {
    const byte value = static_cast<byte>(Wire.read());
    while (Wire.available()) Wire.read();
    if (value >= STATUS_BYTE_SELECT_BASE &&
        value < STATUS_BYTE_SELECT_BASE + STATUS_REPLY_SIZE) {
      selectedStatusByte = value - STATUS_BYTE_SELECT_BASE;
      return;
    }
    pendingCommand = 0xFF;
    pendingSequence = appliedSequence;
    commandPending = true;
    return;
  }

  if (byteCount != 2 || Wire.available() < 2) {
    while (Wire.available()) Wire.read();
    pendingCommand = 0xFF;
    pendingSequence = appliedSequence;
    commandPending = true;
    return;
  }

  const byte command = Wire.read();
  const byte sequence = Wire.read();
  while (Wire.available()) Wire.read();

  if (command == COMMAND_KEEPALIVE) {
    keepalivePending = true;
    return;
  }
  pendingCommand = command;
  pendingSequence = sequence;
  commandPending = true;
}

void sendStatus() {
  const byte reply[STATUS_REPLY_SIZE] = {
      motorStatus, appliedCommand, appliedSequence};
  const byte index = selectedStatusByte < STATUS_REPLY_SIZE
                         ? selectedStatusByte
                         : 0;
  Wire.write(reply[index]);
}

void enterSafeStop(byte status) {
  activeCommand = COMMAND_STOP;
  resumeCommand = COMMAND_STOP;
  directionChangeDeadTimeActive = false;
  outputsOff();
  motorStatus = status;
}

void applyHomeSync() {
  // HOME_SYNC는 탐색 명령이 아니다. 정지 상태에서 두 IR 입력이 모두
  // HIGH(넓은 HOME 검은 마커 프록시)일 때만 interlock을 해제한다.
  calibrated = false;
  enterSafeStop(STATUS_CALIBRATION_REQUIRED);
  lastControlAt = millis();
  if (digitalRead(LEFT_IR_PIN) != HIGH ||
      digitalRead(RIGHT_IR_PIN) != HIGH) {
    Serial.println(F("[CALIBRATION] HOME_SYNC rejected: marker absent"));
    return;
  }
  headingHomebound = false;
  calibrated = true;
  motorStatus = STATUS_IDLE;
  Serial.println(F("[CALIBRATION] HOME_SYNC -> calibrated / OUTBOUND"));
}

bool isValidControlCommand(byte command) {
  return command == COMMAND_STOP || command == COMMAND_OUTBOUND ||
         command == COMMAND_RETURN || command == COMMAND_PAUSE ||
         command == COMMAND_RESUME || command == COMMAND_KEEPALIVE ||
         command == COMMAND_HOME_SYNC || command == COMMAND_PROTOCOL_SYNC;
}

void applyCommand(byte command, byte sequence) {
  appliedCommand = command;
  appliedSequence = sequence;
  lastControlAt = millis();

  if (!isValidControlCommand(command)) {
    enterSafeStop(STATUS_INVALID_COMMAND);
    return;
  }

  if (command == COMMAND_PROTOCOL_SYNC) {
    calibrated = false;
    protocolValidated = true;
    enterSafeStop(STATUS_PROTOCOL_REQUIRED);
    Serial.println(F("[PROTOCOL] 4WD v2 sync confirmed"));
    return;
  }

  if (!protocolValidated) {
    enterSafeStop(STATUS_PROTOCOL_REQUIRED);
    return;
  }

  if (command == COMMAND_HOME_SYNC) {
    applyHomeSync();
    return;
  }

  if (command == COMMAND_STOP) {
    enterSafeStop(calibrated ? STATUS_IDLE : STATUS_CALIBRATION_REQUIRED);
    Serial.println(F("[I2C 0x08] STOP"));
    return;
  }

  if (!calibrated) {
    enterSafeStop(STATUS_CALIBRATION_REQUIRED);
    Serial.print(F("[CALIBRATION] command rejected="));
    Serial.println(command);
    return;
  }

  if (command == COMMAND_PAUSE) {
    resumeCommand = activeCommand;
    activeCommand = COMMAND_PAUSE;
    outputsOff();
    motorStatus = STATUS_OBSTACLE;
    return;
  }

  if (command == COMMAND_RESUME) {
    activeCommand = resumeCommand == COMMAND_STOP ? COMMAND_OUTBOUND : resumeCommand;
    headingHomebound = activeCommand == COMMAND_RETURN;
    motorStatus = STATUS_RUNNING;
    return;
  }

  const bool nextHomebound = command == COMMAND_RETURN;
  const bool preserveExistingDeadTime =
      directionChangeDeadTimeActive && headingHomebound == nextHomebound;
  const bool reversesLastDrive =
      motorDirectionKnown && lastDrivenHomebound != nextHomebound;
  const bool releaseIntervalIncomplete =
      outputsActive ||
      millis() - lastOutputsOffAt < DIRECTION_CHANGE_DEAD_TIME_MS;
  const bool startNewDeadTime =
      !preserveExistingDeadTime && reversesLastDrive &&
      releaseIntervalIncomplete;
  activeCommand = command;
  resumeCommand = command;
  headingHomebound = nextHomebound;
  motorStatus = STATUS_RUNNING;

  if (startNewDeadTime) {
    outputsOff();
    directionChangeDeadTimeActive = true;
    directionChangeDeadTimeStartedAt = millis();
    Serial.println(F("[MOTOR] direction changed -> all channels OFF for 120ms"));
  } else if (preserveExistingDeadTime) {
    outputsOff();
  } else if (headingHomebound) {
    directionChangeDeadTimeActive = false;
    driveHomebound();
    Serial.println(F("[I2C 0x08] RETURN -> M1/M2/M3/M4 REVERSE_HOME"));
  } else {
    directionChangeDeadTimeActive = false;
    driveOutbound();
    Serial.println(F("[I2C 0x08] OUTBOUND -> M1/M2/M3/M4 FORWARD"));
  }
}

void serviceI2cCommand() {
  noInterrupts();
  const bool hasCommand = commandPending;
  const bool hasKeepalive = keepalivePending;
  const byte command = pendingCommand;
  const byte sequence = pendingSequence;
  commandPending = false;
  keepalivePending = false;
  interrupts();

  if (hasKeepalive) {
    lastControlAt = millis();
    if (!protocolValidated) enterSafeStop(STATUS_PROTOCOL_REQUIRED);
    else if (!calibrated) enterSafeStop(STATUS_CALIBRATION_REQUIRED);
  }
  if (hasCommand) {
    // 같은 command+sequence 재전송은 방향 변경 dead-time을 다시 시작하지
    // 않고 통신 생존만 갱신한다.
    if (command == appliedCommand && sequence == appliedSequence) {
      if (isValidControlCommand(command)) lastControlAt = millis();
    } else {
      applyCommand(command, sequence);
    }
  }
}

void serviceMovement(unsigned long now) {
  if (activeCommand == COMMAND_STOP || activeCommand == COMMAND_PAUSE) return;

  if (now - lastControlAt > WATCHDOG_MS) {
    outputsOff();
    activeCommand = COMMAND_STOP;
    directionChangeDeadTimeActive = false;
    motorStatus = STATUS_WATCHDOG_TIMEOUT;
    Serial.println(F("[WATCHDOG] no SensorUno keepalive -> STOP"));
    return;
  }

  if (directionChangeDeadTimeActive) {
    outputsOff();
    if (now - directionChangeDeadTimeStartedAt <
        DIRECTION_CHANGE_DEAD_TIME_MS) return;
    directionChangeDeadTimeActive = false;
    Serial.println(F("[MOTOR] direction dead-time complete"));
  }

  const bool leftDeviation = digitalRead(LEFT_IR_PIN) == LOW;
  const bool rightDeviation = digitalRead(RIGHT_IR_PIN) == LOW;
  if (leftDeviation && !rightDeviation) correctTowardLeft();
  else if (rightDeviation && !leftDeviation) correctTowardRight();
  else driveForCurrentDirection();
}

void setup() {
  Serial.begin(9600);
  pinMode(M1_FORWARD_LED, OUTPUT);
  pinMode(M1_REVERSE_LED, OUTPUT);
  pinMode(M2_FORWARD_LED, OUTPUT);
  pinMode(M2_REVERSE_LED, OUTPUT);
  pinMode(M3_FORWARD_LED, OUTPUT);
  pinMode(M3_REVERSE_LED, OUTPUT);
  pinMode(M4_FORWARD_LED, OUTPUT);
  pinMode(M4_REVERSE_LED, OUTPUT);
  pinMode(LEFT_IR_PIN, INPUT_PULLUP);
  pinMode(RIGHT_IR_PIN, INPUT_PULLUP);
  outputsOff();
  calibrated = false;
  protocolValidated = false;
  enterSafeStop(STATUS_PROTOCOL_REQUIRED);

  Wire.begin(I2C_ADDRESS);
  Wire.onReceive(receiveCommand);
  Wire.onRequest(sendStatus);
  lastControlAt = millis();
  Serial.println(F("MotorUno proxy ready: address=0x08"));
  Serial.println(F("PROTOCOL_SYNC=7 then HOME_SYNC=6 required"));
  Serial.println(F("4WD proxy: M1/M3 LEFT, M2/M4 RIGHT"));
  Serial.println(F("command 1=FORWARD, 2=straight REVERSE_HOME; no U-turn"));
}

void loop() {
  serviceI2cCommand();
  serviceMovement(millis());
}
