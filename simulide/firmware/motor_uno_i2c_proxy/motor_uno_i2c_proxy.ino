#include <Arduino.h>
#include <Wire.h>

// SimulIDE용 MotorUno 프록시 펌웨어.
// I2C 주소와 command+sequence/status+command+sequence 프레임은 실제
// MotorUno와 같고, AFMotor 실드와 4륜 출력은 네 개 LED로 표시한다.

constexpr byte I2C_ADDRESS = 0x08;

constexpr byte COMMAND_STOP = 0;
constexpr byte COMMAND_OUTBOUND = 1;
constexpr byte COMMAND_RETURN = 2;
constexpr byte COMMAND_PAUSE = 3;
constexpr byte COMMAND_RESUME = 4;
constexpr byte COMMAND_KEEPALIVE = 5;
constexpr byte COMMAND_HOME_SYNC = 6;

constexpr byte STATUS_IDLE = 0;
constexpr byte STATUS_RUNNING = 1;
constexpr byte STATUS_OBSTACLE = 2;
constexpr byte STATUS_WATCHDOG_TIMEOUT = 4;
constexpr byte STATUS_INVALID_COMMAND = 5;
constexpr byte STATUS_CALIBRATION_REQUIRED = 7;

// SimulIDE R260501은 다중 AVR slave가 한 request에서 여러 바이트를
// 안정적으로 반환하지 못한다. SensorUno가 0xE0..0xE2로 논리 응답
// [status, appliedCommand, appliedSequence]의 위치를 고른 뒤 1바이트씩 읽는다.
// 운영 펌웨어의 논리 3바이트 응답 의미는 그대로 유지한다.
constexpr byte STATUS_BYTE_SELECT_BASE = 0xE0;
constexpr byte STATUS_REPLY_SIZE = 3;

// LED는 좌/우 모터 묶음의 정·역방향 구동 신호를 나타낸다.
constexpr byte LEFT_FORWARD_LED = 3;
constexpr byte RIGHT_FORWARD_LED = 5;
constexpr byte LEFT_REVERSE_LED = 6;
constexpr byte RIGHT_REVERSE_LED = 11;

// 실제 핀과 같은 D9/D10 라인센서. 버튼은 편차 검출 프록시다.
constexpr byte LEFT_IR_PIN = 9;
constexpr byte RIGHT_IR_PIN = 10;

constexpr unsigned long TURN_AROUND_MS = 700;
constexpr unsigned long WATCHDOG_MS = 2000;

volatile byte pendingCommand = COMMAND_STOP;
volatile byte pendingSequence = 0;
volatile bool commandPending = false;
volatile bool keepalivePending = false;
volatile byte motorStatus = STATUS_CALIBRATION_REQUIRED;
volatile byte appliedCommand = COMMAND_STOP;
volatile byte appliedSequence = 0;
volatile byte selectedStatusByte = 0;

byte activeCommand = COMMAND_STOP;
byte resumeCommand = COMMAND_STOP;
bool turning = false;
bool calibrated = false;
unsigned long turnStartedAt = 0;
unsigned long lastControlAt = 0;

void outputsOff() {
  digitalWrite(LEFT_FORWARD_LED, LOW);
  digitalWrite(RIGHT_FORWARD_LED, LOW);
  digitalWrite(LEFT_REVERSE_LED, LOW);
  digitalWrite(RIGHT_REVERSE_LED, LOW);
}

void driveForward() {
  outputsOff();
  digitalWrite(LEFT_FORWARD_LED, HIGH);
  digitalWrite(RIGHT_FORWARD_LED, HIGH);
}

void turnLeft() {
  outputsOff();
  digitalWrite(RIGHT_FORWARD_LED, HIGH);
}

void turnRight() {
  outputsOff();
  digitalWrite(LEFT_FORWARD_LED, HIGH);
}

void spinAround() {
  outputsOff();
  digitalWrite(LEFT_REVERSE_LED, HIGH);
  digitalWrite(RIGHT_FORWARD_LED, HIGH);
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
  turning = false;
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
  calibrated = true;
  motorStatus = STATUS_IDLE;
  Serial.println(F("[CALIBRATION] HOME_SYNC -> calibrated / OUTBOUND"));
}

void applyCommand(byte command, byte sequence) {
  appliedCommand = command;
  appliedSequence = sequence;
  lastControlAt = millis();

  if (command > COMMAND_HOME_SYNC) {
    enterSafeStop(STATUS_INVALID_COMMAND);
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
    turning = false;
    outputsOff();
    motorStatus = STATUS_OBSTACLE;
    return;
  }

  if (command == COMMAND_RESUME) {
    activeCommand = resumeCommand == COMMAND_STOP ? COMMAND_OUTBOUND : resumeCommand;
    turning = false;
    motorStatus = STATUS_RUNNING;
    return;
  }

  activeCommand = command;
  resumeCommand = command;
  motorStatus = STATUS_RUNNING;
  if (command == COMMAND_RETURN) {
    turning = true;
    turnStartedAt = millis();
    spinAround();
    Serial.println(F("[I2C 0x08] TURN_AROUND 700ms"));
  } else {
    turning = false;
    driveForward();
    Serial.println(F("[I2C 0x08] CONTINUE"));
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
    if (!calibrated) enterSafeStop(STATUS_CALIBRATION_REQUIRED);
  }
  if (hasCommand) {
    // 같은 command+sequence 재전송은 HOME_SYNC/RETURN 시간을 다시
    // 시작하지 않고 통신 생존만 갱신한다.
    if (command == appliedCommand && sequence == appliedSequence) {
      if (command <= COMMAND_HOME_SYNC) lastControlAt = millis();
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
    turning = false;
    motorStatus = STATUS_WATCHDOG_TIMEOUT;
    Serial.println(F("[WATCHDOG] no SensorUno keepalive -> STOP"));
    return;
  }

  if (turning) {
    if (now - turnStartedAt < TURN_AROUND_MS) {
      spinAround();
      return;
    }
    turning = false;
  }

  const bool leftDeviation = digitalRead(LEFT_IR_PIN) == LOW;
  const bool rightDeviation = digitalRead(RIGHT_IR_PIN) == LOW;
  if (leftDeviation && !rightDeviation) turnLeft();
  else if (rightDeviation && !leftDeviation) turnRight();
  else driveForward();
}

void setup() {
  Serial.begin(9600);
  pinMode(LEFT_FORWARD_LED, OUTPUT);
  pinMode(RIGHT_FORWARD_LED, OUTPUT);
  pinMode(LEFT_REVERSE_LED, OUTPUT);
  pinMode(RIGHT_REVERSE_LED, OUTPUT);
  pinMode(LEFT_IR_PIN, INPUT_PULLUP);
  pinMode(RIGHT_IR_PIN, INPUT_PULLUP);
  outputsOff();
  calibrated = false;
  enterSafeStop(STATUS_CALIBRATION_REQUIRED);

  Wire.begin(I2C_ADDRESS);
  Wire.onReceive(receiveCommand);
  Wire.onRequest(sendStatus);
  lastControlAt = millis();
  Serial.println(F("MotorUno proxy ready: address=0x08"));
  Serial.println(F("HOME_SYNC=6 required; boot status=7 CALIBRATION_REQUIRED"));
}

void loop() {
  serviceI2cCommand();
  serviceMovement(millis());
}
