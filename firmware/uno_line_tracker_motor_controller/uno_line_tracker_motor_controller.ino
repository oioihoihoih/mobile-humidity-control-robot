#include <Arduino.h>
#include <Wire.h>
#include "AFMotor_R4.h"

// [MotorUno / I2C 슬레이브 주소 0x08]
// SensorUno SDA/SCL -> A4/A5
// 왼쪽/오른쪽 IR  -> D9/D10
// 모터 실드 M1/M2 -> 왼쪽/오른쪽 구동 모터

constexpr byte I2C_ADDRESS = 0x08;

constexpr byte COMMAND_STOP = 0;
constexpr byte COMMAND_OUTBOUND = 1;
constexpr byte COMMAND_RETURN = 2;
constexpr byte COMMAND_PAUSE = 3;
constexpr byte COMMAND_RESUME = 4;
// SensorUno는 주행 중 500ms 정도마다 이 값을 보내 통신 생존을 알린다.
// KEEPALIVE는 실제 주행 명령을 변경하지 않는다.
constexpr byte COMMAND_KEEPALIVE = 5;
// HOME의 넓은 검은 마커 위에 정차했음을 두 IR 센서로 확인한 뒤에만
// 주행 interlock을 해제한다. 확인 과정에서는 모터를 움직이지 않는다.
constexpr byte COMMAND_HOME_SYNC = 6;

constexpr byte STATUS_IDLE = 0;
constexpr byte STATUS_RUNNING = 1;
constexpr byte STATUS_OBSTACLE = 2;
// ZONE2/ZONE99 도착은 RFID가 결정하지만, RFID가 없는 HOME 종점의
// 검은 정지 마커에는 기존 값 3을 그대로 사용한다.
constexpr byte STATUS_STOP_LINE = 3;
constexpr byte STATUS_WATCHDOG_TIMEOUT = 4;
constexpr byte STATUS_INVALID_COMMAND = 5;
// 기존 안전 고장값 6을 그대로 사용한다. SensorUno는 값 6을 받으면 주행과
// 액추에이터를 모두 안전 정지한다. 출동 방향의 예기치 않은 넓은 검은
// 마커나 HOME 출발 마커를 제한시간 안에 벗어나지 못한 경우에 사용한다.
constexpr byte STATUS_UNEXPECTED_MARKER = 6;
constexpr byte STATUS_CALIBRATION_REQUIRED = 7;

constexpr byte LEFT_IR_PIN = 9;
constexpr byte RIGHT_IR_PIN = 10;

// 센서 모듈에 따라 검은색 출력 극성이 반대일 수 있다. 실제 바닥/라인을
// 시리얼 진단으로 확인한 뒤 이 한 값만 변경한다.
constexpr bool LINE_BLACK_IS_HIGH = true;

// RFID 정지 동작만 분리 검증할 때 사용하는 벤치 모드다. 운영 빌드는
// 반드시 false로 두어 라인센서와 HOME 마커 안전 로직을 우회하지 않는다.
constexpr bool BENCH_RFID_ONLY_MODE = false;

// 6V(AA 4개) + 현재 2WD 실물에서 PWM 150은 기동하지 않았고 255에서
// 좌우 모터 회전을 확인했다. 첫 라인/RFID 시험은 검증된 값으로 시작하고,
// 실제 선 추종 안정성을 본 뒤 감속값을 다시 측정한다.
constexpr byte MOTOR_SPEED = 255;
constexpr unsigned long RETURN_TURN_MS = 700;
constexpr unsigned long HOME_MARKER_CLEAR_TIMEOUT_MS = 2000;
// 현재 배선에서는 검정=HIGH, 바닥=LOW이며 검은 선이 두 센서 사이에 있을
// 때 LOW/LOW로 직진한다. HIGH/HIGH는 두 센서가 모두 검정을 읽는 넓은
// 마커다. 짧은 노이즈에는 즉시 멈추되 아래 시간 이상일 때만 확정한다.
// 실제 라인 유실도 LOW/LOW일 수 있어 이 2센서 배치만으로는 직진과 구분할
// 수 없으며, 아래 검사는 '라인 유실 검출'을 의미하지 않는다.
constexpr unsigned long BOTH_HIGH_CONFIRM_MS = 300;
constexpr unsigned long SENSOR_LOG_INTERVAL_MS = 1000;
// 주행/일시정지 중 이 시간 동안 SensorUno의 유효 명령이나 KEEPALIVE가
// 오지 않으면 마지막 명령을 신뢰하지 않고 MotorUno가 자체 정지한다.
constexpr unsigned long CONTROL_WATCHDOG_MS = 2000;

AF_DCMotor motorLeft(1);
AF_DCMotor motorRight(2);

// I2C 콜백은 인터럽트 안에서 실행되므로 명령값만 저장하고
// 실제 모터 제어와 시리얼 출력은 loop()에서 처리한다.
volatile byte requestedCommand = COMMAND_STOP;
volatile byte requestedSequence = 0;
volatile bool commandPending = false;
// KEEPALIVE를 별도 플래그로 받아, KEEPALIVE가 아직 처리하지 않은
// OUTBOUND/RETURN/STOP 명령을 덮어쓰지 못하게 한다.
volatile bool keepalivePending = false;
volatile byte motorStatus = STATUS_CALIBRATION_REQUIRED;
// SensorUno가 오래된 상태값을 새 명령의 ACK로 오인하지 않도록, 실제로
// loop()에서 적용을 끝낸 명령과 그 8비트 순번을 상태와 함께 반환한다.
volatile byte appliedCommand = COMMAND_STOP;
volatile byte appliedSequence = 0;

byte activeCommand = COMMAND_STOP;
// COMMAND_RETURN(2)는 HOME 전용이 아니라 현재 물리 방향을 180도 뒤집는
// 명령이다. STOP/OUTBOUND(현재 방향 진행)는 이 값을 보존한다. 전원 켤 때
// 방향값은 outbound로 시작하지만 HOME_SYNC가 성공하기 전에는 주행할 수
// 없다.
bool headingHomebound = false;
bool calibrated = false;
bool returnTurnActive = false;
bool pausedBySensorUno = false;
bool homeMarkerLatched = false;
bool lineFollowingStarted = false;
bool homeMarkerClearing = false;
bool homeMarkerDetectionArmed = false;
bool bothHighCandidate = false;
unsigned long returnTurnStartedAt = 0;
unsigned long returnTurnElapsedAtPause = 0;
unsigned long homeMarkerClearStartedAt = 0;
unsigned long homeMarkerClearElapsedAtPause = 0;
unsigned long bothHighStartedAt = 0;
unsigned long lastSensorLogAt = 0;
unsigned long lastValidControlAt = 0;

void stopMotors() {
  motorLeft.run(RELEASE);
  motorRight.run(RELEASE);
}

void driveForward() {
  motorLeft.run(FORWARD);
  motorRight.run(FORWARD);
}

void turnLeft() {
  motorLeft.run(RELEASE);
  motorRight.run(FORWARD);
}

void turnRight() {
  motorLeft.run(FORWARD);
  motorRight.run(RELEASE);
}

void spinForReturn() {
  motorLeft.run(BACKWARD);
  motorRight.run(FORWARD);
}

void resetMarkerCandidateState() {
  bothHighCandidate = false;
  bothHighStartedAt = 0;
}

void resetHomeMarkerState() {
  homeMarkerLatched = false;
  lineFollowingStarted = false;
  homeMarkerClearing = false;
  homeMarkerDetectionArmed = false;
  homeMarkerClearStartedAt = 0;
  homeMarkerClearElapsedAtPause = 0;
}

bool isBlack(byte pin) {
  const int blackLevel = LINE_BLACK_IS_HIGH ? HIGH : LOW;
  return digitalRead(pin) == blackLevel;
}

void receiveI2cCommand(int byteCount) {
  // 데이터가 없는 주소 탐색은 허용한다. 그 외에는 command+sequence
  // 정확히 2바이트만 정상 프레임이며, 잘린/긴 프레임은 안전 정지시킨다.
  if (byteCount <= 0) return;
  if (byteCount != 2 || Wire.available() < 2) {
    while (Wire.available()) Wire.read();
    requestedCommand = 0xFF;
    requestedSequence = appliedSequence;
    commandPending = true;
    return;
  }

  const byte command = static_cast<byte>(Wire.read());
  const byte sequence = static_cast<byte>(Wire.read());

  if (command == COMMAND_KEEPALIVE) {
    keepalivePending = true;
  } else {
    requestedCommand = command;
    requestedSequence = sequence;
    commandPending = true;
  }
  while (Wire.available()) Wire.read();
}

void sendI2cStatus() {
  // 이 콜백은 TWI ISR 안에서 실행되므로 loop()가 세 값을 바꿀 수 없다.
  // 콜백 안에서는 Serial/millis/모터 제어를 절대 실행하지 않는다.
  const byte statusSnapshot = motorStatus;
  const byte commandSnapshot = appliedCommand;
  const byte sequenceSnapshot = appliedSequence;
  Wire.write(statusSnapshot);
  Wire.write(commandSnapshot);
  Wire.write(sequenceSnapshot);
}

bool isValidControlCommand(byte command) {
  return command <= COMMAND_HOME_SYNC;
}

void publishAppliedCommand(byte command, byte sequence) {
  // sequence를 마지막에 기록해 새 ACK의 commit 값으로 사용한다. SREG를
  // 복원하므로 호출 전 인터럽트 상태를 임의로 바꾸지 않는다.
  const byte savedSreg = SREG;
  noInterrupts();
  appliedCommand = command;
  appliedSequence = sequence;
  SREG = savedSreg;
}

void enterSafeStop(byte status) {
  stopMotors();
  activeCommand = COMMAND_STOP;
  returnTurnActive = false;
  returnTurnStartedAt = 0;
  pausedBySensorUno = false;
  returnTurnElapsedAtPause = 0;
  resetHomeMarkerState();
  resetMarkerCandidateState();
  motorStatus = status;
}

void applyHomeSync() {
  // HOME_SYNC 자체는 탐색 동작이 아니다. 먼저 완전 정지한 다음 현재 센서
  // 값만 검사하며, 실패하면 이전 calibration도 신뢰하지 않는다.
  calibrated = false;
  enterSafeStop(STATUS_CALIBRATION_REQUIRED);
  lastValidControlAt = millis();

  if (!BENCH_RFID_ONLY_MODE &&
      (!isBlack(LEFT_IR_PIN) || !isBlack(RIGHT_IR_PIN))) {
    Serial.println(F("[CALIBRATION] HOME_SYNC rejected: HOME marker absent"));
    return;
  }

  headingHomebound = false;
  calibrated = true;
  motorStatus = STATUS_IDLE;
  Serial.println(F("[CALIBRATION] HOME_SYNC -> calibrated / heading=OUTBOUND"));
}

void applyCommand(byte command) {
  if (!isValidControlCommand(command)) {
    enterSafeStop(STATUS_INVALID_COMMAND);
    Serial.print(F("[I2C] INVALID command="));
    Serial.print(command);
    Serial.println(F(" -> safe stop"));
    return;
  }

  // 유효한 제어 명령을 받은 시점부터 watchdog 시간을 다시 센다.
  // 명시적 STOP 상태에서는 watchdog이 동작하지 않지만 기록은 갱신한다.
  lastValidControlAt = millis();

  if (command == COMMAND_KEEPALIVE) {
    // Calibration 전 KEEPALIVE도 유효하지만 interlock 상태를 해제하지는
    // 않는다. 어떤 경우에도 이 경로에서 모터가 움직이지 않게 재확인한다.
    if (!calibrated) enterSafeStop(STATUS_CALIBRATION_REQUIRED);
    return;
  }

  if (command == COMMAND_HOME_SYNC) {
    applyHomeSync();
    return;
  }

  if (command == COMMAND_STOP) {
    enterSafeStop(calibrated ? STATUS_IDLE : STATUS_CALIBRATION_REQUIRED);
    Serial.println(F("[I2C] STOP command"));
    return;
  }

  // OUTBOUND/RETURN/PAUSE/RESUME는 HOME에서 물리적으로 동기화되기 전까지
  // 모두 ACK하되 실행하지 않는다. processI2cInbox()가 원래 command와
  // sequence를 그대로 publish하므로 SensorUno는 거절된 프레임도 식별한다.
  if (!calibrated) {
    enterSafeStop(STATUS_CALIBRATION_REQUIRED);
    Serial.print(F("[CALIBRATION] command rejected="));
    Serial.println(command);
    return;
  }

  if (command == COMMAND_PAUSE) {
    // HOME 마커 도착은 이미 래치된 최종 정지다. 뒤늦은 PAUSE가 상태를
    // OBSTACLE로 덮으면 SensorUno가 HOME 도착을 놓칠 수 있으므로 무시한다.
    if (activeCommand == COMMAND_STOP || homeMarkerLatched) {
      Serial.println(F("[I2C] PAUSE ignored: motors are already stopped"));
      return;
    }
    if (!pausedBySensorUno) {
      pausedBySensorUno = true;
      if (returnTurnActive) {
        returnTurnElapsedAtPause = millis() - returnTurnStartedAt;
      }
      if (homeMarkerClearing) {
        homeMarkerClearElapsedAtPause = millis() - homeMarkerClearStartedAt;
      }
      stopMotors();
      motorStatus = STATUS_OBSTACLE;
      Serial.println(F("[I2C] PAUSE -> obstacle stop"));
    }
    return;
  }

  if (command == COMMAND_RESUME) {
    if (!pausedBySensorUno || activeCommand == COMMAND_STOP) {
      Serial.println(F("[I2C] RESUME ignored: no resumable movement"));
      return;
    }
    pausedBySensorUno = false;
    if (returnTurnActive) {
      returnTurnStartedAt = millis() - returnTurnElapsedAtPause;
    }
    if (homeMarkerClearing) {
      homeMarkerClearStartedAt = millis() - homeMarkerClearElapsedAtPause;
    }
    resetMarkerCandidateState();
    motorStatus = STATUS_RUNNING;
    Serial.println(F("[I2C] RESUME -> continue previous movement"));
    return;
  }

  activeCommand = command;
  returnTurnActive = false;
  pausedBySensorUno = false;
  returnTurnElapsedAtPause = 0;
  resetHomeMarkerState();
  resetMarkerCandidateState();

  motorStatus = STATUS_RUNNING;
  if (command == COMMAND_RETURN) {
    headingHomebound = !headingHomebound;
    returnTurnActive = true;
    returnTurnStartedAt = millis();
    spinForReturn();
    Serial.print(F("[I2C] TURN_AROUND -> heading="));
    Serial.println(headingHomebound ? F("HOMEBOUND") : F("OUTBOUND"));
  } else {
    Serial.print(F("[I2C] CONTINUE -> heading="));
    Serial.println(headingHomebound ? F("HOMEBOUND") : F("OUTBOUND"));
  }
}

void followLine() {
  if (BENCH_RFID_ONLY_MODE) {
    driveForward();
    motorStatus = STATUS_RUNNING;
    return;
  }

  if (homeMarkerLatched) {
    stopMotors();
    motorStatus = STATUS_STOP_LINE;
    return;
  }

  const int leftValue = digitalRead(LEFT_IR_PIN);
  const int rightValue = digitalRead(RIGHT_IR_PIN);
  const bool leftBlack = leftValue == (LINE_BLACK_IS_HIGH ? HIGH : LOW);
  const bool rightBlack = rightValue == (LINE_BLACK_IS_HIGH ? HIGH : LOW);

  // HOME의 검은 정지 마커 위에서 출발할 수 있도록, 새 주행의 첫 상태가
  // 양쪽 모두 검정이면 이 마커를 먼저 직진해서 빠져나온다. ZONE2/ZONE99는
  // 연속 라인 위 RFID이므로 보통 이 분기를 사용하지 않는다.
  if (!lineFollowingStarted) {
    lineFollowingStarted = true;
    // HOME 마커를 밖으로 밀며 출발하는 것은 실제 진행 방향이 outbound일
    // 때만 허용한다. homebound 재개라면 같은 마커를 다시 HOME으로 확정한다.
    if (!headingHomebound && leftBlack && rightBlack) {
      homeMarkerClearing = true;
      homeMarkerDetectionArmed = false;
      homeMarkerClearStartedAt = millis();
      homeMarkerClearElapsedAtPause = 0;
      driveForward();
      motorStatus = STATUS_RUNNING;
      Serial.println(F("[DEPARTURE] HOME marker detected -> clearing forward"));
      return;
    }
    homeMarkerDetectionArmed = true;
  }

  if (homeMarkerClearing) {
    if (!leftBlack || !rightBlack) {
      homeMarkerClearing = false;
      homeMarkerDetectionArmed = true;
      homeMarkerClearElapsedAtPause = 0;
      Serial.println(F("[DEPARTURE] HOME marker cleared -> continuous tracking"));
    } else {
      driveForward();
      motorStatus = STATUS_RUNNING;
      return;
    }
  }

  if (leftBlack && rightBlack) {
    // RETURN도 단 한 번의 HIGH/HIGH를 HOME으로 확정하면 노이즈나 순간
    // 이탈을 HOME 도착으로 오인한다. 두 방향 모두 후보 순간에는 먼저
    // 물리 정지하고, 300ms 연속 유지될 때만 방향별 최종 상태를 확정한다.
    stopMotors();
    if (!bothHighCandidate) {
      bothHighCandidate = true;
      bothHighStartedAt = millis();
      Serial.println(F("[LINE] both sensors HIGH -> provisional safety stop"));
    } else if (millis() - bothHighStartedAt >= BOTH_HIGH_CONFIRM_MS) {
      if (headingHomebound && homeMarkerDetectionArmed) {
        // command 2 직후뿐 아니라 ZONE2 RFID 정지 후 command 1로 같은
        // 복귀 방향을 계속 가는 동안에도 물리 방향값은 HOMEBOUND로 남는다.
        // MotorUno는 목적지를 모르므로 최종 HOME 판단은 SensorUno가 한다.
        homeMarkerLatched = true;
        motorStatus = STATUS_STOP_LINE;
        resetMarkerCandidateState();
        Serial.println(F("[HOME] sustained return marker -> latched stop"));
      } else {
        enterSafeStop(STATUS_UNEXPECTED_MARKER);
        Serial.println(F("[SAFETY] unexpected outbound black marker -> latched stop"));
      }
    }
  } else {
    if (bothHighCandidate) {
      bothHighCandidate = false;
      bothHighStartedAt = 0;
      Serial.println(F("[LINE] normal pattern recovered before confirmation"));
    }

    if (!leftBlack && !rightBlack) {
      driveForward();
      motorStatus = STATUS_RUNNING;
    } else if (leftBlack && !rightBlack) {
      turnLeft();
      motorStatus = STATUS_RUNNING;
    } else {  // left background / right black
      turnRight();
      motorStatus = STATUS_RUNNING;
    }
  }

  if (millis() - lastSensorLogAt >= SENSOR_LOG_INTERVAL_MS) {
    lastSensorLogAt = millis();
    Serial.print(F("[LINE] L="));
    Serial.print(leftValue);
    Serial.print(F(" R="));
    Serial.print(rightValue);
    Serial.print(F(" status="));
    Serial.println(motorStatus);
  }
}

void updateHomeMarkerClearSafety() {
  if (!homeMarkerClearing || pausedBySensorUno) return;
  if (millis() - homeMarkerClearStartedAt < HOME_MARKER_CLEAR_TIMEOUT_MS) return;

  // HOME 마커를 실제로 벗어나지 못했으면 계속 강제 직진하지 않는다.
  if (!isBlack(LEFT_IR_PIN) || !isBlack(RIGHT_IR_PIN)) return;

  enterSafeStop(STATUS_UNEXPECTED_MARKER);
  Serial.println(F("[SAFETY] HOME marker clear timeout -> motors stopped"));
}

void processI2cInbox() {
  if (!commandPending && !keepalivePending) return;

  noInterrupts();
  const bool hasCommand = commandPending;
  const byte command = requestedCommand;
  const byte sequence = requestedSequence;
  const bool hasKeepalive = keepalivePending;
  commandPending = false;
  keepalivePending = false;
  interrupts();

  // 실제 명령을 먼저 처리한다. 같은 순간 도착한 KEEPALIVE는 명령을
  // 바꾸지 않고 watchdog 시간만 한 번 더 갱신한다.
  if (hasCommand) {
    if (command == appliedCommand && sequence == appliedSequence) {
      // 같은 프레임의 재전송은 RETURN 회전이나 출발 상태를 다시 시작하지
      // 않는다. 유효한 중복 프레임은 통신 생존 신호로만 인정한다.
      if (isValidControlCommand(command)) lastValidControlAt = millis();
    } else {
      applyCommand(command);
      // 모터 상태 변경을 모두 끝낸 뒤에만 이 순번을 ACK한다.
      publishAppliedCommand(command, sequence);
    }
  }
  if (hasKeepalive) applyCommand(COMMAND_KEEPALIVE);
}

void updateControlWatchdog() {
  // 명시적 STOP, 잘못된 명령에 의한 안전 정지, 이미 발생한 watchdog
  // 정지는 시간이 지나도 그대로 유지한다.
  if (activeCommand == COMMAND_STOP) return;
  if (millis() - lastValidControlAt <= CONTROL_WATCHDOG_MS) return;

  enterSafeStop(STATUS_WATCHDOG_TIMEOUT);
  Serial.println(F("[SAFETY] SensorUno keepalive timeout -> motors stopped"));
}

// USB 진단은 모터를 임의로 구동하지 않는다.
// '?' : 라인 센서와 현재 상태 조회, '0' : 즉시 안전 정지.
void handleSerialDiagnostic() {
  if (!Serial.available()) return;
  const char command = static_cast<char>(Serial.read());

  if (command == '0') {
    enterSafeStop(calibrated ? STATUS_IDLE : STATUS_CALIBRATION_REQUIRED);
    lastValidControlAt = millis();
    Serial.println(F("[SERIAL] 0 -> motors RELEASED"));
    return;
  }
  if (command != '?') return;

  Serial.print(F("[DIAG] IR_LEFT_D9="));
  Serial.print(digitalRead(LEFT_IR_PIN));
  Serial.print(F(" IR_RIGHT_D10="));
  Serial.print(digitalRead(RIGHT_IR_PIN));
  Serial.print(F(" command="));
  Serial.print(activeCommand);
  Serial.print(F(" heading="));
  Serial.print(headingHomebound ? F("HOMEBOUND") : F("OUTBOUND"));
  Serial.print(F(" status="));
  Serial.print(motorStatus);
  Serial.print(F(" calibrated="));
  Serial.print(calibrated ? 1 : 0);
  Serial.print(F(" paused="));
  Serial.print(pausedBySensorUno ? 1 : 0);
  Serial.print(F(" homeMarker="));
  Serial.print(homeMarkerLatched ? 1 : 0);
  Serial.print(F(" markerClearing="));
  Serial.print(homeMarkerClearing ? 1 : 0);
  Serial.print(F(" markerArmed="));
  Serial.print(homeMarkerDetectionArmed ? 1 : 0);
  Serial.print(F(" bothHighCandidate="));
  Serial.print(bothHighCandidate ? 1 : 0);
  Serial.print(F(" bothHighMs="));
  Serial.println(bothHighCandidate ? millis() - bothHighStartedAt : 0);
}

void setup() {
  Serial.begin(9600);

  pinMode(LEFT_IR_PIN, INPUT);
  pinMode(RIGHT_IR_PIN, INPUT);

  motorLeft.setSpeed(MOTOR_SPEED);
  motorRight.setSpeed(MOTOR_SPEED);
  calibrated = false;
  headingHomebound = false;
  enterSafeStop(STATUS_CALIBRATION_REQUIRED);

  Wire.begin(I2C_ADDRESS);
  Wire.onReceive(receiveI2cCommand);
  Wire.onRequest(sendI2cStatus);

  Serial.println(F("[BOOT] MotorUno I2C slave started"));
  Serial.println(F("[BOOT] address=0x08 SDA=A4 SCL=A5"));
  Serial.println(F("[BOOT] IR=D9/D10, HC-SR04 is on SensorUno"));
  Serial.println(F("[BOOT] protocol: 0=STOP 1=OUTBOUND 2=RETURN 3=PAUSE 4=RESUME 5=KEEPALIVE 6=HOME_SYNC"));
  Serial.println(F("[BOOT] ZONE arrival: SensorUno RFID -> STOP command"));
  Serial.println(F("[BOOT] HOME arrival: return-direction HIGH/HIGH marker"));
  Serial.println(F("[BOOT] initial heading=OUTBOUND; HOME_SYNC required before motion"));
  if (BENCH_RFID_ONLY_MODE) {
    Serial.println(F("[BOOT] BENCH RFID-ONLY: line/marker bypassed"));
  }
  Serial.println(F("[BOOT] status: 0=IDLE 1=RUNNING 2=PAUSED 3=HOME_MARKER 4=WATCHDOG 5=INVALID 6=UNEXPECTED_MARKER 7=CALIBRATION_REQUIRED"));
  Serial.print(F("[BOOT] HOME marker clear timeout="));
  Serial.print(HOME_MARKER_CLEAR_TIMEOUT_MS);
  Serial.println(F("ms"));
  Serial.print(F("[BOOT] black-marker confirmation="));
  Serial.print(BOTH_HIGH_CONFIRM_MS);
  Serial.println(F("ms (motors stop immediately while confirming)"));
  Serial.print(F("[BOOT] control watchdog="));
  Serial.print(CONTROL_WATCHDOG_MS);
  Serial.println(F("ms"));
  Serial.println(F("[BOOT] waiting for SensorUno command"));
}

void loop() {
  processI2cInbox();
  updateHomeMarkerClearSafety();
  updateControlWatchdog();
  handleSerialDiagnostic();

  if (activeCommand == COMMAND_STOP || pausedBySensorUno) return;

  if (returnTurnActive) {
    if (millis() - returnTurnStartedAt < RETURN_TURN_MS) {
      spinForReturn();
      return;
    }
    returnTurnActive = false;
    Serial.println(F("[MOTOR] return turn complete -> line tracking"));
  }

  followLine();
}
