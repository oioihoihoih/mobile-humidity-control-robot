#include <Arduino.h>
#include <Wire.h>
#include "AFMotor_R4.h"

// [MotorUno / I2C 슬레이브 주소 0x08]
// SensorUno SDA/SCL -> A4/A5
// 왼쪽/오른쪽 IR  -> D9/D10
// 모터 실드 M1/M3 -> 왼쪽 축, M2/M4 -> 오른쪽 축
// M1/M2는 기존 모터, M3/M4는 새 N20 1:298 후륜 모터

constexpr byte I2C_ADDRESS = 0x08;

constexpr byte COMMAND_STOP = 0;
// v2 이동 명령은 구형 1/2와 값 자체를 분리한다. 어느 한 보드만 구형으로
// 교체돼도 상대 보드가 반대 방향 의미를 실행하지 않고 INVALID로 정지한다.
constexpr byte COMMAND_OUTBOUND = 0x11;
constexpr byte COMMAND_RETURN = 0x12;
constexpr byte COMMAND_PAUSE = 3;
constexpr byte COMMAND_RESUME = 4;
// SensorUno는 주행 중 500ms 정도마다 이 값을 보내 통신 생존을 알린다.
// KEEPALIVE는 실제 주행 명령을 변경하지 않는다.
constexpr byte COMMAND_KEEPALIVE = 5;
// HOME의 넓은 검은 마커 위에 정차했음을 두 IR 센서로 확인한 뒤에만
// 주행 interlock을 해제한다. 확인 과정에서는 모터를 움직이지 않는다.
constexpr byte COMMAND_HOME_SYNC = 6;
// 이동 명령의 의미가 4모터 절대 전진/후진으로 바뀐 세대다. SensorUno가
// 이 명령을 먼저 exact ACK로 확인하지 않으면 HOME_SYNC와 주행을 거절한다.
// 구형 MotorUno는 7을 invalid로 안전 정지하므로 부분 업로드도 움직이지 않는다.
constexpr byte COMMAND_PROTOCOL_SYNC = 7;

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
constexpr byte STATUS_PROTOCOL_REQUIRED = 8;

constexpr byte LEFT_IR_PIN = 9;
constexpr byte RIGHT_IR_PIN = 10;

// 센서 모듈에 따라 검은색 출력 극성이 반대일 수 있다. 실제 바닥/라인을
// 시리얼 진단으로 확인한 뒤 이 한 값만 변경한다.
constexpr bool LINE_BLACK_IS_HIGH = true;

// RFID 정지 동작만 분리 검증할 때 사용하는 벤치 모드다. 운영 빌드는
// 반드시 false로 두어 라인센서와 HOME 마커 안전 로직을 우회하지 않는다.
constexpr bool BENCH_RFID_ONLY_MODE = false;

// 아래 255는 이전 2모터 벤치의 기동 관찰값일 뿐 현재 4모터 검증 근거가 아니다.
// 기존 축과 N20 1:298 후륜은 기어비와 바퀴 크기가 다르므로 PWM을 분리한다.
// 현재는 확인된 기동값 255로 시작한다. 차체를 바닥에 놓고 실제 속도를 잰 뒤
// 더 빠른 축의 값만 낮춰 직진 속도를 맞춘다.
constexpr byte EXISTING_AXLE_SPEED = 255;
constexpr byte N20_REAR_AXLE_SPEED = 255;
// 라인 보정은 바퀴를 반대 방향으로 돌리지 않고 한쪽 축만 살짝 감속한다.
constexpr byte TRACKING_INNER_PERCENT = 80;
// 현재 IR 센서가 후진 시 차체의 뒤쪽(주행 기준 trailing)에 남는 배치다.
// 실물에서 후진 보정이 반대로 나타나면 이 값만 false로 바꾼다.
constexpr bool MIRROR_LINE_CORRECTION_WHEN_REVERSING = true;
// 전진↔후진이 바뀌면 모든 H-bridge 출력을 먼저 해제한다. 명시적 STOP 직후
// 반대 방향 명령이 와도 마지막 RELEASE부터 이 시간은 반드시 보장한다.
constexpr unsigned long DIRECTION_CHANGE_DEAD_TIME_MS = 120;
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

// 직선 왕복 차체이므로 실드의 네 채널을 모두 사용한다.
// M1과 M3가 같은 물리 방향, M2와 M4가 같은 물리 방향으로 회전해야 한다.
AF_DCMotor motorExistingLeft(1);
AF_DCMotor motorExistingRight(2);
AF_DCMotor motorN20RearLeft(3);
AF_DCMotor motorN20RearRight(4);

// I2C 콜백은 인터럽트 안에서 실행되므로 명령값만 저장하고
// 실제 모터 제어와 시리얼 출력은 loop()에서 처리한다.
volatile byte requestedCommand = COMMAND_STOP;
volatile byte requestedSequence = 0;
volatile bool commandPending = false;
// KEEPALIVE를 별도 플래그로 받아, KEEPALIVE가 아직 처리하지 않은
// OUTBOUND/RETURN/STOP 명령을 덮어쓰지 못하게 한다.
volatile bool keepalivePending = false;
volatile byte motorStatus = STATUS_PROTOCOL_REQUIRED;
// SensorUno가 오래된 상태값을 새 명령의 ACK로 오인하지 않도록, 실제로
// loop()에서 적용을 끝낸 명령과 그 8비트 순번을 상태와 함께 반환한다.
volatile byte appliedCommand = COMMAND_STOP;
volatile byte appliedSequence = 0;

byte activeCommand = COMMAND_STOP;
// 차체를 돌리지 않는 직선 경로다. OUTBOUND(0x11)는 네 바퀴 전진,
// RETURN(0x12)는 네 바퀴 후진이며, 같은 명령을 반복해도 방향이 뒤집히지 않는다.
bool headingHomebound = false;
bool calibrated = false;
bool protocolValidated = false;
bool pausedBySensorUno = false;
bool directionChangeDeadTimeActive = false;
bool motorOutputsEnergized = false;
bool motorDirectionKnown = false;
bool lastDrivenHomebound = false;
bool homeMarkerLatched = false;
bool lineFollowingStarted = false;
bool homeMarkerClearing = false;
bool homeMarkerDetectionArmed = false;
bool bothHighCandidate = false;
unsigned long homeMarkerClearStartedAt = 0;
unsigned long homeMarkerClearElapsedAtPause = 0;
unsigned long directionChangeDeadTimeStartedAt = 0;
unsigned long lastMotorReleaseAt = 0;
unsigned long bothHighStartedAt = 0;
unsigned long lastSensorLogAt = 0;
unsigned long lastValidControlAt = 0;

byte scaledTrackingSpeed(byte fullSpeed) {
  return static_cast<byte>((static_cast<unsigned int>(fullSpeed) *
                            TRACKING_INNER_PERCENT) / 100U);
}

void setLeftPairSpeed(bool reduced) {
  motorExistingLeft.setSpeed(reduced ? scaledTrackingSpeed(EXISTING_AXLE_SPEED)
                                     : EXISTING_AXLE_SPEED);
  motorN20RearLeft.setSpeed(reduced ? scaledTrackingSpeed(N20_REAR_AXLE_SPEED)
                                    : N20_REAR_AXLE_SPEED);
}

void setRightPairSpeed(bool reduced) {
  motorExistingRight.setSpeed(reduced ? scaledTrackingSpeed(EXISTING_AXLE_SPEED)
                                      : EXISTING_AXLE_SPEED);
  motorN20RearRight.setSpeed(reduced ? scaledTrackingSpeed(N20_REAR_AXLE_SPEED)
                                     : N20_REAR_AXLE_SPEED);
}

void setCruiseSpeeds() {
  setLeftPairSpeed(false);
  setRightPairSpeed(false);
}

void runLeftPair(uint8_t direction) {
  motorExistingLeft.run(direction);
  motorN20RearLeft.run(direction);
}

void runRightPair(uint8_t direction) {
  motorExistingRight.run(direction);
  motorN20RearRight.run(direction);
}

void stopMotors() {
  motorExistingLeft.run(RELEASE);
  motorExistingRight.run(RELEASE);
  motorN20RearLeft.run(RELEASE);
  motorN20RearRight.run(RELEASE);
  if (motorOutputsEnergized) {
    motorOutputsEnergized = false;
    lastMotorReleaseAt = millis();
  }
}

void markMotorDrive(bool homebound) {
  motorOutputsEnergized = true;
  motorDirectionKnown = true;
  lastDrivenHomebound = homebound;
}

void driveOutbound() {
  setCruiseSpeeds();
  runLeftPair(FORWARD);
  runRightPair(FORWARD);
  markMotorDrive(false);
}

void driveHomebound() {
  setCruiseSpeeds();
  runLeftPair(BACKWARD);
  runRightPair(BACKWARD);
  markMotorDrive(true);
}

void driveForCurrentHeading() {
  if (headingHomebound) {
    driveHomebound();
  } else {
    driveOutbound();
  }
}

// 제자리 회전은 하지 않는다. 네 바퀴가 모두 현재 주행 방향을 유지하면서
// 안쪽 축만 잠시 감속해 선을 부드럽게 보정한다. 후진할 때는 센서가 진행
// 방향의 뒤쪽에 있으므로 전진 때와 반대쪽 축을 감속한다.
void correctTowardLeft() {
  const uint8_t direction = headingHomebound ? BACKWARD : FORWARD;
  const bool mirror =
      headingHomebound && MIRROR_LINE_CORRECTION_WHEN_REVERSING;
  setLeftPairSpeed(!mirror);
  setRightPairSpeed(mirror);
  runLeftPair(direction);
  runRightPair(direction);
  markMotorDrive(headingHomebound);
}

void correctTowardRight() {
  const uint8_t direction = headingHomebound ? BACKWARD : FORWARD;
  const bool mirror =
      headingHomebound && MIRROR_LINE_CORRECTION_WHEN_REVERSING;
  setLeftPairSpeed(mirror);
  setRightPairSpeed(!mirror);
  runLeftPair(direction);
  runRightPair(direction);
  markMotorDrive(headingHomebound);
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
  return command == COMMAND_STOP || command == COMMAND_OUTBOUND ||
         command == COMMAND_RETURN || command == COMMAND_PAUSE ||
         command == COMMAND_RESUME || command == COMMAND_KEEPALIVE ||
         command == COMMAND_HOME_SYNC || command == COMMAND_PROTOCOL_SYNC;
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
  pausedBySensorUno = false;
  directionChangeDeadTimeActive = false;
  directionChangeDeadTimeStartedAt = 0;
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
    if (!protocolValidated) enterSafeStop(STATUS_PROTOCOL_REQUIRED);
    else if (!calibrated) enterSafeStop(STATUS_CALIBRATION_REQUIRED);
    return;
  }

  if (command == COMMAND_PROTOCOL_SYNC) {
    calibrated = false;
    protocolValidated = true;
    enterSafeStop(STATUS_PROTOCOL_REQUIRED);
    Serial.println(F("[PROTOCOL] 4WD v2 sync confirmed; HOME_SYNC required"));
    return;
  }

  // 4모터 의미를 모르는 구형 SensorUno가 STOP/HOME_SYNC를 보내더라도
  // 출력은 계속 RELEASE하고 v2 이동 명령을 절대 실행하지 않는다.
  if (!protocolValidated) {
    enterSafeStop(STATUS_PROTOCOL_REQUIRED);
    Serial.println(F("[PROTOCOL] command rejected: v2 sync required"));
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
    if (homeMarkerClearing) {
      homeMarkerClearStartedAt = millis() - homeMarkerClearElapsedAtPause;
    }
    resetMarkerCandidateState();
    motorStatus = STATUS_RUNNING;
    Serial.println(F("[I2C] RESUME -> continue previous movement"));
    return;
  }

  const bool nextHomebound = command == COMMAND_RETURN;
  const bool preserveExistingDeadTime =
      directionChangeDeadTimeActive && headingHomebound == nextHomebound;
  const bool reversesLastDrive =
      motorDirectionKnown && lastDrivenHomebound != nextHomebound;
  const bool releaseIntervalIncomplete =
      motorOutputsEnergized ||
      millis() - lastMotorReleaseAt < DIRECTION_CHANGE_DEAD_TIME_MS;
  const bool startNewDeadTime =
      !preserveExistingDeadTime && reversesLastDrive &&
      releaseIntervalIncomplete;

  activeCommand = command;
  pausedBySensorUno = false;
  resetHomeMarkerState();
  resetMarkerCandidateState();
  headingHomebound = nextHomebound;
  motorStatus = STATUS_RUNNING;

  if (startNewDeadTime) {
    stopMotors();
    directionChangeDeadTimeActive = true;
    directionChangeDeadTimeStartedAt = millis();
    Serial.println(F("[MOTOR] direction changed -> all RELEASE for 120ms"));
    return;
  }
  if (preserveExistingDeadTime) {
    // 같은 방향의 새 순번이 도착해도 진행 중인 출력 해제 시간을 줄이지 않는다.
    stopMotors();
    return;
  }

  directionChangeDeadTimeActive = false;
  directionChangeDeadTimeStartedAt = 0;
  if (headingHomebound) {
    driveHomebound();
    Serial.println(F("[I2C] RETURN -> four motors BACKWARD / HOMEBOUND"));
  } else {
    driveOutbound();
    Serial.println(F("[I2C] OUTBOUND -> four motors FORWARD"));
  }
}

bool serviceDirectionChangeDeadTime() {
  if (!directionChangeDeadTimeActive) return false;

  // 대기 시간 내내 네 채널을 모두 RELEASE로 유지한다. delay()를 쓰지 않으므로
  // 이 동안에도 I2C ACK, KEEPALIVE, STOP, watchdog 처리는 계속 동작한다.
  stopMotors();
  if (millis() - directionChangeDeadTimeStartedAt <
      DIRECTION_CHANGE_DEAD_TIME_MS) {
    return true;
  }

  directionChangeDeadTimeActive = false;
  directionChangeDeadTimeStartedAt = 0;
  Serial.println(F("[MOTOR] direction dead-time complete"));
  return false;
}

void followLine() {
  if (BENCH_RFID_ONLY_MODE) {
    driveForCurrentHeading();
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
      driveOutbound();
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
      driveOutbound();
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
        // RETURN(0x12)으로 후진 중 HOME의 넓은 검은 마커를 확인한 경우다.
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
      driveForCurrentHeading();
      motorStatus = STATUS_RUNNING;
    } else if (leftBlack && !rightBlack) {
      correctTowardLeft();
      motorStatus = STATUS_RUNNING;
    } else {  // left background / right black
      correctTowardRight();
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
  Serial.print(F(" directionDeadTime="));
  Serial.print(directionChangeDeadTimeActive ? 1 : 0);
  Serial.print(F(" bothHighCandidate="));
  Serial.print(bothHighCandidate ? 1 : 0);
  Serial.print(F(" bothHighMs="));
  Serial.println(bothHighCandidate ? millis() - bothHighStartedAt : 0);
}

void setup() {
  Serial.begin(9600);

  pinMode(LEFT_IR_PIN, INPUT);
  pinMode(RIGHT_IR_PIN, INPUT);

  setCruiseSpeeds();
  calibrated = false;
  protocolValidated = false;
  headingHomebound = false;
  enterSafeStop(STATUS_PROTOCOL_REQUIRED);

  Wire.begin(I2C_ADDRESS);
  Wire.onReceive(receiveI2cCommand);
  Wire.onRequest(sendI2cStatus);

  Serial.println(F("[BOOT] MotorUno I2C slave started"));
  Serial.println(F("[BOOT] address=0x08 SDA=A4 SCL=A5"));
  Serial.println(F("[BOOT] IR=D9/D10, rear HC-SR04 is on SensorUno"));
  Serial.println(F("[BOOT] 4WD: M1/M3=LEFT, M2/M4=RIGHT"));
  Serial.print(F("[BOOT] PWM existing M1/M2="));
  Serial.print(EXISTING_AXLE_SPEED);
  Serial.print(F(" N20 M3/M4="));
  Serial.println(N20_REAR_AXLE_SPEED);
  Serial.print(F("[BOOT] direction change dead-time="));
  Serial.print(DIRECTION_CHANGE_DEAD_TIME_MS);
  Serial.println(F("ms"));
  Serial.println(F("[BOOT] protocol v2: 7=SYNC, 6=HOME, 1=FWD, 2=REVERSE"));
  Serial.println(F("[BOOT] RETURN uses straight reverse; no 180-degree turn"));
  Serial.println(F("[BOOT] ZONE arrival: SensorUno RFID -> STOP command"));
  Serial.println(F("[BOOT] HOME arrival: return-direction HIGH/HIGH marker"));
  Serial.println(F("[BOOT] protocol sync then HOME_SYNC required before motion"));
  if (BENCH_RFID_ONLY_MODE) {
    Serial.println(F("[BOOT] BENCH RFID-ONLY: line/marker bypassed"));
  }
  Serial.println(F("[BOOT] status 8=PROTOCOL_REQUIRED, 7=CALIBRATION_REQUIRED"));
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
  if (serviceDirectionChangeDeadTime()) return;

  followLine();
}
