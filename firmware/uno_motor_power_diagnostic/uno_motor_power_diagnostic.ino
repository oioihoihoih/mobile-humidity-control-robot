// 4개 DC 모터의 전원·배선·회전 방향을 한 채널씩 확인하는 진단 스케치
//
// 현재 차체 배선:
//   M1 = 기존 앞쪽 왼쪽,  M2 = 기존 앞쪽 오른쪽
//   M3 = N20 뒤쪽 왼쪽,  M4 = N20 뒤쪽 오른쪽
//   M1/M3은 같은 측, M2/M4는 같은 측으로 함께 움직인다.
//
// 안전 규칙:
//   - 부팅 직후와 명령이 없을 때는 네 채널 모두 RELEASE
//   - 반드시 바퀴를 공중에 띄우고 먼저 A를 입력해 5초 시험 창을 연다.
//   - 다음 명령 하나를 실행하면 500ms 뒤 네 채널 모두 자동 RELEASE
//   - 1/2/3/4: 해당 포트만 전진
//   - L/R: 왼쪽(M1+M3)/오른쪽(M2+M4)만 전진
//   - F/B: 네 바퀴 전진/후진
//   - 0: 즉시 정지, ?: 현재 상태

#include "AFMotor_R4.h"

// 기존 모터와 1:298 N20은 같은 PWM에서도 실제 속도가 다를 수 있다.
// 아래 두 값을 따로 조절하되 첫 방향 시험은 짧게 255로 수행한다.
constexpr byte FRONT_MOTOR_TEST_SPEED = 255;
constexpr byte REAR_N20_TEST_SPEED = 255;
constexpr unsigned long TEST_DURATION_MS = 500;
constexpr unsigned long ARM_WINDOW_MS = 5000;

AF_DCMotor motorM1(1);
AF_DCMotor motorM2(2);
AF_DCMotor motorM3(3);
AF_DCMotor motorM4(4);

enum TestTarget : byte {
  TEST_NONE = 0,
  TEST_M1,
  TEST_M2,
  TEST_M3,
  TEST_M4,
  TEST_LEFT_PAIR,
  TEST_RIGHT_PAIR,
  TEST_ALL_FORWARD,
  TEST_ALL_REVERSE,
};

bool armed = false;
TestTarget activeTest = TEST_NONE;
unsigned long armedAt = 0;
unsigned long testStartedAt = 0;

void releaseAll(const __FlashStringHelper* reason) {
  motorM1.run(RELEASE);
  motorM2.run(RELEASE);
  motorM3.run(RELEASE);
  motorM4.run(RELEASE);
  activeTest = TEST_NONE;
  armed = false;
  Serial.print(F("[STOP] M1-M4 RELEASED: "));
  Serial.println(reason);
}

void printStatus() {
  Serial.print(F("[STATUS] armed="));
  Serial.print(armed ? 1 : 0);
  Serial.print(F(" active="));
  Serial.print(static_cast<byte>(activeTest));
  Serial.print(F(" front_pwm="));
  Serial.print(FRONT_MOTOR_TEST_SPEED);
  Serial.print(F(" rear_n20_pwm="));
  Serial.print(REAR_N20_TEST_SPEED);
  Serial.print(F(" duration_ms="));
  Serial.println(TEST_DURATION_MS);
}

void startTest(TestTarget target) {
  if (!armed || millis() - armedAt > ARM_WINDOW_MS) {
    releaseAll(F("not armed; send A first"));
    return;
  }

  motorM1.run(RELEASE);
  motorM2.run(RELEASE);
  motorM3.run(RELEASE);
  motorM4.run(RELEASE);

  if (target == TEST_M1 || target == TEST_LEFT_PAIR ||
      target == TEST_ALL_FORWARD) motorM1.run(FORWARD);
  if (target == TEST_M2 || target == TEST_RIGHT_PAIR ||
      target == TEST_ALL_FORWARD) motorM2.run(FORWARD);
  if (target == TEST_M3 || target == TEST_LEFT_PAIR ||
      target == TEST_ALL_FORWARD) motorM3.run(FORWARD);
  if (target == TEST_M4 || target == TEST_RIGHT_PAIR ||
      target == TEST_ALL_FORWARD) motorM4.run(FORWARD);

  if (target == TEST_ALL_REVERSE) {
    motorM1.run(BACKWARD);
    motorM2.run(BACKWARD);
    motorM3.run(BACKWARD);
    motorM4.run(BACKWARD);
  }

  activeTest = target;
  testStartedAt = millis();
  armed = false;
  Serial.print(F("[TEST] START target="));
  Serial.print(static_cast<byte>(target));
  Serial.print(F(" duration_ms="));
  Serial.println(TEST_DURATION_MS);
}

void handleSerial() {
  while (Serial.available()) {
    const char input = static_cast<char>(Serial.read());

    if (input == '0') {
      releaseAll(F("serial emergency stop"));
    } else if (input == '?') {
      printStatus();
    } else if ((input == 'A' || input == 'a') && activeTest == TEST_NONE) {
      armed = true;
      armedAt = millis();
      Serial.println(F("[ARMED] 1/2/3/4, L/R, F/B; 0=stop ?=status"));
    } else if (input == '1') {
      startTest(TEST_M1);
    } else if (input == '2') {
      startTest(TEST_M2);
    } else if (input == '3') {
      startTest(TEST_M3);
    } else if (input == '4') {
      startTest(TEST_M4);
    } else if (input == 'L' || input == 'l') {
      startTest(TEST_LEFT_PAIR);
    } else if (input == 'R' || input == 'r') {
      startTest(TEST_RIGHT_PAIR);
    } else if (input == 'F' || input == 'f') {
      startTest(TEST_ALL_FORWARD);
    } else if (input == 'B' || input == 'b') {
      startTest(TEST_ALL_REVERSE);
    }
  }
}

void setup() {
  motorM1.setSpeed(FRONT_MOTOR_TEST_SPEED);
  motorM2.setSpeed(FRONT_MOTOR_TEST_SPEED);
  motorM3.setSpeed(REAR_N20_TEST_SPEED);
  motorM4.setSpeed(REAR_N20_TEST_SPEED);
  motorM1.run(RELEASE);
  motorM2.run(RELEASE);
  motorM3.run(RELEASE);
  motorM4.run(RELEASE);

  Serial.begin(9600);
  Serial.println();
  Serial.println(F("[BOOT] M1-M4 motor power diagnostic"));
  Serial.println(F("[MAP] LEFT=M1+M3 RIGHT=M2+M4 / M3+M4=N20 1:298"));
  Serial.println(F("[SAFE] lift all wheels; outputs are RELEASED"));
  Serial.println(F("[COMMAND] A then 1/2/3/4/L/R/F/B; 0=stop ?=status"));
}

void loop() {
  handleSerial();

  if (activeTest != TEST_NONE &&
      millis() - testStartedAt >= TEST_DURATION_MS) {
    releaseAll(F("automatic 500ms cutoff"));
  }

  if (armed && millis() - armedAt > ARM_WINDOW_MS) {
    releaseAll(F("arm window expired"));
  }
}
