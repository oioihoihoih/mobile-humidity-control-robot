// 좌우 2개 구동 모터의 전원·배선 확인용 임시 진단 스케치
//
// 대상 배선:
//   M1/M2를 각각 왼쪽/오른쪽 모터로 사용한다.
//
// 안전 규칙:
//   - 부팅 직후와 명령이 없을 때는 항상 RELEASE
//   - 먼저 A를 입력해 5초 동안만 시험을 허용
//   - L/R/B 중 하나를 입력하면 500ms만 회전 후 자동 RELEASE
//   - 0은 즉시 정지, ?는 현재 상태 출력
//   - 한 번 시험하면 다시 A로 허용해야 함

#include "AFMotor_R4.h"

constexpr byte TEST_SPEED = 255;
constexpr unsigned long TEST_DURATION_MS = 500;
constexpr unsigned long ARM_WINDOW_MS = 5000;

AF_DCMotor motorLeft(1);
AF_DCMotor motorRight(2);

enum TestTarget : byte {
  TEST_NONE = 0,
  TEST_LEFT = 1,
  TEST_RIGHT = 2,
  TEST_BOTH = 3,
};

bool armed = false;
TestTarget activeTest = TEST_NONE;
unsigned long armedAt = 0;
unsigned long testStartedAt = 0;

void releaseAll(const __FlashStringHelper *reason) {
  motorLeft.run(RELEASE);
  motorRight.run(RELEASE);
  activeTest = TEST_NONE;
  armed = false;
  Serial.print(F("[STOP] motors RELEASED: "));
  Serial.println(reason);
}

void printStatus() {
  Serial.print(F("[STATUS] armed="));
  Serial.print(armed ? 1 : 0);
  Serial.print(F(" active="));
  Serial.print(static_cast<byte>(activeTest));
  Serial.print(F(" speed="));
  Serial.print(TEST_SPEED);
  Serial.print(F(" duration_ms="));
  Serial.println(TEST_DURATION_MS);
}

void startTest(TestTarget target) {
  if (!armed || millis() - armedAt > ARM_WINDOW_MS) {
    releaseAll(F("not armed; send A first"));
    return;
  }

  motorLeft.run(RELEASE);
  motorRight.run(RELEASE);
  if (target == TEST_LEFT || target == TEST_BOTH) motorLeft.run(FORWARD);
  if (target == TEST_RIGHT || target == TEST_BOTH) motorRight.run(FORWARD);

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
      Serial.println(F("[ARMED] L=M1 R=M2 B=M1+M2 0=stop"));
    } else if (input == 'L' || input == 'l') {
      startTest(TEST_LEFT);
    } else if (input == 'R' || input == 'r') {
      startTest(TEST_RIGHT);
    } else if (input == 'B' || input == 'b') {
      startTest(TEST_BOTH);
    }
  }
}

void setup() {
  motorLeft.setSpeed(TEST_SPEED);
  motorRight.setSpeed(TEST_SPEED);
  motorLeft.run(RELEASE);
  motorRight.run(RELEASE);

  Serial.begin(9600);
  Serial.println();
  Serial.println(F("[BOOT] Motor power diagnostic"));
  Serial.println(F("[SAFE] wheels must be lifted; outputs are RELEASED"));
  Serial.println(F("[COMMAND] A then L/R/B; 0=stop ?=status"));
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
