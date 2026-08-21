#include <Arduino.h>

// 선택한 SensorUno의 I2C 하드웨어를 단독 검사하는 동안 버스를 해제하는 코드.
// 상태 릴레이 D5는 OFF, ESP/RC522/센서는 건드리지 않는다.
constexpr bool RELAY_ACTIVE_LOW = true;

void setup() {
  pinMode(5, OUTPUT);
  digitalWrite(5, RELAY_ACTIVE_LOW ? HIGH : LOW);
  pinMode(A4, INPUT);
  pinMode(A5, INPUT);
  digitalWrite(A4, LOW);
  digitalWrite(A5, LOW);
  Serial.begin(9600);
  Serial.println(F("[BOOT] SensorUno I2C bus released; D5 relay OFF"));
}

void loop() {}
