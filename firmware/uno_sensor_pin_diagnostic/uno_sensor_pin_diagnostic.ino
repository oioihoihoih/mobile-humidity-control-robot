#include <Arduino.h>
#include <DHT.h>

// LEGACY 벤치 전용: DHT22/HC-SR04를 한 Uno의 D4/D2/D3에 임시로 꽂아
// 센서 자체만 확인하던 스케치다. 최신 3-Uno 운영 배선에서는 DHT22가
// ActuatorUno D2, HC-SR04가 MotorUno D2/A1에 있으므로 production 대신
// 이 파일을 업로드하거나 이 핀맵을 운영 배선으로 사용하지 않는다.

constexpr byte DHT_PIN = 4;
constexpr byte ECHO_PIN = 2;
constexpr byte TRIG_PIN = 3;
DHT dht22(DHT_PIN, DHT22);
unsigned long sampleNumber = 0;

void printDht(const __FlashStringHelper* label, DHT& sensor) {
  const float humidity = sensor.readHumidity(true);
  const float temperature = sensor.readTemperature(false, true);
  Serial.print(label);
  Serial.print(F(" T="));
  if (isnan(temperature)) Serial.print(F("ERR"));
  else Serial.print(temperature, 1);
  Serial.print(F(" H="));
  if (isnan(humidity)) Serial.println(F("ERR"));
  else {
    Serial.print(humidity, 1);
    Serial.println('%');
  }
}

void printDistance() {
  ++sampleNumber;
  Serial.print(F("[HC-SR04 #"));
  Serial.print(sampleNumber);
  Serial.print(F("] "));

  // 정상 HC-SR04는 TRIG 전 ECHO가 LOW이다. pulseIn()만 호출하면 LOW 고정과
  // HIGH 고정이 모두 0으로 반환되므로 먼저 유휴 레벨을 따로 검사한다.
  const bool idleHighFirst = digitalRead(ECHO_PIN) == HIGH;
  if (idleHighFirst) {
    delayMicroseconds(200);
    if (digitalRead(ECHO_PIN) == HIGH) {
      Serial.println(F("FAULT=ECHO_STUCK_HIGH_BEFORE_TRIGGER pulse_us=0"));
      Serial.println(F("  expected ECHO=LOW; check ECHO/TRIG order, common GND, VCC, module"));
      return;
    }
    Serial.print(F("idle_transient=HIGH->LOW "));
  } else {
    Serial.print(F("idle=LOW "));
  }

  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);
  const unsigned long duration = pulseIn(ECHO_PIN, HIGH, 30000UL);
  Serial.print(F(" pulse_us="));
  Serial.print(duration);
  if (duration) {
    const unsigned long distanceCm = duration * 17UL / 1000UL;
    if (distanceCm < 2 || distanceCm > 400) {
      Serial.print(F(" FAULT=OUT_OF_RANGE distance_cm="));
      Serial.println(distanceCm);
    } else {
      Serial.print(F(" result=VALID distance_cm="));
      Serial.println(distanceCm);
    }
  } else if (digitalRead(ECHO_PIN) == HIGH) {
    Serial.println(F(" FAULT=ECHO_STUCK_HIGH_AFTER_TRIGGER"));
  } else {
    Serial.println(F(" FAULT=NO_ECHO_RISE (ECHO stayed LOW)"));
  }
}

void setup() {
  Serial.begin(9600);
  pinMode(ECHO_PIN, INPUT);
  pinMode(TRIG_PIN, OUTPUT);
  digitalWrite(TRIG_PIN, LOW);
  dht22.begin();
  Serial.println();
  Serial.println(F("[LEGACY BENCH ONLY] not the distributed production pin map"));
  Serial.println(F("=== D4 DHT / D2-D3 HC-SR04 DIAGNOSTIC ==="));
  Serial.println(F("[WIRING] HC-SR04 ECHO->D2 TRIG->D3; all grounds must be common"));
  Serial.println(F("[EXPECT] ECHO must be LOW before each trigger"));
  Serial.print(F("[IDLE] DHT D4="));
  Serial.print(digitalRead(DHT_PIN));
  Serial.print(F(" ECHO D2="));
  Serial.println(digitalRead(ECHO_PIN));
  delay(2500);
}

void loop() {
  printDht(F("[DHT22 mode]"), dht22);
  printDistance();
  delay(2500);
}
