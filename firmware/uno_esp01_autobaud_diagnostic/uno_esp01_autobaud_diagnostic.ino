#include <Arduino.h>
#include <SoftwareSerial.h>

// 현재 실물 배선: D5(UNO TX) -> ESP RX, D6(UNO RX) <- ESP TX.
// D5에서 ESP RX로 들어가는 신호는 3.3V 분압을 거쳐야 한다.
constexpr byte UNO_RX_PIN = 6;
constexpr byte UNO_TX_PIN = 5;
SoftwareSerial esp(UNO_RX_PIN, UNO_TX_PIN);

const unsigned long BAUDS[] = {9600, 19200, 38400, 57600, 115200};

enum BaudTestResult : byte {
  BAUD_NO_RESPONSE,
  BAUD_AT_OK,
  BAUD_AT_BUSY
};

void drainEsp() {
  while (esp.available()) esp.read();
}

// 이전 CIPSEND가 끝나지 않은 모듈에 새 명령을 연속해서 넣으면 busy 상태가
// 더 오래 유지될 수 있다. 지정 시간 동안 ESP에는 한 바이트도 쓰지 않고,
// 자발적으로 출력하는 완료/CLOSED/ready 메시지만 USB로 관찰한다.
void observeWithoutTransmit(unsigned long durationMs) {
  const unsigned long startedAt = millis();
  while (millis() - startedAt < durationMs) {
    while (esp.available()) Serial.write(esp.read());
  }
}

BaudTestResult testBaud(unsigned long baud) {
  esp.end();
  esp.begin(baud);
  esp.listen();
  delay(150);
  drainEsp();

  Serial.print(F("[TEST] baud="));
  Serial.println(baud);
  esp.print(F("AT\r\n"));

  char response[80];
  byte used = 0;
  const unsigned long startedAt = millis();
  while (millis() - startedAt < 1500) {
    while (esp.available()) {
      const char value = static_cast<char>(esp.read());
      if (used < sizeof(response) - 1) response[used++] = value;
    }
  }
  response[used] = '\0';

  Serial.print(F("[RX bytes] "));
  Serial.println(used);
  Serial.print(F("[RX text] "));
  for (byte index = 0; index < used; ++index) {
    const byte value = static_cast<byte>(response[index]);
    if (value >= 32 && value <= 126) Serial.write(value);
    else {
      Serial.print(F("<"));
      if (value < 0x10) Serial.print('0');
      Serial.print(value, HEX);
      Serial.print(F(">"));
    }
  }
  Serial.println();
  if (strstr(response, "OK")) return BAUD_AT_OK;
  if (strstr(response, "busy s") || strstr(response, "busy p")) {
    return BAUD_AT_BUSY;
  }
  return BAUD_NO_RESPONSE;
}

void printFirmwareVersion() {
  drainEsp();
  Serial.println(F("[GMR] firmware identification"));
  esp.print(F("AT+GMR\r\n"));
  const unsigned long startedAt = millis();
  while (millis() - startedAt < 2500) {
    while (esp.available()) Serial.write(esp.read());
  }
  Serial.println();
}

void setup() {
  Serial.begin(115200);
  delay(500);
  Serial.println();
  Serial.println(F("=== ESP-01 D6 RX / D5 TX AUTOBAUD ==="));
  Serial.print(F("[IDLE] ESP TX level on D6="));
  pinMode(UNO_RX_PIN, INPUT);
  Serial.println(digitalRead(UNO_RX_PIN));

  esp.begin(9600);
  esp.listen();
  Serial.println(F("[QUIET] 30 seconds with no ESP TX"));
  observeWithoutTransmit(30000UL);
  BaudTestResult result = testBaud(9600);
  if (result == BAUD_NO_RESPONSE) {
    Serial.println(F("[RECOVERY] escape, disable transparent mode, close TCP, reset"));
    esp.end();
    esp.begin(9600);
    esp.listen();
    // +++ 뒤에는 Espressif 지침대로 다음 AT 명령 전 1초 이상 기다린다.
    esp.print(F("+++"));
    delay(1200);
    esp.print(F("AT+CIPMODE=0\r\n"));
    delay(1500);
    drainEsp();
    esp.print(F("AT+CIPCLOSE\r\n"));
    delay(1500);
    drainEsp();
    esp.print(F("AT+RST\r\n"));
    delay(8000);
    drainEsp();
    result = testBaud(9600);
  }
  for (byte index = 1;
       result == BAUD_NO_RESPONSE &&
       index < sizeof(BAUDS) / sizeof(BAUDS[0]); ++index) {
    result = testBaud(BAUDS[index]);
    if (result == BAUD_AT_OK) {
      Serial.print(F("[RESULT] AT OK at "));
      Serial.println(BAUDS[index]);
      break;
    } else if (result == BAUD_AT_BUSY) {
      Serial.print(F("[RESULT] AT BUSY at "));
      Serial.println(BAUDS[index]);
      break;
    }
  }
  if (result == BAUD_AT_OK) {
    printFirmwareVersion();
  } else if (result == BAUD_AT_BUSY) {
    Serial.println(F("[RESULT] ESP AT parser is busy"));
    Serial.println(F("Power off ESP VCC/EN for 5 seconds; do not send more AT commands."));
  } else if (result == BAUD_NO_RESPONSE) {
    Serial.println(F("[RESULT] NO AT RESPONSE"));
    Serial.println(F("Check common GND, ESP VCC/EN, RX divider, and TX/RX wiring."));
  }
  Serial.println(F("[DONE] Restore the production SensorUno sketch after this test."));
}

void loop() {}
