// [레거시 프로토타입 — 현재 3-Uno 런타임에는 사용하지 않음]
// [프로젝트 흐름]
// DHT11(온습도 측정) -> Uno(값 처리) -> ESP-01(Wi-Fi) -> PC 서버(값 저장/관제)
//
// [배선]
//   DHT11 DATA -> Uno D2
//   ESP-01 TX  -> Uno D4  (Uno가 ESP의 응답을 받는 핀)
//   ESP-01 RX  <- Uno D3  (Uno가 ESP에 명령을 보내는 핀, 5V->3.3V 분압 권장)
//   Uno / DHT11 / ESP-01의 GND는 반드시 모두 연결한다.
//
// 이 스케치는 ESP-01 자체 웹서버를 만들지 않는다.
// ESP-01은 '클라이언트'로 동작하며, 15초마다 PC 서버 /api/readings에 값을 POST한다.

#include <DHT.h>
#include <SoftwareSerial.h>

// Wi-Fi 정보, PC 서버 IP, 구역 번호는 secrets.h에 따로 저장한다.
#include "secrets.h"

// 사용할 핀과 동작 주기 설정
constexpr byte DHT_PIN = 2;
constexpr byte DHT_TYPE = DHT11;
constexpr byte ESP_RX_PIN = 4;
constexpr byte ESP_TX_PIN = 3;
constexpr unsigned long ESP_BAUD = 9600;
constexpr unsigned long SEND_INTERVAL_MS = 15000;

// DHT11 제어 객체
DHT dht(DHT_PIN, DHT_TYPE);
// SoftwareSerial(받는 핀, 보내는 핀) 순서임에 주의한다.
SoftwareSerial esp8266(ESP_RX_PIN, ESP_TX_PIN);

bool networkReady = false;     // Wi-Fi 연결이 완료되었는지 저장
unsigned long lastSendAt = 0;  // 마지막 서버 전송 시각

// ESP-01에 남아 있는 이전 응답을 비운다.
// 이전 명령의 응답을 새 명령의 응답으로 잘못 판단하지 않기 위해 사용한다.
void clearEspInput() {
  while (esp8266.available()) {
    esp8266.read();
  }
}

// ESP-01 응답에서 expected 문자열(예: "OK", ">", "SEND OK")이 올 때까지 기다린다.
// alternativeExpected는 펌웨어마다 다른 성공 응답을 함께 허용하기 위한 값이다.
bool waitFor(const char* expected, unsigned long timeoutMs, bool printResponse = true,
             const char* alternativeExpected = nullptr) {
  String response;
  response.reserve(220);
  const unsigned long startedAt = millis();

  while (millis() - startedAt < timeoutMs) {
    while (esp8266.available()) {
      const char c = static_cast<char>(esp8266.read());
      if (response.length() < 300) response += c;

      // 설정값이 이미 같으면 일부 ESP-01 펌웨어는 OK 대신 "no change"만 보낸다.
      // 이것도 정상 성공으로 처리해야 Wi-Fi 설정이 중단되지 않는다.
      if (response.indexOf(expected) >= 0 ||
          (alternativeExpected && response.indexOf(alternativeExpected) >= 0) ||
          response.indexOf(F("no change")) >= 0) {
        if (printResponse) Serial.print(response);
        return true;
      }
      // ERROR 또는 FAIL이 들어오면 더 기다리지 않고 실패로 끝낸다.
      if (response.indexOf(F("ERROR")) >= 0 || response.indexOf(F("FAIL")) >= 0) {
        if (printResponse) Serial.print(response);
        return false;
      }
    }
  }

  // 시간 안에 응답이 오지 않은 경우도 실패로 처리한다.
  if (printResponse && response.length() > 0) Serial.print(response);
  return false;
}

// ESP-01에 AT 명령을 보내고, 정해진 성공 응답을 기다린다.
bool sendCommand(const String& command, const char* expected, unsigned long timeoutMs,
                 bool showCommand = true, bool showResponse = true,
                 const char* alternativeExpected = nullptr) {
  clearEspInput();
  if (showCommand) {
    Serial.print(F("ESP > "));
    Serial.println(command);
  }
  // 모든 AT 명령은 CR+LF(\r\n)로 끝내야 한다.
  esp8266.print(command);
  esp8266.print(F("\r\n"));

  const bool ok = waitFor(expected, timeoutMs, showResponse, alternativeExpected);
  if (!ok && showCommand) {
    Serial.print(F("ESP command failed: "));
    Serial.println(command);
  }
  return ok;
}

// 새 프로그램을 올렸어도 ESP-01은 이전 웹서버 설정을 유지할 수 있다.
// 먼저 리셋한 뒤 AT 명령에 실제로 응답하는지 확인한다.
bool resetAndPingEsp() {
  clearEspInput();
  Serial.println(F("ESP > AT+RST"));
  esp8266.print(F("AT+RST\r\n"));
  // 부팅 초반 글자는 74880bps라 깨져 보일 수 있지만,
  // 마지막 ready 메시지는 설정한 9600bps에서 수신된다.
  waitFor("ready", 9000, true);
  delay(500);

  // 일시적으로 응답이 늦는 경우를 대비해 AT 명령을 최대 4번 확인한다.
  for (byte attempt = 1; attempt <= 4; ++attempt) {
    if (sendCommand(F("AT"), "OK", 2500)) return true;
    Serial.print(F("ESP AT retry "));
    Serial.println(attempt);
    delay(1000);
  }
  return false;
}

// ESP-01을 PC 서버 전송에 맞는 Wi-Fi 클라이언트 상태로 만든다.
bool connectWifi() {
  if (!resetAndPingEsp()) {
    Serial.println(F("ESP-01 did not answer AT."));
    return false;
  }

  // 비밀번호를 보낼 때 시리얼 모니터에 그대로 출력되지 않도록 echo를 끈다.
  sendCommand(F("ATE0"), "OK", 2500, false, false);

  // 이전 스케치가 만든 ESP 웹서버가 있으면 종료한다.
  // 리셋 뒤에는 서버가 이미 없을 수 있으므로, 이 명령의 ERROR는 무시해도 된다.
  sendCommand(F("AT+CIPSERVER=0,1"), "OK", 2500, false, false);
  // PC 서버로 값을 보내려면 연결 ID가 필요 없는 단일 연결 모드(CIPMUX=0)를 쓴다.
  if (!sendCommand(F("AT+CIPMUX=0"), "OK", 3000)) return false;
  // ESP-01을 공유기에 접속하는 Station 모드로 설정한다.
  if (!sendCommand(F("AT+CWMODE=1"), "OK", 3000)) return false;

  // secrets.h의 Wi-Fi 이름/비밀번호로 공유기에 접속하는 AT 명령을 만든다.
  const String join = String(F("AT+CWJAP=\"")) + WIFI_SSID + F("\",\"") +
                      WIFI_PASSWORD + F("\"");
  Serial.println(F("ESP: joining Wi-Fi..."));
  clearEspInput();
  esp8266.print(join);
  esp8266.print(F("\r\n"));
  if (!waitFor("OK", 30000, false)) {
    Serial.println(F("Wi-Fi join failed."));
    return false;
  }

  Serial.println(F("Wi-Fi connected."));
  // 이 ESP-01의 구형 AI-Thinker 펌웨어는 CIPSTA? 대신 CIFSR로 IP를 확인한다.
  sendCommand(F("AT+CIFSR"), "OK", 4000);
  return true;
}

// 측정값 하나를 JSON 형식으로 만들어 PC 서버에 HTTP POST한다.
bool postReading(float temperature, float humidity) {
  // PC 서버가 받는 JSON 형식. ZONE1/ZONE2 중 어느 구역의 값인지도 포함한다.
  const String body = String(F("{\"zone_id\":\"")) + ZONE_ID +
                      F("\",\"temperature\":") + String(temperature, 1) +
                      F(",\"humidity\":") + String(humidity, 1) + F("}");
  // HTTP 요청 전체를 만든다. Content-Length는 JSON 글자 수와 정확히 같아야 한다.
  const String request = String(F("POST /api/readings HTTP/1.1\r\nHost: ")) +
                         SERVER_HOST + F(":") + String(SERVER_PORT) +
                         F("\r\nContent-Type: application/json\r\nConnection: close\r\nContent-Length: ") +
                         body.length() + F("\r\n\r\n") + body;

  // ESP-01에서 PC 서버의 8000번 포트로 TCP 연결을 연다.
  const String open = String(F("AT+CIPSTART=\"TCP\",\"")) + SERVER_HOST +
                      F("\",") + String(SERVER_PORT);
  // 일반 ESP 펌웨어는 CONNECT, 현재 AI-Thinker 펌웨어는 Linked를 반환한다.
  // 두 응답 모두 PC 서버와의 TCP 연결 성공을 뜻한다.
  if (!sendCommand(open, "CONNECT", 10000, true, true, "Linked")) return false;

  // CIPSEND에 HTTP 요청의 총 바이트 수를 알려주면 ESP-01이 '>'를 돌려준다.
  const String sendLength = String(F("AT+CIPSEND=")) + request.length();
  if (!sendCommand(sendLength, ">", 5000)) {
    sendCommand(F("AT+CIPCLOSE"), "OK", 3000, false, false);
    return false;
  }

  // '>'를 받은 뒤에만 실제 HTTP POST 요청을 전송한다.
  esp8266.print(request);
  if (!waitFor("SEND OK", 10000, true)) {
    Serial.println(F("POST payload was not accepted."));
    sendCommand(F("AT+CIPCLOSE"), "OK", 3000, false, false);
    return false;
  }

  // 이번 전송의 연결을 닫는다. 다음 15초 주기의 전송도 깨끗한 상태에서 시작된다.
  sendCommand(F("AT+CIPCLOSE"), "OK", 3000, false, false);
  return true;
}

void setup() {
  // PC 시리얼 모니터와 ESP-01 AT 통신 속도는 모두 9600bps로 맞춘다.
  Serial.begin(9600);
  esp8266.begin(ESP_BAUD);
  dht.begin();
  delay(1500);

  Serial.println(F("UNO DHT11 -> PC server telemetry"));
  Serial.print(F("Target: http://"));
  Serial.print(SERVER_HOST);
  Serial.print(':');
  Serial.println(SERVER_PORT);

  // 시작할 때 ESP 초기화와 Wi-Fi 연결을 한 번 수행한다.
  networkReady = connectWifi();
}

void loop() {
  // loop()는 매우 빠르게 반복되므로, 설정한 주기가 되기 전에는 아무 것도 하지 않는다.
  if (millis() - lastSendAt < SEND_INTERVAL_MS) return;
  lastSendAt = millis();

  // DHT11에서 현재 습도와 섭씨 온도를 읽는다.
  const float humidity = dht.readHumidity();
  const float temperature = dht.readTemperature();
  if (isnan(humidity) || isnan(temperature)) {
    Serial.println(F("DHT11 read failed"));
    return;
  }

  // 측정값을 시리얼 모니터에 먼저 표시한다.
  Serial.print(F("DHT11: "));
  Serial.print(temperature, 1);
  Serial.print(F(" C, "));
  Serial.print(humidity, 1);
  Serial.println(F(" %RH"));

  // 이전 전송에 실패했다면 다음 주기에 ESP를 리셋하고 Wi-Fi부터 다시 연결한다.
  if (!networkReady) networkReady = connectWifi();
  if (!networkReady) {
    Serial.println(F("Wi-Fi unavailable; POST skipped."));
    return;
  }

  // 온습도 값을 PC 서버에 전송한다.
  if (postReading(temperature, humidity)) {
    Serial.println(F("Reading accepted by PC server."));
  } else {
    // 실패 상태를 저장해 다음 주기에는 Wi-Fi 초기화부터 다시 하도록 만든다.
    Serial.println(F("POST failed; ESP will reinitialize next cycle."));
    networkReady = false;
  }
}
