#include <Arduino.h>
#include <SoftwareSerial.h>

#include "gateway_network_config.h"

// [UNO R3(1): PC 서버 게이트웨이]
// 이 보드는 Python 서버를 실행하는 보드가 아니다.
// USB 시리얼로 PC에 로그를 보여 주고, ESP-01로 서버에 주기적인 생존 신호를 보낸다.
//
// 배선
// ESP-01 TX  -> UNO D6 (SoftwareSerial RX)
// ESP-01 RX  <- UNO D7 (SoftwareSerial TX, 반드시 5V->3.3V 분압)
// ESP-01 VCC/EN -> 안정적인 3.3V 전원, ESP-01 GND -> UNO GND
constexpr byte ESP_RX_PIN = 6;
constexpr byte ESP_TX_PIN = 7;

const char WIFI_SSID[] = GATEWAY_WIFI_SSID;
const char WIFI_PASSWORD[] = GATEWAY_WIFI_PASSWORD;
const char SERVER_HOST[] = GATEWAY_SERVER_HOST;
constexpr uint16_t SERVER_PORT = GATEWAY_SERVER_PORT;
constexpr unsigned long HEARTBEAT_INTERVAL_MS = 10000;

SoftwareSerial esp8266(ESP_RX_PIN, ESP_TX_PIN);
char responseBuffer[260];
char commandBuffer[100];
char requestBuffer[150];
bool wifiReady = false;
unsigned long lastHeartbeatAt = 0;

void clearEspInput() {
  while (esp8266.available()) esp8266.read();
}

bool waitFor(const char* expected, unsigned long timeoutMs,
             const char* alternative = nullptr) {
  uint16_t index = 0;
  responseBuffer[0] = '\0';
  const unsigned long startedAt = millis();
  while (millis() - startedAt < timeoutMs) {
    while (esp8266.available()) {
      const char c = static_cast<char>(esp8266.read());
      if (index < sizeof(responseBuffer) - 1) {
        responseBuffer[index++] = c;
        responseBuffer[index] = '\0';
      }
      if (strstr(responseBuffer, expected) ||
          (alternative && strstr(responseBuffer, alternative)) ||
          strstr(responseBuffer, "no change")) return true;
      if (strstr(responseBuffer, "ERROR") || strstr(responseBuffer, "FAIL")) return false;
    }
  }
  return false;
}

bool sendAt(const char* command, const char* expected, unsigned long timeoutMs,
            const char* alternative = nullptr) {
  clearEspInput();
  Serial.print(F("[ESP TX] "));
  if (!strncmp(command, "AT+CWJAP=", 9)) {
    Serial.print(F("AT+CWJAP=\""));
    Serial.print(WIFI_SSID);
    Serial.println(F("\",\"********\""));
  } else {
    Serial.println(command);
  }
  esp8266.print(command);
  esp8266.print(F("\r\n"));
  const bool ok = waitFor(expected, timeoutMs, alternative);
  Serial.print(F("[ESP RX] "));
  Serial.println(responseBuffer[0] ? responseBuffer : "(no response)");
  return ok;
}

bool connectWifi() {
  Serial.println(F("[GATEWAY] Wi-Fi connection start"));
  // AT+RST를 반복하면 일부 구형 ESP AT 펌웨어가 busy p... 재부팅 루프에 빠진다.
  // 전원 투입 후 ESP가 스스로 준비될 때까지 AT만 재시도한다.
  bool atReady = false;
  for (byte attempt = 1; attempt <= 6 && !atReady; ++attempt) {
    Serial.print(F("[GATEWAY] AT ready check "));
    Serial.print(attempt);
    Serial.println(F("/6"));
    atReady = sendAt("AT", "OK", 3000);
    if (!atReady) delay(2500);
  }
  if (!atReady) return false;
  sendAt("ATE0", "OK", 2000);
  // Python 서버가 재시작되면 ESP 쪽에 이전 TCP 링크가 남을 수 있다.
  // 기존 링크가 없어 ERROR가 나오는 경우는 무시하고 단일 연결 모드로 진행한다.
  sendAt("AT+CIPCLOSE", "OK", 3000, "Unlink");
  if (!sendAt("AT+CIPMUX=0", "OK", 3000)) return false;
  if (!sendAt("AT+CWMODE=1", "OK", 3000)) return false;

  snprintf_P(commandBuffer, sizeof(commandBuffer), PSTR("AT+CWJAP=\"%s\",\"%s\""),
             WIFI_SSID, WIFI_PASSWORD);
  if (!sendAt(commandBuffer, "OK", 30000)) return false;
  sendAt("AT+CIFSR", "OK", 4000);
  Serial.println(F("[GATEWAY] WIFI OK"));
  return true;
}

bool readHttp200() {
  uint16_t index = 0;
  unsigned long lastDataAt = millis();
  const unsigned long startedAt = millis();
  bool httpStarted = false;
  responseBuffer[0] = '\0';
  while (millis() - startedAt < 7000) {
    while (esp8266.available()) {
      const char c = static_cast<char>(esp8266.read());
      lastDataAt = millis();
      if (index < sizeof(responseBuffer) - 1) {
        responseBuffer[index++] = c;
        responseBuffer[index] = '\0';
      }
      if (strstr(responseBuffer, "HTTP/1.")) httpStarted = true;
    }
    if (httpStarted && millis() - lastDataAt > 500) break;
  }
  return strstr(responseBuffer, "HTTP/1.0 200") ||
         strstr(responseBuffer, "HTTP/1.1 200");
}

bool sendHeartbeat() {
  snprintf_P(commandBuffer, sizeof(commandBuffer),
             PSTR("AT+CIPSTART=\"TCP\",\"%s\",%u"), SERVER_HOST, SERVER_PORT);
  if (!sendAt(commandBuffer, "CONNECT", 10000, "Linked")) return false;

  snprintf_P(requestBuffer, sizeof(requestBuffer),
             PSTR("GET /api/gateway/heartbeat HTTP/1.1\r\nHost: %s:%u\r\nConnection: close\r\n\r\n"),
             SERVER_HOST, SERVER_PORT);
  snprintf_P(commandBuffer, sizeof(commandBuffer), PSTR("AT+CIPSEND=%u"),
             static_cast<unsigned int>(strlen(requestBuffer)));
  if (!sendAt(commandBuffer, ">", 5000)) return false;

  Serial.println(F("[HTTP TX] GET /api/gateway/heartbeat"));
  esp8266.print(requestBuffer);
  const bool ok = readHttp200();
  Serial.println(F("[HTTP RX]"));
  Serial.println(responseBuffer[0] ? responseBuffer : "(no response)");
  Serial.println(ok ? F("[GATEWAY] HEARTBEAT OK") : F("[GATEWAY] HEARTBEAT ERROR"));
  return ok;
}

void setup() {
  Serial.begin(9600);
  esp8266.begin(9600);
  Serial.println();
  Serial.println(F("[BOOT] UNO SERVER GATEWAY START"));
  Serial.println(F("[BOOT] ESP-01 TX->D6, RX<-D7(divider)"));
  wifiReady = connectWifi();
  Serial.println(wifiReady ? F("[BOOT] GATEWAY READY") : F("[BOOT] WIFI ERROR"));
  lastHeartbeatAt = millis() - HEARTBEAT_INTERVAL_MS;
}

void loop() {
  if (millis() - lastHeartbeatAt < HEARTBEAT_INTERVAL_MS) return;
  lastHeartbeatAt = millis();

  if (!wifiReady) {
    Serial.println(F("[GATEWAY] reconnecting Wi-Fi"));
    wifiReady = connectWifi();
  }
  if (wifiReady && !sendHeartbeat()) {
    wifiReady = false;
    Serial.println(F("[GATEWAY] next cycle will reconnect"));
  }
}
