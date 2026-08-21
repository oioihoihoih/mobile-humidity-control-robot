// [ZONE2 고정 센서 노드]
// DHT11 -> ESP-01(ESP8266) -> Wi-Fi -> PC 서버 /api/readings
//
// Uno는 사용하지 않는다. CP2102는 ESP-01에 프로그램을 올리고
// 시리얼 로그를 확인하는 USB-UART 변환기 역할만 한다.

#include <ESP8266WiFi.h>
#include <ESP8266HTTPClient.h>
#include <WiFiClient.h>
#include <DHT.h>

// Wi-Fi 정보와 서버 주소는 secrets.h에 별도로 작성한다.
#include "secrets.h"

// ESP-01에서 DHT11 DATA를 연결할 핀이다.
// GPIO2는 ESP-01의 두 번째 GPIO이며, 부팅 시 HIGH 상태가 유지되어야 한다.
constexpr uint8_t DHT_PIN = 2;
constexpr uint8_t DHT_TYPE = DHT11;
constexpr unsigned long SEND_INTERVAL_MS = 15000;

DHT dht(DHT_PIN, DHT_TYPE);
unsigned long lastSendAt = 0;

// Wi-Fi가 끊어졌을 때 다시 접속한다.
bool connectWifi() {
  if (WiFi.status() == WL_CONNECTED) return true;

  WiFi.mode(WIFI_STA);  // ESP-01을 공유기에 접속하는 Station 모드로 설정
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  Serial.print(F("Wi-Fi connecting"));

  // 최대 15초 동안 0.5초 간격으로 연결 완료를 확인한다.
  for (byte attempt = 0; attempt < 30; ++attempt) {
    if (WiFi.status() == WL_CONNECTED) {
      Serial.println();
      Serial.print(F("Wi-Fi connected. IP: "));
      Serial.println(WiFi.localIP());
      return true;
    }
    delay(500);
    Serial.print('.');
  }

  Serial.println(F("\nWi-Fi connection failed."));
  return false;
}

// 현재 온도/습도 값을 PC 서버의 ZONE2로 전송한다.
bool postReading(float temperature, float humidity) {
  if (WiFi.status() != WL_CONNECTED) return false;

  // PC 서버가 요구하는 JSON 형식이다.
  const String body = String(F("{\"zone_id\":\"")) + ZONE_ID +
                      F("\",\"temperature\":") + String(temperature, 1) +
                      F(",\"humidity\":") + String(humidity, 1) + F("}");

  WiFiClient client;
  HTTPClient http;
  http.setTimeout(5000);

  // secrets.h의 SERVER_URL 예: http://192.0.2.10:8000/api/readings
  if (!http.begin(client, SERVER_URL)) {
    Serial.println(F("HTTP client start failed."));
    return false;
  }

  http.addHeader(F("Content-Type"), F("application/json"));
  const int statusCode = http.POST(body);
  const bool accepted = statusCode == HTTP_CODE_OK;

  Serial.print(F("POST -> "));
  Serial.print(statusCode);
  if (statusCode > 0) {
    Serial.print(F(" : "));
    Serial.println(http.getString());
  } else {
    Serial.println(F(" (server connection failed)"));
  }

  http.end();  // 이번 HTTP 연결을 닫아 메모리와 소켓을 정리한다.
  return accepted;
}

void setup() {
  // 프로그램 실행 후 CP2102 시리얼 모니터를 115200bps로 연다.
  Serial.begin(115200);
  dht.begin();
  delay(1000);

  Serial.println(F("ZONE2 ESP-01 sensor node starting"));
  connectWifi();
}

void loop() {
  // DHT11은 빠르게 연속 측정할 필요가 없으므로 15초마다만 실행한다.
  if (millis() - lastSendAt < SEND_INTERVAL_MS) return;
  lastSendAt = millis();

  const float humidity = dht.readHumidity();
  const float temperature = dht.readTemperature();
  if (isnan(humidity) || isnan(temperature)) {
    Serial.println(F("DHT11 read failed."));
    return;
  }

  Serial.print(F("DHT11: "));
  Serial.print(temperature, 1);
  Serial.print(F(" C, "));
  Serial.print(humidity, 1);
  Serial.println(F(" %RH"));

  if (connectWifi() && postReading(temperature, humidity)) {
    Serial.println(F("ZONE2 reading accepted by PC server."));
  } else {
    Serial.println(F("ZONE2 POST failed; it will retry next cycle."));
  }
}
