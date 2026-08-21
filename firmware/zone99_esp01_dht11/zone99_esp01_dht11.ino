#include <ESP8266HTTPClient.h>
#include <ESP8266WiFi.h>
#include <WiFiClient.h>
#include <DHT.h>

// CP2102는 코드 업로드와 로그 확인용이다. 실행 중 서버 통신은 ESP-01 Wi-Fi가 담당한다.
#include "secrets.h"

constexpr uint8_t DHT_PIN = 2;  // ESP-01 GPIO2
constexpr uint8_t DHT_TYPE = DHT_SENSOR_TYPE;
constexpr unsigned long SEND_INTERVAL_MS = 5000;
constexpr unsigned long WIFI_RETRY_INTERVAL_MS = 10000;

DHT dht(DHT_PIN, DHT_TYPE);
unsigned long lastSendAt = 0;
unsigned long lastWifiAttemptAt = 0;

void beginWifi() {
  lastWifiAttemptAt = millis();
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  Serial.print(F("[WIFI] connecting to "));
  Serial.println(WIFI_SSID);
}

bool postReading(float temperature, float humidity) {
  if (WiFi.status() != WL_CONNECTED) return false;

  const String body = String(F("{\"zone_id\":\"")) + ZONE_ID +
                      F("\",\"temperature\":") +
                      String(temperature, 1) + F(",\"humidity\":") +
                      String(humidity, 1) + F("}");

  WiFiClient client;
  HTTPClient http;
  http.setTimeout(5000);
  if (!http.begin(client, SERVER_URL)) {
    Serial.println(F("[HTTP] begin failed"));
    return false;
  }

  http.addHeader(F("Content-Type"), F("application/json"));
  const int status = http.POST(body);
  Serial.print(F("[HTTP] POST status="));
  Serial.println(status);
  if (status > 0) {
    Serial.print(F("[HTTP] response="));
    Serial.println(http.getString());
  }
  http.end();
  return status == HTTP_CODE_OK;
}

void setup() {
  Serial.begin(115200);
  dht.begin();
  WiFi.persistent(false);
  WiFi.setAutoReconnect(true);
  delay(1000);

  Serial.print(F("[BOOT] zone="));
  Serial.print(ZONE_ID);
  Serial.print(F(", sensor="));
  Serial.println(DHT_TYPE == DHT22 ? F("DHT22") : F("DHT11"));
  beginWifi();
  // 부팅 직후 첫 측정값도 바로 전송하도록 주기를 당겨 둔다.
  lastSendAt = millis() - SEND_INTERVAL_MS;
}

void loop() {
  if (WiFi.status() != WL_CONNECTED) {
    if (millis() - lastWifiAttemptAt >= WIFI_RETRY_INTERVAL_MS) beginWifi();
    delay(20);
    return;
  }

  static bool addressPrinted = false;
  if (!addressPrinted) {
    addressPrinted = true;
    Serial.print(F("[WIFI] connected, IP="));
    Serial.println(WiFi.localIP());
  }

  if (millis() - lastSendAt < SEND_INTERVAL_MS) return;
  lastSendAt = millis();

  const float humidity = dht.readHumidity();
  const float temperature = dht.readTemperature();
  if (isnan(humidity) || isnan(temperature)) {
    Serial.println(F("[DHT11] read failed"));
    return;
  }

  Serial.print(F("[DHT11] T="));
  Serial.print(temperature, 1);
  Serial.print(F("C H="));
  Serial.print(humidity, 1);
  Serial.println(F("%"));

  if (!postReading(temperature, humidity)) {
    Serial.println(F("[HTTP] send failed; next cycle will retry"));
  }
}
