// LEGACY ESP32 PROTOTYPE — current fixed zones use the ESP-01 sketches.
// Flash this sketch to each fixed sensing-zone ESP32.
// Change ZONE_ID, WIFI_SSID, WIFI_PASSWORD, and SERVER_BASE_URL before upload.

#include <WiFi.h>
#include <HTTPClient.h>
#include <DHT.h>

const char* WIFI_SSID = "YOUR_WIFI_SSID";
const char* WIFI_PASSWORD = "YOUR_WIFI_PASSWORD";
const char* SERVER_BASE_URL = "http://192.0.2.10:8000";  // TEST-NET example; replace locally
const char* ZONE_ID = "ZONE1";                              // change second board to ZONE2

constexpr uint8_t DHT_PIN = 4;
constexpr uint8_t DHT_TYPE = DHT22;
constexpr unsigned long SEND_INTERVAL_MS = 5000;

DHT dht(DHT_PIN, DHT_TYPE);
unsigned long lastSendAt = 0;

void connectWifi() {
  if (WiFi.status() == WL_CONNECTED) return;

  Serial.printf("Connecting to %s", WIFI_SSID);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  for (int attempt = 0; attempt < 30 && WiFi.status() != WL_CONNECTED; ++attempt) {
    delay(500);
    Serial.print('.');
  }
  Serial.println();
  if (WiFi.status() == WL_CONNECTED) {
    Serial.print("IP: ");
    Serial.println(WiFi.localIP());
  }
}

void sendReading() {
  const float humidity = dht.readHumidity();
  const float temperature = dht.readTemperature();
  if (isnan(humidity) || isnan(temperature)) {
    Serial.println("DHT22 read failed");
    return;
  }
  if (WiFi.status() != WL_CONNECTED) return;

  HTTPClient http;
  const String endpoint = String(SERVER_BASE_URL) + "/api/readings";
  http.begin(endpoint);
  http.addHeader("Content-Type", "application/json");
  const String body = String("{\"zone_id\":\"") + ZONE_ID +
                      "\",\"temperature\":" + String(temperature, 1) +
                      ",\"humidity\":" + String(humidity, 1) + "}";
  const int status = http.POST(body);
  Serial.printf("POST %s -> %d: %s\n", endpoint.c_str(), status,
                status > 0 ? http.getString().c_str() : "connection failed");
  http.end();
}

void setup() {
  Serial.begin(115200);
  dht.begin();
  connectWifi();
}

void loop() {
  connectWifi();
  if (millis() - lastSendAt >= SEND_INTERVAL_MS) {
    lastSendAt = millis();
    sendReading();
  }
}
