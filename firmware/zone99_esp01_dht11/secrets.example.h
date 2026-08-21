#pragma once

#define WIFI_SSID "YOUR_WIFI_SSID"
#define WIFI_PASSWORD "YOUR_WIFI_PASSWORD"

// localhost가 아니라 서버 PC의 Wi-Fi 내부 IP를 사용한다.
#define SERVER_URL "http://192.0.2.10:8000/api/readings"

// 구역마다 이 두 값만 바꿔 각각 업로드한다.
#define ZONE_ID "ZONE99"
#define DHT_SENSOR_TYPE DHT11  // DHT22를 사용하면 DHT22로 변경
