# 구역별 ESP-01 + DHT11/DHT22 + CP2102

이 노드는 Uno 없이 ESP-01이 DHT11 또는 DHT22를 직접 읽어 PC 서버의 `POST /api/readings`로 전송합니다. CP2102는 업로드, USB 시리얼 로그와 컴퓨터 연결에 사용하며 실제 서버 통신은 ESP-01 Wi-Fi가 담당합니다.

## 배선

| ESP-01/CP2102 | 연결 |
| --- | --- |
| ESP-01 VCC | 안정적인 3.3V, 500mA 이상 |
| ESP-01 GND | 전원 GND 및 CP2102 GND |
| ESP-01 EN/CH_PD | 3.3V |
| ESP-01 GPIO2 | DHT DATA 및 10kΩ 풀업 |
| DHT11/22 VCC·GND | 3.3V·GND |
| CP2102 TX | ESP-01 RX |
| CP2102 RX | ESP-01 TX |

업로드할 때만 GPIO0을 GND로 연결하고 리셋합니다. 업로드 후 GPIO0을 분리하고 다시 리셋하면 정상 실행됩니다. CP2102의 3.3V 출력이 Wi-Fi 송신 전류를 감당하지 못하면 별도의 3.3V 레귤레이터를 사용하세요.

`secrets.example.h`를 참고해 로컬 `secrets.h`에 Wi-Fi, 서버 주소, `ZONE_ID`, `DHT_SENSOR_TYPE`을 설정합니다. 시리얼 모니터는 115200bps입니다. 예를 들어 ZONE2의 DHT22는 `ZONE_ID "ZONE2"`, `DHT_SENSOR_TYPE DHT22`로 설정하고, ZONE99의 DHT11은 각각 `ZONE99`, `DHT11`로 설정합니다.

CP2102의 USB 케이블을 계속 연결해도 데이터는 USB가 아니라 Wi-Fi로 서버에 전달됩니다. 다수 구역 노드는 각각 별도 CP2102/USB 포트를 사용할 수 있지만, 서버는 COM 포트가 아니라 `zone_id`가 포함된 HTTP 데이터로 구역을 구분합니다.
