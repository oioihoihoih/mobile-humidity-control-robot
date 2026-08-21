# ZONE2: ESP-01 + DHT11 + CP2102

이 스케치는 ESP-01이 DHT11을 직접 읽어 PC 관제 서버의 `ZONE2`로 전송하게 합니다. Uno는 사용하지 않습니다.

## 1. 배선

| 연결 대상 | ESP-01 | 설명 |
| --- | --- | --- |
| 3.3V 전원 | VCC, EN/CH_PD | 둘 다 3.3V HIGH. ESP-01 VCC에 5V를 연결하지 않습니다. |
| 공통 접지 | GND | CP2102, DHT11, ESP-01의 GND를 모두 연결합니다. |
| DHT11 VCC | 3.3V | DHT11 전원 |
| DHT11 GND | GND | DHT11 접지 |
| DHT11 DATA | GPIO2 | DATA는 3.3V 쪽으로 4.7k~10k 풀업이 필요합니다. 3핀 DHT11 모듈은 보통 내장되어 있습니다. |
| CP2102 TXD | ESP-01 RXD | 업로드/로그 통신. TX-RX는 교차 연결합니다. |
| CP2102 RXD | ESP-01 TXD | 업로드/로그 통신. |

CP2102 UART 신호는 반드시 3.3V 레벨이어야 합니다. CP2102의 3.3V 핀은 **UART 신호용 또는 최대 100mA급 전원일 수 있으므로**, ESP-01 Wi-Fi 구동 전원으로 바로 쓰지 마세요. CP2102 보드에 별도 3.3V 500mA 이상 레귤레이터가 명시되어 있거나, ESP-01 프로그래머 어댑터에 안정적인 3.3V 전원이 있을 때만 해당 전원을 사용합니다.

## 2. 업로드할 때

1. `secrets.example.h`를 `secrets.h`로 복사하고 Wi-Fi 비밀번호를 입력합니다.
2. ESP-01의 `GPIO0`을 GND에 연결합니다.
3. ESP-01을 리셋하거나 전원을 다시 연결합니다. 이 상태가 업로드 모드입니다.
4. Arduino IDE에서 보드 매니저로 **ESP8266 by ESP8266 Community**를 설치합니다.
5. 보드는 `Generic ESP8266 Module`, 포트는 CP2102의 COM 포트를 선택하고 업로드합니다.
6. 업로드가 끝나면 GPIO0-GND 연결을 제거하고 GPIO0을 HIGH 상태로 둔 뒤 ESP-01을 리셋합니다.
7. 시리얼 모니터를 `115200bps`로 열어 `ZONE2 reading accepted by PC server.`를 확인합니다.

GPIO0은 리셋 순간 LOW면 업로드 모드, HIGH면 저장된 프로그램 실행 모드입니다. GPIO2는 부팅 시 HIGH여야 하므로 DHT11 DATA 선이 LOW로 끌어내리지 않도록 풀업 저항을 사용합니다.

## 3. 결과 확인

관제 PC에서 `http://<SERVER_PC_IP>:8000`을 열면 `ZONE2` 행에 온도와 습도가 표시됩니다. PC IP가 바뀌면 `secrets.h`의 `SERVER_URL`도 바꿔 다시 업로드해야 합니다.
