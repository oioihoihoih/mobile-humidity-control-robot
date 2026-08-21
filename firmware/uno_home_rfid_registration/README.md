# HOME RFID UID 등록 예제

현재 SensorUno의 RC522 배선 그대로 HOME 카드의 UID를 읽는 독립 예제입니다.
모터, 릴레이, I2C, ESP-01과 EEPROM은 사용하지 않습니다.

## 배선

| RC522 | SensorUno |
| --- | --- |
| SDA/SS | D8 |
| SCK | D9 |
| MOSI | D10 |
| MISO | D11 |
| RST | D12 |
| 3.3V | 3.3V |
| GND | GND |

RC522의 `SDA` 표시는 I2C가 아니라 SPI의 `SS/CS`입니다. RC522에는 5V가 아닌
3.3V만 공급합니다.

## 사용 순서

1. 서버를 `ALL_STOP`으로 두고 바퀴를 띄우거나 모터 전원을 분리합니다.
2. 이 폴더의 스케치를 SensorUno(COM6)에 업로드합니다.
3. 시리얼 모니터를 `115200 baud`로 엽니다.
4. HOME으로 사용할 카드 한 장만 RC522에 댑니다.
5. `[HOME UID] AA BB CC DD` 값을 기록하고, 카드를 떼었다 다시 대서 같은 값인지 확인합니다.
6. 등록 카드를 차량의 RC522 리더에서 완전히 떼어 트랙의 물리 HOME 위치에
   고정합니다. 카드를 리더에 붙인 채 주행하면 ZONE2/ZONE99 태그를 읽을 수 없습니다.
7. 확인 후 production 스케치 `uno_robot_esp01_rfid_relay`를 SensorUno에 다시 업로드합니다.

출력되는 `ROBOT_HOME_UID` 줄을 Git에 무시되는 production
`robot_network_config.h`에 저장하면 복귀 중 HOME RFID를 보조 도착 수단으로
사용할 수 있습니다. 실제 UID는 공개 Git이나 문서에 넣지 않습니다.

HOME RFID는 기존 넓은 검은 HOME 마커와 `CALIBRATE_HOME`을 대체하지 않습니다.
보정은 계속 IR HOME 마커 위에서 수행하고, 복귀는 HOME RFID 또는 기존 마커 중
먼저 안전하게 확인된 쪽으로 완료합니다.

## 컴파일

```powershell
arduino-cli compile --fqbn arduino:avr:uno --warnings all firmware/uno_home_rfid_registration
```
