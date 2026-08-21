# Firmware

## 자동차용 Uno 3개

GitHub에 공개하는 운영 Uno 코드는 아래 세 개뿐입니다.

| 보드 | 스케치 | 역할 |
| --- | --- | --- |
| SensorUno | `uno_robot_esp01_rfid_relay/` | 서버 통신, HOME·구역 RFID, 임무 조율 |
| MotorUno | `uno_line_tracker_motor_controller/` | 4모터, 라인 센서, 후방 초음파 |
| ActuatorUno | `uno_humidity_module_controller/` | 가습·제습 릴레이, 팬, LCD |

세 스케치는 항상 같은 커밋 버전으로 함께 업로드합니다. SensorUno를 빌드하기
전에는 `robot_network_config.example.h`를 복사해 Git에서 제외되는
`robot_network_config.h`를 만들고 현장 설정을 입력합니다.

## HOME RFID UID 등록 예제

`uno_home_rfid_registration/`은 현재 SensorUno의 RC522 배선
`SDA/SS=D8`, `SCK=D9`, `MOSI=D10`, `MISO=D11`, `RST=D12`를 그대로
사용해 HOME 카드 UID만 읽는 예제입니다. 모터·릴레이·ESP-01·I2C를 제어하는
네 번째 운영 펌웨어가 아닙니다.

서버를 `ALL_STOP`으로 두고 바퀴를 띄운 상태에서 예제를 SensorUno에 잠시
업로드해 UID를 두 번 확인합니다. 실제 값은 공개 문서나 소스에 쓰지 않고
Git에서 제외되는 `uno_robot_esp01_rfid_relay/robot_network_config.h`의
`ROBOT_HOME_UID`에만 넣습니다. 등록을 마치면
`uno_robot_esp01_rfid_relay` 운영 스케치를 SensorUno에 반드시 다시
업로드합니다. 예제 스케치가 올라간 상태에서는 자동차 운영 기능이 동작하지
않습니다.

운영 펌웨어는 등록된 HOME UID를 복귀 중 예상 HOME에서만 보조 도착 수단으로
인정합니다. 출발할 때 리더 아래에 남아 있는 HOME 카드는 무시하며, 넓은 검은
HOME 마커와 `CALIBRATE_HOME`은 계속 필수입니다. RFID를 놓쳐도 마커 기반
도착 판정은 fallback으로 유지됩니다. 자세한 순서는
[`uno_home_rfid_registration/README.md`](uno_home_rfid_registration/README.md)를
확인하세요.

## 고정 구역 센서

아래 코드는 Uno가 아니라 ESP-01용입니다.

- `zone2_esp01_direct/`
- `zone99_esp01_dht11/`

`robot/`과 `zone_sensor/`는 이전 ESP32 실험 자료이며 현재 자동차 펌웨어가
아닙니다. 그 밖의 Uno 진단·실험 스케치는 로컬에만 두고 GitHub에서는
추적하지 않습니다.

배선은 [`../docs/hardware.md`](../docs/hardware.md), 검증 절차는
[`../docs/testing.md`](../docs/testing.md)를 확인하세요.
