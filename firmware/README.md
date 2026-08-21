# Firmware

## 자동차용 Uno 3개

GitHub에 공개하는 운영 Uno 코드는 아래 세 개뿐입니다.

| 보드 | 스케치 | 역할 |
| --- | --- | --- |
| SensorUno | `uno_robot_esp01_rfid_relay/` | 서버 통신, RFID, 임무 조율 |
| MotorUno | `uno_line_tracker_motor_controller/` | 4모터, 라인 센서, 후방 초음파 |
| ActuatorUno | `uno_humidity_module_controller/` | 가습·제습 릴레이, 팬, LCD |

세 스케치는 항상 같은 커밋 버전으로 함께 업로드합니다. SensorUno를 빌드하기
전에는 `robot_network_config.example.h`를 복사해 Git에서 제외되는
`robot_network_config.h`를 만들고 현장 설정을 입력합니다.

## 고정 구역 센서

아래 코드는 Uno가 아니라 ESP-01용입니다.

- `zone2_esp01_direct/`
- `zone99_esp01_dht11/`

`robot/`과 `zone_sensor/`는 이전 ESP32 실험 자료이며 현재 자동차 펌웨어가
아닙니다. 그 밖의 Uno 진단·실험 스케치는 로컬에만 두고 GitHub에서는
추적하지 않습니다.

배선은 [`../docs/hardware.md`](../docs/hardware.md), 검증 절차는
[`../docs/testing.md`](../docs/testing.md)를 확인하세요.
