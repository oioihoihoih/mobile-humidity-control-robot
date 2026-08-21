# 4WD 직선 전·후진 구조 전달 메모

## 이번 변경의 기준

- MotorUno의 M1·M3는 왼쪽, M2·M4는 오른쪽입니다.
- 기존 모터는 M1/M2, 새 N20 1:298 모터는 M3/M4에 연결합니다.
- 출동 명령 `0x11`은 네 바퀴 전진, 복귀 명령 `0x12`는 네 바퀴 후진입니다. 구형 이동값 `1/2`는 사용하지 않습니다.
- 제자리 180도 회전은 사용하지 않습니다.
- 주행 중 방향을 바꿀 때 네 모터를 120ms RELEASE한 뒤 반대 방향으로 구동합니다.
- 라인 보정은 한쪽 바퀴쌍을 역회전시키지 않고 PWM을 80%로만 낮춥니다.
- ZONE2/ZONE99는 주행 중 RFID로 확인하고 즉시 정지합니다.
- 방향 전환 직후에는 850ms와 `카드 없음` 1회 확인이 끝나기 전 RFID를 도착으로 인정하지 않습니다.
- HC-SR04 본체는 N20 쪽 후방을 보고 MotorUno ECHO A0/TRIG A1에 연결합니다.
- 후진 중 유효 거리 15cm 미만이면 PAUSE, 18cm 이상이 3회 연속이면 RESUME합니다.
- 후진 중 STUCK_HIGH/근거리는 MotorUno가 로컬 PAUSE합니다. NO_ECHO·OUT_OF_RANGE는 새 정지를 만들지 않는 진단값이며 이미 걸린 래치를 풀지 않습니다.
- SensorUno·MotorUno·ActuatorUno의 세 운영 스케치는 같은 커밋으로 모두 업로드합니다. command 7/status 8 exact ACK가 없거나 구형 이동값 `1/2`가 들어오면 HOME 보정과 이동이 잠깁니다.

## 보드별 업로드 파일

1. SensorUno: `firmware/uno_robot_esp01_rfid_relay/uno_robot_esp01_rfid_relay.ino`
2. MotorUno: `firmware/uno_line_tracker_motor_controller/uno_line_tracker_motor_controller.ino`
3. ActuatorUno: `firmware/uno_humidity_module_controller/uno_humidity_module_controller.ino`

`robot_network_config.h`는 Wi-Fi 암호가 들어가는 로컬 파일이므로 배포 압축을 만들 때 제외합니다.
`robot_network_config.example.h`를 복사해 SSID, 암호, 서버 주소와 RFID UID를 채운 뒤 SensorUno를 빌드하십시오.

## 최종 핀·포트 계약

### SensorUno

- D2/D3: 현재 사용하지 않음
- DHT22: D4
- ESP-01: Uno TX D5 → ESP RX(분압 필수), ESP TX → Uno RX D6
- RC522 소프트 SPI: SDA/SS D8, SCK D9, MOSI D10, MISO D11, RST D12
- 3-Uno 하드웨어 I2C: SDA A4, SCL A5

### MotorUno

- AFMotor M1: 기존 왼쪽
- AFMotor M2: 기존 오른쪽
- AFMotor M3: N20 후방 왼쪽
- AFMotor M4: N20 후방 오른쪽
- 라인 센서: 왼쪽 D9, 오른쪽 D10
- 후방 HC-SR04: ECHO A0(PCINT8), TRIG A1
- SensorUno와 I2C: A4/A5, slave `0x08`

### ActuatorUno

- 가습 릴레이 A0, 펠티어 릴레이 A1, 팬 릴레이 D7(active-low)
- SensorUno와 I2C: A4/A5, slave `0x09`
- LCD1602 소프트 I2C: SDA D5, SCL D4
- DHT22를 직접 읽지 않고 SensorUno의 10바이트 telemetry로 전달된 온·습도를 LCD에 표시

## 전원과 첫 시험

- 바퀴를 바닥에서 띄운 상태에서만 첫 시험을 시작합니다.
- 4×AA 모터 전원은 모터 실드 `EXT_PWR`에 연결합니다.
- MotorUno 논리 전원은 USB나 별도의 안정된 5V로 공급하고 GND만 공통으로 묶습니다.
- 모터 전류를 Uno 5V/VIN이나 브레드보드로 흘리지 않습니다.
- M1/M3와 M2/M4가 실제 바닥 기준 같은 방향인지 각 채널을 짧게 확인합니다. 반대면 모터 선을 바꾸거나 해당 채널 방향 상수를 수정합니다.
- 현재 기존축과 N20축 PWM은 모두 255인 시작값입니다. 빠른 축만 낮춰 직진을 맞춥니다.
- 4개 모터 동시 기동 때 전압 강하, L293D 발열, 모터 실속을 확인한 뒤에만 바닥 시험을 합니다.

## 검증 결과 기록

릴리스 커밋에서 `python scripts/check.py`와 세 운영 스케치의 Arduino CLI·CI
컴파일을 다시 실행하고, 그 출력의 테스트 개수와 flash/SRAM을 인계 기록에
첨부합니다. 현재 로컬 flash/SRAM은 SensorUno `27,586B / 1,428B`, MotorUno
`11,346B / 464B`, ActuatorUno `12,534B / 532B`이며 SensorUno CI budget은
`29,000B / 1,500B`입니다.

자동 검사와 컴파일 결과도 혼합 모터 4륜의 실제 방향, 선속도, 정지거리,
RFID 판독 거리와 전원 여유를 증명하지 않습니다. 체크인 SimulIDE 프록시는
이전 센서 배치를 보존하므로 이번 핀 재배치의 검증 근거로 사용하지 않습니다.
