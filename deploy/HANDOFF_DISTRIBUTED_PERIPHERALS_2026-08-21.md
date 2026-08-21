# 3-Uno 주변기기 분배 전달 메모

## 변경 목적

SensorUno에 집중돼 있던 DHT22와 HC-SR04 처리를 여유가 큰 두 보드로
분배했습니다. Wi-Fi/HTTP, RFID, 경로, 서버 revision, STOP 재시도 권한은
SensorUno 한 곳에 그대로 남겨 제어 권한이 둘로 갈라지지 않게 했습니다.

## 최종 역할과 배선

### SensorUno

- ESP-01: Uno TX D5 → ESP RX(5V→3.3V 분압 필수), ESP TX → Uno RX D6
- RC522 소프트 SPI: SDA/SS D8, SCK D9, MOSI D10, MISO D11, RST D12
- 3-Uno 제어 I2C master: SDA A4, SCL A5
- D2, D3, D4: 분배 후 사용하지 않음
- 담당: 서버 통신, 명령 revision, 경로/RFID 판정, Motor/Actuator ACK, STOP 재시도

### MotorUno

- I2C slave `0x08`: SDA A4, SCL A5
- 라인 센서: 왼쪽 D9, 오른쪽 D10
- 뒤쪽 HC-SR04: ECHO D2(INT0), TRIG A1
- 모터: M1/M3 왼쪽, M2/M4 오른쪽
- 담당: 4륜 전진/후진, 라인트레이싱, watchdog, HOME 마커, 후진 장애물 정지
- 후진 중 `<15cm` 또는 ECHO `STUCK_HIGH`: 네 모터 즉시 `RELEASE`
- `>=18cm` 유효값 3회 연속: 로컬 후진 자동 재개
- `NO_ECHO`/`OUT_OF_RANGE`: 진단만 남기며 이미 걸린 장애물 정지는 해제하지 않음

### ActuatorUno

- I2C slave `0x09`: SDA A4, SCL A5
- DHT22 DATA: D2
- 가습 릴레이 A0, 펠티어 릴레이 A1, 냉각팬 릴레이 D7(active-low)
- LCD1602 별도 Software I2C: SDA D5, SCL D4 (`0x27`, 없으면 `0x3F`)
- 담당: 로컬 온습도 측정/LCD 첫 줄, 상태/LCD 둘째 줄, 릴레이 타이머
- DHT 오류는 LCD에만 표시하며 릴레이 출력과 actuator status를 바꾸지 않음

세 Uno의 GND는 공통으로 연결합니다. 서로 다른 전원의 `+` 출력끼리는 직접
묶지 않고, 모터·펠티어·팬 전류를 Uno 5V 핀이나 브레드보드로 흘리지 않습니다.

## 호환 프로토콜

- Motor: Sensor→Motor `[command, sequence]` 2바이트, Motor→Sensor
  `[status, appliedCommand, appliedSequence]` 3바이트
- Actuator 제어: `[0xA5, sequence, command, CRC8]` 4바이트
- Actuator 응답: `[status, command, appliedSequence, displaySequence,
  displayFlags, CRC8]` 6바이트
- 표시 telemetry는 기존 10바이트 형식을 유지합니다. SensorUno는 상태/구역/
  Wi-Fi/task/fault만 보내고, 예약된 온습도 4바이트는 0입니다. ActuatorUno가
  D2의 로컬 DHT22를 LCD 첫 줄에 직접 표시합니다.

## 안전한 현장 적용 순서

1. 서버에서 `ALL_STOP`을 유지하고 바퀴를 바닥에서 띄웁니다.
2. 세 Uno 및 모터/액추에이터 전원을 모두 끕니다.
3. DHT22 DATA를 Sensor D4에서 Actuator D2로 옮깁니다.
4. 뒤쪽 HC-SR04 ECHO를 Sensor D2에서 Motor D2로, TRIG를 Sensor D3에서
   Motor A1로 옮깁니다.
5. 공통 GND와 전원 극성을 확인합니다.
6. ActuatorUno, MotorUno, SensorUno 순서로 이 커밋의 세 production 스케치를
   각각 업로드합니다. 부분 업로드 상태로 주행하지 않습니다.
7. 시리얼 진단에서 Actuator DHT/LCD, Motor HC/모터 정지, Sensor I2C/ESP/RFID를
   개별 확인합니다.
8. 차를 HOME 넓은 검은 마커에 두고 ZONE2 방향을 향하게 한 뒤
   `CALIBRATE_HOME`을 실행합니다.
9. 바퀴를 띄운 상태의 짧은 전진/후진 시험 후에만 바닥 주행을 시작합니다.

## 배포 파일

- SensorUno: `firmware/uno_robot_esp01_rfid_relay/`
- MotorUno: `firmware/uno_line_tracker_motor_controller/`
- ActuatorUno: `firmware/uno_humidity_module_controller/`
- 분배 회귀 테스트:
  - `tests/test_motor_local_ultrasonic.py`
  - `tests/test_actuator_local_dht.py`
- 전체 검증: `py -3 scripts/check.py`

`robot_network_config.h`, `.env`, Wi-Fi 암호, RFID 실 UID는 배포 ZIP과 Git에
넣지 않습니다. 각 `.example` 파일에서 현장용 로컬 설정을 만듭니다.

## 검증 기준

- SensorUno: flash 27,190B(84%), SRAM 1,396B(68%, 652B 여유)
- MotorUno: flash 11,446B(35%), SRAM 468B(22%, 1,580B 여유)
- ActuatorUno: flash 15,452B(47%), SRAM 554B(27%, 1,494B 여유)
- Server tests: 39개 통과
- Robot/protocol/closed-loop tests: 129개 통과

이는 소스, AVR 컴파일, 오프라인 모델 검증 결과입니다. 새 배선 상태의 실제
DHT22/HC-SR04, 후진 정지거리, 모터 전원 여유는 업로드 후 실물에서 별도로
확인해야 합니다. 이 변경 작업에서는 새 분배 펌웨어를 실물 보드에 업로드하지
않았습니다.
