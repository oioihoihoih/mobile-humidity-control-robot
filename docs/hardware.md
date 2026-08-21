# 3 Uno · 4모터 전·후진 하드웨어 구성

이 문서는 현재 소스가 목표로 하는 하드웨어 계약을 설명한다. 특정 PC의 IP,
Windows COM 번호, Wi-Fi 자격 증명, RFID UID는 환경별 값이므로 저장소 문서에
고정하지 않는다. 실제 배포 전에는 로컬 설정 파일과 장치 로그로 다시 확인한다.

## 전체 구조

| 장치 | 책임 | 연결 |
| --- | --- | --- |
| 서버 PC | ZONE2/ZONE99 측정 저장, 임계값·우선순위 판정, 명령과 웹 UI | `<SERVER_HOST>:<SERVER_PORT>` |
| SensorUno | ESP-01, RC522, DHT22, 후방 초음파, 경로 상태, 하위 보드 조율 | Wi-Fi + I2C master |
| MotorUno | 기존 모터 M1/M2 + N20 1:298 M3/M4, 2채널 라인센서, HOME 동기화, watchdog | I2C slave `0x08` |
| ActuatorUno | 가습·펠티어·팬 릴레이, LCD1602 | I2C slave `0x09` + LCD 전용 software I2C |
| 구역 센서 노드 | 구역별 DHT11/22 측정 전송 | ESP-01 Wi-Fi |

서버는 Arduino가 아니라 별도 컴퓨터에서 실행한다. 자동차와 구역 센서 노드는
같은 네트워크에서 서버에 연결하지만, 자동차의 이동 상태와 관계없이 구역 측정은
계속된다.

USB 시리얼 포트 이름은 연결 순서와 컴퓨터에 따라 바뀐다. 업로드 전에는 보드
식별자와 부트 로그로 SensorUno, MotorUno, ActuatorUno를 식별하고
`<SENSOR_PORT>`, `<MOTOR_PORT>`, `<ACTUATOR_PORT>`에 대응시킨다.

## SensorUno

| 핀 | 연결 |
| --- | --- |
| D2 | HC-SR04 ECHO |
| D3 | HC-SR04 TRIG |
| D4 | DHT22 DATA |
| D5 | UNO TX → ESP-01 RX, 3.3V 레벨 변환 필수 |
| D6 | UNO RX ← ESP-01 TX |
| D8 | RC522 SDA/SS |
| D9 | RC522 SCK |
| D10 | RC522 MOSI |
| D11 | RC522 MISO |
| D12 | RC522 RST |
| A4/A5 | MotorUno `0x08`, ActuatorUno `0x09` 제어 I2C |

RC522는 이 비표준 핀맵에 맞춘 프로젝트의 software-SPI 드라이버를 사용한다.
RC522와 ESP-01은 3.3V 장치다. 특히 ESP-01은 송신 순간 전류를 감당할 수 있는
안정된 3.3V 전원과 공통 GND가 필요하다.

HC-SR04 본체는 N20 후륜 쪽을 향하게 장착하지만 신호선은 계속 SensorUno의
D2/D3에 연결한다. MotorUno의 D3는 M2 PWM이므로 초음파를 MotorUno D2/D3로
옮기면 충돌한다. 후방 센서는 전진 중에는 측정만 한다. 후진 출발 전 완료
sample이 stale/미준비이거나 `STUCK_HIGH`, 유효 거리 15cm 미만이면 출발하지
않는다. 주행 중에는 15cm 미만 또는 `STUCK_HIGH`에서 PAUSE하고 18cm 이상을
3회 확인해야 재개한다. `NO_ECHO`와 `OUT_OF_RANGE`는 진단만 남긴다.

RFID 매핑은 배포마다 로컬에서 읽은 값을 사용한다.

```text
ZONE2  → <ZONE2_TAG_UID>
ZONE99 → <ZONE99_TAG_UID>
```

UID를 README, 화면 캡처, 로그 예시나 공개 이슈에 기록하지 않는다. SensorUno는
구역명과 상태만 서버에 보고한다.

SensorUno는 자동차 DHT22 값과 상태를 고정 10바이트 telemetry로
ActuatorUno에 보낸다. LCD 오류는 표시 오류로만 다루고, 모터나 릴레이의 안전
정지 판단과 분리한다.

부팅 때 위치와 다음 역은 `UNKNOWN`, 경로 보정은 false다. 차량을 HOME의 넓은
검은 마커 위에서 ZONE2 방향으로 놓고 `CALIBRATE_HOME`을 완료하기 전에는
TASK, RETURN_HOME, 수동 주행을 거절한다. HOME 동기화는 마커만 확인하므로
차체 방향은 사용자가 확인해야 한다.

## MotorUno · M1~M4 4모터

| 핀/출력 | 연결 |
| --- | --- |
| D9/D10 | 왼쪽/오른쪽 IR 라인센서 |
| A4/A5 | I2C SDA/SCL, slave `0x08` |
| AFMotor M1 | 기존 앞쪽 왼쪽 모터 |
| AFMotor M2 | 기존 앞쪽 오른쪽 모터 |
| AFMotor M3 | N20 1:298 뒤쪽 왼쪽 모터, M1과 같은 측·방향 |
| AFMotor M4 | N20 1:298 뒤쪽 오른쪽 모터, M2와 같은 측·방향 |

M1/M3을 왼쪽 한 쌍, M2/M4를 오른쪽 한 쌍으로 제어하되 네 모터는 각각 별도
실드 채널을 사용한다. M1/M2와 M3/M4는 기어비와 바퀴 지름이 다르므로 코드의
`EXISTING_AXLE_SPEED`, `N20_REAR_AXLE_SPEED`를 별도로 조절한다. PWM만으로
실제 선속도 차이를 완전히 없앨 수 없으며, 강체 4륜에서 한 축이 다른 축을 끌면
전류·타이어 마찰·드라이버 발열이 증가할 수 있다.

새 바퀴 높이를 맞추기 위해 차체 판을 뒤집은 현재 조립에서는 부품 이름보다 실제
주행 방향을 기준으로 확인한다. HOME에서 ZONE2 쪽이 전진이고 N20·초음파 쪽이
후진이다. 업로드 뒤에는 네 바퀴를 띄워 M1/M3 및 M2/M4가 각각 같은 물리
방향인지, IR 센서가 바닥을 향하고 좌우 판정이 뒤바뀌지 않았는지 먼저 확인한다.

MotorUno는 SensorUno의 주기적 KEEPALIVE가 끊기면 네 모터를 RELEASE한다.
라인센서는 연속된 검은 선의 좌우 편차를 보정하고, 역 도착은 SensorUno의 RFID
판정 뒤 STOP 적용 ACK로 확정한다. `BENCH_RFID_ONLY_MODE`는 진단 전용이며
운영 빌드에서는 false여야 한다.

명령 1은 네 바퀴 전진, 명령 2는 차체를 180도 돌리지 않는 네 바퀴 후진이다.
진행 방향을 바꿀 때는 모든 채널을 잠시 RELEASE한 뒤 반대 방향을 적용한다.
라인 보정도 한쪽 바퀴를 반대 방향으로 돌리지 않고 같은 진행 방향에서 한쪽
PWM만 낮춘다. 후진에서는 차체 반대편에 있는 라인센서의 조향 효과를 고려해
좌우 보정량을 반대로 적용한다.

MotorUno는 부팅 시 모터를 RELEASE하고 `PROTOCOL_REQUIRED(8)`을 반환한다.
SensorUno가 `PROTOCOL_SYNC(7)`의 같은 command/sequence와 status 8을 확인한
뒤에만 `CALIBRATION_REQUIRED(7)` 단계로 진행한다. `HOME_SYNC(6)`은 탐색
주행 없이 양쪽 IR이 HOME 마커 조건인지 확인한다. versioned 이동 명령
`0x11/0x12`는 구형 값 `1/2`와 분리되어 혼합 펌웨어의 이동을 거절한다.

## ActuatorUno

| 핀 | 연결 | 동작 |
| --- | --- | --- |
| A0 | 가습기 릴레이 IN | HUMIDIFY |
| A1 | 펠티어 릴레이 IN | DEHUMIDIFY |
| D7 | 냉각팬 릴레이 IN | 펠티어 전후 냉각 |
| D5/D4 | LCD1602 SDA/SCL | 별도 software I2C |
| A4/A5 | SensorUno 제어 I2C | slave `0x09` |

릴레이는 active-low 구성이다. 부팅, STOP, timeout, 잘못된 명령 프레임에서는
A0/A1/D7을 모두 OFF로 둔다. 가습은 제한 시간 뒤 자동 종료한다. 제습은 팬
선가동, 펠티어 제한 동작, 팬 후열 냉각 순서로 끝난다. 시간 상수는 소스를
권위 기준으로 삼는다.

LCD는 ActuatorUno의 D5/D4 전용 버스에만 연결한다. 세 Uno의 A4/A5 제어
버스와 합치지 않는다. 기본 주소와 대체 주소 탐색은 표시 편의를 위한 기능이며,
LCD 실패가 릴레이 상태 머신을 우회하게 해서는 안 된다.

## 제어 프로토콜

- Motor 명령: `[command, sequence]`; `0x11=네 바퀴 전진`, `0x12=네 바퀴 후진`,
  `3=PAUSE`, `4=RESUME`, `5=KEEPALIVE`, `6=HOME_SYNC`, `7=4모터 프로토콜 동기화`
- Motor 상태: `[status, appliedCommand, appliedSequence]`
- Actuator 명령: `[0xA5, sequence, command, CRC8]`
- Actuator 상태: `[status, command, appliedSequence, displaySequence, flags, CRC8]`
- LCD telemetry: magic, sequence, 상태, 구역, 온도, 습도, flags, CRC를 담은
  고정 10바이트 프레임
- CRC: CRC-8/ATM, 다항식 `0x07`

SensorUno는 현재 command와 sequence가 모두 일치하는 RUNNING을 본 뒤 같은
tuple의 DONE만 완료로 인정한다. 중복 tuple은 멱등 처리하고, 오래된 ACK,
잘못된 magic/CRC/길이와 같은 sequence의 충돌 명령은 성공으로 인정하지 않는다.
SensorUno·MotorUno는 반드시 같은 커밋을 연속 업로드하고, status 8 → command 7
exact ACK → status 7 → HOME_SYNC 순서를 현장에서 확인한다.

## 전원 원칙

- 세 Uno와 센서·제어 모듈의 GND는 공통으로 연결한다.
- 서로 다른 전원 장치의 양극 출력은 직접 묶지 않는다.
- 모터, 펠티어, 팬, 가습기 부하 전류를 Uno 핀이나 브레드보드로 흘리지 않는다.
- Motor Shield의 외부 모터 전원과 Uno 논리 전원을 분리한다.
- M1~M4 각 모터의 정지전류가 해당 드라이버 채널 정격 안인지 확인한다. 서로
  다른 모터를 장착한 상태에서는 네 바퀴 동시 기동 전류와 AA 팩 전압 강하를
  실제로 측정한다.
- RC522에는 3.3V만 공급한다.
- ESP-01에는 500mA 이상을 공급할 수 있는 안정된 3.3V 레귤레이터와 근접
  디커플링을 사용한다.
- 고전력 부하는 정격 퓨즈, 스위칭 소자, 방열, 굵은 배선과 결로 격리를 포함해
  별도 검토한다.

## 환경별 설정

공개 저장소에는 예제 설정만 둔다. 실제 값은 무시되는 로컬 파일이나 환경
변수로 제공한다.

| 값 | 자리표시자 |
| --- | --- |
| 서버 주소 | `<SERVER_HOST>` |
| 서버 포트 | `<SERVER_PORT>` |
| 서버 시리얼 | `<SERIAL_PORT>` |
| ZONE2/ZONE99 UID | `<ZONE2_TAG_UID>`, `<ZONE99_TAG_UID>` |
| Wi-Fi 자격 증명 | 로컬 secrets 파일 |
| MySQL 자격 증명 | `MYSQL_*` 환경 변수 |

예제 헤더를 복사한 뒤 실제 값을 채우되, 생성한 로컬 설정 파일은 커밋하지
않는다. 서버는 `ROBOT_BIND_HOST`, `ROBOT_PORT`, `SERIAL_PORT`와 `MYSQL_*`
환경 변수를 사용한다.

## 임무 흐름

1. 세 Uno는 출력 OFF와 경로 UNKNOWN으로 부팅한다.
2. 사용자가 HOME 마커와 방향을 확인하고 `CALIBRATE_HOME`을 수행한다.
3. ZONE2/ZONE99 센서는 자동차와 독립적으로 측정을 계속 전송한다.
4. 서버는 fresh 데이터만 사용해 목표와 HUMIDIFY/DEHUMIDIFY를 정한다.
5. MotorUno는 출동 때 전진, 복귀 때 후진으로 연속 검은 선을 따라가고 SensorUno는 RFID를 계속 스캔한다.
6. 예상 역 이벤트에서 STOP ACK를 받은 뒤 위치를 확정한다. 목표가 아니면
   중복 인식을 막고 같은 방향으로 다시 출발한다.
7. 목표 역에서 Actuator exact ACK를 확인하며 제한 시간 동작을 수행한다.
8. 서버는 완료 이후 들어온 새 측정으로 재작동 또는 RETURN_HOME을 결정한다.
9. 복귀 중 새 TASK가 생기면 현재 위치와 방향에 맞춰 재라우팅한다.
10. HOME 방향에서 중간 역을 확인한 뒤 넓은 HOME 마커에서 정상 완료한다.

HOME에는 RFID가 없으므로 최종 위치는 방향·중간 역·마커 조건을 함께 사용한다.
복귀 중 ZONE2 태그를 놓친 경우에도 HOME 마커에서 정지해 위치는 복구하지만
`FAILED / HOME_RFID_MISSED`를 보고한다. 이는 완전한 절대 위치 측정이 아니며,
태그 누락이나 재부팅 뒤에는 원인을 점검하고 HOME에서 다시 보정해야 한다.

## 검증 경계

| 범위 | 저장소가 제공하는 근거 | 제공하지 않는 근거 |
| --- | --- | --- |
| Python 테스트 | 서버 판단, revision/ACK, 프로토콜·상태 모델 | 실제 네트워크·전원·기구 동작 |
| SimulIDE | 3 Uno I2C, 프레임, 상태 전이, LCD와 출력 신호 프록시 | 실제 RFID RF, Wi-Fi, 모터 전류, 릴레이 부하 |
| 펌웨어 빌드 | 지정 소스와 HEX의 SHA-256 결합 | HEX가 특정 하드웨어에 업로드됐다는 증거 |
| 벤치 시험 | 실행한 시점과 배선에서 관찰한 제한적 결과 | 전체 트랙 왕복, 장시간 부하, 재현성 보장 |

전체 오프라인 검사는 저장소 루트에서 실행한다.

```powershell
python scripts\check.py
```

SimulIDE 검사는 `simulide/firmware/build-manifest.json`의 source/HEX 경로와
SHA-256을 실제 파일에 대조한다. 자세한 갱신·검증 절차는
[`../simulide/README.md`](../simulide/README.md)를 따른다.

하드웨어 완료를 주장하려면 별도로 다음을 기록해야 한다.

- 사용한 보드 식별자와 펌웨어 commit
- 전원 정격과 무부하/부하 전압
- 좌·우 모터 방향, 라인센서 극성, 태그 높이와 제동 거리
- 가습·펠티어·팬의 무부하 및 정격 부하 시험
- 전체 `HOME → ZONE2 → ZONE99 → HOME` 왕복과 안전 정지 결과

현재 공개 문서는 위 항목의 완료를 전제로 하지 않는다. 후방 초음파 PAUSE는
보조 기능이며, 4모터 속도 정합·전체 트랙·RFID·고전력 부하는 하드웨어 검증
대상으로 남아 있다.
