# 3 Uno SimulIDE 논리 프록시

권장 회로는 `mobile_humidity_robot_3uno_proxy.sim2`다. SensorUno, MotorUno,
ActuatorUno를 독립된 ATmega328P로 실행하고 A4/A5 I2C 계약과 상태 전이를
재현한다.

이 회로의 목적은 **프로토콜·인터록·상태 머신 검증**이다. 실제 로봇을 디지털
트윈으로 재현하지 않으며, 하드웨어 성능이나 통합 완료를 증명하지 않는다.

## 구성

| 보드 | I2C 역할 | 프록시 기능 |
| --- | --- | --- |
| SensorUno (`Uno-1`) | master | HOME 보정, 서버 명령, 역 순서, RFID 이벤트, DHT22 telemetry |
| MotorUno (`Uno-2`) | slave `0x08` | 보정 인터록, 라인 입력, 회전, watchdog, 좌·우 2WD 출력 |
| ActuatorUno (`Uno-3`) | slave `0x09` | 가습·펠티어·팬 시퀀스와 LCD 표시 |

Motor 명령은 `[command, sequence]`, 상태는
`[status, appliedCommand, appliedSequence]`다. MotorUno는 부팅 시
`CALIBRATION_REQUIRED(7)`이며, 두 IR 입력이 HOME 조건일 때
`HOME_SYNC(6)`를 같은 sequence로 ACK한 뒤에만 이동을 허용한다.

Actuator 명령은 `[0xA5, sequence, command, CRC8]`, 상태는
`[status, command, appliedSequence, displaySequence, flags, CRC8]`다.
별도 LCD telemetry는 10바이트 고정 프레임이며 두 CRC는 CRC-8/ATM
다항식 `0x07`을 사용한다. SensorUno는 같은 command/sequence의 RUNNING과
DONE만 현재 임무로 인정한다.

SimulIDE R260501의 다중 AVR slave-transmitter 제약 때문에 프록시 펌웨어는
Motor 상태 바이트를 `0xE0` 계열, Actuator 상태 바이트를 `0xF0` 계열 selector로
나누어 읽는다. 이 selector와 짧은 event-yield 지연은 `simulide/firmware`에만
있으며 운영 펌웨어의 wire frame을 바꾸지 않는다.

## 하드웨어와 프록시 대응

| 실제 계약 | 회로 입력/출력 |
| --- | --- |
| 자동차 DHT22 D4 | SimulIDE `Dht22` 모델 |
| 서버 명령 | SensorUno 명령 버튼 |
| ZONE2/ZONE99 RFID 판정 | 구역 이벤트 버튼; `<ZONE2_TAG_UID>` 같은 실제 UID를 사용하지 않음 |
| HOME 마커와 보정 | HOME 버튼 + MotorUno D9/D10 입력 |
| 왼쪽 모터 1개 / 오른쪽 모터 1개 | 좌·우 정/역방향 LED |
| IR 라인센서 | 좌·우 편차 버튼 |
| 가습·펠티어·팬 릴레이 | Actuator LED |
| LCD1602 백팩 | 주소 `0x27`의 `I2CToParallel` + 16×2 `Hd44780` |

프록시 버튼은 서버나 RFID 리더가 이미 판정한 이벤트를 주입한다. 회로는
ESP-01 AT, Wi-Fi HTTP, 카드 RF 판독, 태그 거리, 모터 전류·토크, 실제 릴레이
접점, 고전력 부하와 열·결로를 검증하지 않는다. HC-SR04는 현재 운영 코드에서도
관측 전용이므로 이 회로에서 제외한다.

## 실행

1. `simulide` 폴더 구조를 유지한 채 지원 `.sim2` 파일을 연다.
2. 실행 직후 모든 출력 LED가 OFF이고 Motor 상태가 7인지 확인한다.
3. 두 IR 입력이 HOME 조건일 때 `CALIBRATE_HOME / HOME` 또는 키 `h`를 누른다.
4. LCD의 온·습도와 상태를 확인하고 필요한 경우 세 시리얼 터미널을 연다.
5. 아래 표의 입력을 한 번씩 순서대로 준다.

`.sim2`만 다른 위치로 옮기면 상대 경로의 HEX를 찾지 못한다.

## ZONE99 제습 논리 시나리오

| 단계 | 입력 | 기대 상태 |
| --- | --- | --- |
| 1 | `CALIBRATE_HOME / HOME` (`h`) | 출력 OFF, HOME_SYNC ACK, `IDLE HOME` |
| 2 | `ZONE99 DEHUMIDIFY` (`0`) | 좌·우 정방향 LED ON |
| 3 | `RFID ZONE2` (`z`) | STOP, 중간 역 기록, 같은 방향 재출발 |
| 4 | `RFID ZONE99` (`x`) | 모터 OFF, 팬 LED ON |
| 5 | 단계 지연 | 펠티어 LED ON |
| 6 | 액추에이터 시퀀스 종료 | 액추에이터 OFF, 서버 재판정 대기 |
| 7 | `NORMAL / RETURN` (`r`) | 회전 프록시 뒤 HOME 방향 주행 |
| 8 | `RFID ZONE2` (`z`) | 중간 역 확인 뒤 재출발 |
| 9 | `HOME marker` (`h`) | 모든 출력 OFF, `HOME / IDLE` |

가습 시나리오는 화면의 ZONE2/ZONE99 HUMIDIFY 입력을 사용한다. `ALL_STOP`
입력은 어느 단계에서든 Motor와 Actuator 출력을 OFF로 만들어야 한다. 라인 입력은
왼쪽 또는 오른쪽 편차 버튼으로 바꾸며, 출력 LED는 해당 시점의 방향 신호만
뜻한다.

## 오류 시나리오

- HOME 보정 전 임무/복귀 입력은 상태 7로 차단되고 출력은 OFF다.
- 한쪽 IR을 HOME 조건에서 벗어나게 한 보정은 실패하고 움직이지 않는다.
- MotorUno 재부팅 뒤 다음 상태 확인에서 SensorUno도 미보정 상태로 잠긴다.
- 예상 순서와 다른 구역 이벤트는 SAFE_STOP으로 끝난다.
- 프록시에서는 복귀 중 중간 역 확인 전 HOME 입력이 SAFE_STOP으로 끝난다.
  운영 SensorUno는 같은 상황에서 HOME 마커로 정지·위치 복구 후
  `FAILED / HOME_RFID_MISSED`를 보고하므로, 이 실패 복구 분기는 프록시가
  그대로 재현하지 않는 차이점이다.
- keepalive가 2초 이상 끊기면 Motor 출력이 OFF된다.
- Actuator의 잘못된 magic, CRC, 길이 또는 sequence 충돌은 전체 출력 OFF와
  ERROR 상태로 끝난다.

## build manifest와 SHA-256

세 프록시 소스와 HEX는
[`firmware/build-manifest.json`](firmware/build-manifest.json)에 다음 정보로
묶여 있다.

- Arduino FQBN (`target`)
- source 상대 경로와 `source_sha256`
- HEX 상대 경로와 `hex_sha256`

검증 명령:

```powershell
python simulide\validate_sim2.py
```

검사기는 회로가 참조하는 source/HEX 경로가 manifest와 같은지 확인한 뒤 각
파일의 SHA-256을 계산해 manifest 값과 비교한다. 수정 시간이 최신이라는 이유만
으로 HEX를 신뢰하지 않는다. 성공 출력에는 다음 문장이 포함된다.

```text
all three firmware HEX/source hashes match the build manifest
```

스케치 또는 빌드 옵션이 변경되면 다음을 하나의 변경으로 처리한다.

1. `arduino:avr:uno` 대상으로 해당 프록시를 다시 컴파일한다.
2. 회로가 참조하는 `.ino.hex`를 새 결과로 교체한다.
3. source와 HEX SHA-256을 다시 계산한다.
4. manifest의 경로·해시·target을 갱신한다.
5. `validate_sim2.py`와 전체 오프라인 검사를 다시 실행한다.

```powershell
python scripts\check.py
```

해시가 일치하지 않거나 manifest가 없으면 회로 스크린샷과 수동 시나리오 결과를
검증 근거로 제출하지 않는다.

## 검증 결과 기록 규칙

결과에는 commit, SimulIDE 버전, manifest SHA, 실행한 입력 시나리오와 실제
출력을 기록한다. 다음 표현을 구분한다.

- `static PASS`: XML·연결·manifest SHA 검사 통과
- `proxy scenario PASS`: 버튼/LED/LCD 상태 전이 관찰
- `hardware PASS`: 별도 하드웨어 시험 기록이 있을 때만 사용

다른 회로와 데모 펌웨어는 초기 아이디어 보존용이며 현재 3 Uno · 2WD 구조의
검증 근거로 사용하지 않는다.
