# 테스트와 검증 범위

이 프로젝트는 소스 모델, 컴파일, 회로 프록시, 실물 벤치와 실차 트랙을 서로 다른 증거로 취급합니다. 한 단계의 통과를 다음 단계의 성공으로 확대 해석하지 않습니다.

## 현재 상태

| 단계 | 상태 | 현재 증거 | 증명하지 않는 것 |
| --- | --- | --- | --- |
| Python 서버 단위시험 | 자동화됨 | 임무 우선순위, stale·히스테리시스, DB 워터마크, 명령·ACK와 제어 API 계약 | 실제 MySQL 설치 상태, Wi-Fi 품질, 실차 |
| 3-Uno 프로토콜·폐루프 모델 | 자동화됨 | calibration, 경로 순서, RFID 이벤트, watchdog, actuator sequence/CRC와 실패 흐름 | 실제 I2C 신호 품질, 카드 판독 거리, 모터 관성 |
| SimulIDE 계약 검사 | 자동화됨 | 현재 회로 파일, 프록시 펌웨어와 빌드 manifest의 일치 | ESP-01 RF 통신, 실제 RC522, 고전력 부하 |
| Uno 운영 스케치 컴파일 | CI 게이트 제공 | 대상 Uno용 세 운영 스케치가 빌드 가능한지 확인 | 보드 업로드 성공과 런타임 안정성 |
| 실물 하드웨어 | **벤치 검증만 완료** | 정지 상태의 보드 연결, 기본 통신과 안전 출력 | 2륜 연속 라인 왕복, 주행 중 RFID 제동, 고전력 임무 |
| 실차·부하 통합 | 미검증 | 아직 완료 증거 없음 | — |

현재 회귀시험 구성은 서버 39개와 `tests/` 103개, 총 142개입니다. SensorUno 운영 빌드는 플래시 사용률이 약 99%이며 CI budget은 flash `32,100B`, SRAM `1,600B`입니다. 작은 변경도 빌드 실패나 기능 제거로 이어질 수 있으므로 컴파일 크기 변화는 필수 검토 항목입니다.

## 전체 오프라인 검사

Python 가상환경을 활성화하고 서버 의존성을 설치한 뒤 저장소 루트에서 실행합니다.

```text
python -m pip install -r server/requirements.txt
python scripts/check.py
```

`scripts/check.py`는 다음 검사를 순서대로 실행하고 하나라도 실패하면 0이 아닌 종료 코드를 반환합니다.

1. `server/test_server_logic.py`의 서버 단위시험
2. `tests/test_*.py`의 3-Uno 프로토콜·폐루프 시험
3. `simulide/validate_sim2.py`의 회로·펌웨어 manifest 검사

마지막 줄의 `All offline checks passed.`는 위 세 묶음이 현재 작업본에서 통과했다는 뜻입니다. 현재 기대값은 서버 39개 + 프로토콜·폐루프 103개 = 142개이지만, 테스트 개수는 소스가 늘면 바뀔 수 있으므로 README에는 고정하지 않고 실행 결과와 CI run을 근거로 사용합니다.

## 개별 검사

문제를 좁힐 때는 각 묶음을 따로 실행할 수 있습니다.

```text
python -m unittest discover -s server -p "test_*.py" -v
python -m unittest discover -s tests -p "test_*.py" -v
python simulide/validate_sim2.py
```

서버 단위시험은 MySQL·시리얼·시간과 네트워크 상태를 테스트 대역으로 분리합니다. 따라서 로컬 MySQL이 꺼져 있어도 통과할 수 있으며, 이것을 `/ready` 성공의 근거로 사용하면 안 됩니다.

## GitHub Actions

[`.github/workflows/ci.yml`](../.github/workflows/ci.yml)은 push와 pull request에서 두 작업을 실행합니다.

### Offline regression

- Python 3.12 환경 구성
- 서버 의존성 설치
- `python scripts/check.py`
- 호스트 C++로 `SoftwareMFRC522` 회귀시험 컴파일·실행

호스트 C++ 시험은 Arduino 없이 RFID 소프트웨어 SPI 파서의 일부 계약을 확인합니다. 실제 RC522 전압, 배선, SPI 타이밍과 판독 거리는 확인하지 않습니다.

### Arduino Uno firmware compile

- 공개 예제 네트워크 헤더로 CI 전용 설정 생성
- `arduino:avr:uno` 대상으로 SensorUno, MotorUno, ActuatorUno 운영 스케치 컴파일
- 컴파일 경고와 메모리 사용량 보고

CI에는 실제 Wi-Fi 자격증명이나 제어 토큰을 넣지 않습니다. 플래시가 약 99%인 SensorUno는 특히 다음을 확인합니다.

- 프로그램 저장공간 초과 여부와 flash `32,100B` budget
- 전역·정적 SRAM과 `1,600B` budget 및 남는 스택 여유
- 새 문자열이 `F()` 또는 `PROGMEM` 없이 SRAM을 소비하지 않는지
- 긴 HTTP/AT 처리로 RFID 스캔, I2C keepalive와 안전 정지가 지연되지 않는지

컴파일 배지는 최신 원격 workflow 결과를 표시할 뿐, 실물 통과 배지가 아닙니다.

## 서버 스모크 테스트

MySQL 데이터베이스와 전용 계정을 준비한 뒤 서버를 로컬 전용·시리얼 비활성 상태로 시작합니다. 자세한 설정은 [설치 문서](setup.md)를 따릅니다.

```text
python server/server.py
```

다음 순서로 확인합니다.

1. `/health`가 `200`과 `ok: true`를 반환한다.
2. `/ready`가 `200`, `database: ready`를 반환한다.
3. ZONE2와 ZONE99의 정상 측정을 보내면 두 구역이 fresh로 표시된다.
4. 한 구역의 새 측정을 기본 습도 하한 아래로 보내면 해당 구역의 `TASK / HUMIDIFY`가 생성된다.
5. 같은 구역을 상한 위로 보내면 히스테리시스와 새 revision이 예상대로 반영된다.
6. 측정을 stale 제한보다 오래 중단했을 때 신선한 위반 후보가 없으면 `ALL_STOP`이 유지된다.

샘플 측정은 실제 DB를 변경하므로 운영 DB 대신 로컬 테스트 DB에서 실행합니다. 스모크 테스트는 API·MySQL 통합을 확인하지만 로봇이 명령을 실행했다는 뜻은 아닙니다.

## 벤치 검증 기준

현재 실물 검증은 정지 상태 또는 바퀴를 띄운 상태로 제한합니다. 재현할 때는 다음 항목을 기록합니다.

| 항목 | 합격 기준 |
| --- | --- |
| 안전 부팅 | MotorUno 모터 RELEASE, ActuatorUno 모든 릴레이 OFF, 경로 calibration 미완료 |
| 3-Uno I2C | SensorUno가 MotorUno와 ActuatorUno를 서로 다른 주소에서 식별하고 잘못된 ACK를 성공으로 처리하지 않음 |
| HOME 동기화 | 넓은 마커 조건과 사용자 방향 확인 뒤에만 `CALIBRATE_HOME` 완료 ACK |
| watchdog | SensorUno keepalive가 끊기면 MotorUno가 제한시간 뒤 스스로 정지 |
| actuator | 고전력 부하 없이 명령·sequence·CRC와 자동 OFF를 확인 |
| LCD 격리 | 표시 오류가 릴레이 안전 상태를 우회하거나 임의 가동을 만들지 않음 |

사진이나 로그에는 Wi-Fi 자격증명, 내부 주소, 시리얼 포트, RFID UID와 제어 토큰이 보이지 않게 가립니다.

## 아직 필요한 실차 검증

다음 항목이 모두 관찰되기 전에는 “완전 자율 왕복 완료”로 표시하지 않습니다.

1. 저전력·바퀴 하중 상태에서 좌우 모터 방향과 최소 기동 PWM 측정
2. 연속 검은 선에서 직진·보정·180도 회전의 반복성 확인
3. 주행 속도별 ZONE2·ZONE99 RFID 판독률과 정지거리 측정
4. `HOME → ZONE2 → ZONE99 → ZONE2 → HOME` 무부하 왕복
5. RFID 누락·순서 오류, SensorUno 재부팅, I2C 단선과 Wi-Fi 끊김 고장 주입
6. 가습·제습 부하를 하나씩 연결해 전류, 전압 강하, 발열과 자동 OFF 측정
7. 최악 조건에서 모터와 액추에이터를 함께 쓸 때 전원·퓨즈·방열 검증
8. 실제 공간 또는 시험 상자에서 임무 전후 습도 변화량과 재측정 시간 확인

실차 위험과 권장 진행 순서는 [현재 한계](limitations.md)에 정리되어 있습니다.

## 결과 기록 형식

재현 가능한 증거에는 최소한 다음 정보를 남깁니다.

```text
날짜/커밋:
시험 단계: offline | compile | bench | track | load
대상 보드/서버:
설정 변경: 비밀값을 제외한 상수와 모드
실행 명령 또는 절차:
예상 결과:
관찰 결과:
PASS/FAIL:
남은 위험:
```

“PASS”는 적은 시험 단계에만 적용합니다. 예를 들어 벤치 PASS는 트랙·부하 PASS를 포함하지 않습니다.
