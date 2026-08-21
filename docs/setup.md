# 설치와 실행

이 문서는 저장소 루트를 현재 디렉터리로 사용합니다. 특정 PC의 절대 경로나 고정 내부 주소를 복사하지 말고, 자신의 환경에 맞는 값은 환경 변수와 Git에서 제외된 로컬 설정 헤더에만 넣으세요.

## 1. 준비물

### 서버 데모

- Git
- Python 3.12
- MySQL 서버
- MySQL 관리자 계정: 최초 데이터베이스와 전용 사용자를 만들 때만 사용
- MySQL 전용 애플리케이션 계정: 평상시 서버 실행에 사용

### 하드웨어 통합

- 위 서버 환경
- 신뢰할 수 있고 격리된 로컬 LAN
- Arduino IDE 또는 Arduino CLI와 대상 보드 코어·라이브러리
- SensorUno, MotorUno, ActuatorUno와 구역 센서 노드 2대
- 모터·펠티어·팬용 정격 전원, 드라이버, 퓨즈와 방열 장치

첫 실행은 하드웨어를 연결하지 않은 로컬 서버 모드로 진행하세요.

## 2. 저장소와 Python 환경

```text
git clone https://github.com/oioihoihoih/mobile-humidity-control-robot.git
cd mobile-humidity-control-robot
python -m venv .venv
```

PowerShell:

```powershell
.venv\Scripts\Activate.ps1
python -m pip install --upgrade pip
python -m pip install -r server/requirements.txt
```

macOS 또는 Linux:

```bash
source .venv/bin/activate
python -m pip install --upgrade pip
python -m pip install -r server/requirements.txt
```

## 3. MySQL 데이터베이스와 전용 사용자

기본 운영 흐름에서는 서버가 데이터베이스 자체를 자동 생성하지 않습니다. MySQL 관리자 세션에서 데이터베이스와 전용 사용자를 한 번 만들고, 이후 Python 서버는 그 전용 계정으로만 실행합니다.

다음 예시의 비밀번호 자리표시자를 실제 강한 비밀번호로 바꾸되, 실행한 SQL이나 비밀번호를 저장소에 커밋하지 마세요.

```sql
CREATE DATABASE mobile_humidity_robot
  CHARACTER SET utf8mb4
  COLLATE utf8mb4_unicode_ci;

CREATE USER 'humibot'@'127.0.0.1'
  IDENTIFIED BY '<db-password>';

GRANT SELECT, INSERT, UPDATE, DELETE, CREATE, ALTER, INDEX
  ON mobile_humidity_robot.*
  TO 'humibot'@'127.0.0.1';

FLUSH PRIVILEGES;
```

서버는 기존 데이터베이스 안에서 필요한 테이블을 생성하고 호환 가능한 마이그레이션을 적용합니다. 기존 `manual_control` 테이블이 현재 스키마와 다르면 자동으로 추측해 고치지 않고 오류를 내므로, 백업 후 스키마 차이를 확인해야 합니다.

개발 환경에서 데이터베이스 생성까지 서버에 맡겨야 할 때만 `MYSQL_AUTO_CREATE_DATABASE=1`을 명시적으로 설정할 수 있습니다. 이 모드는 `CREATE DATABASE` 권한이 있는 계정이 필요하므로 최초 부트스트랩 뒤에는 변수를 제거하고 전용 계정으로 돌아오세요.

## 4. 로컬 서버 시작

기본 바인드 주소는 loopback이므로 같은 PC에서만 접근할 수 있습니다. USB 시리얼 브리지를 끄면 연결된 보드 없이 서버·DB·대시보드를 확인할 수 있습니다.

PowerShell:

```powershell
$env:MYSQL_HOST = "127.0.0.1"
$env:MYSQL_PORT = "3306"
$env:MYSQL_USER = "humibot"
$env:MYSQL_PASSWORD = "<db-password>"
$env:MYSQL_DATABASE = "mobile_humidity_robot"
$env:SERIAL_ENABLED = "0"
python server/server.py
```

macOS 또는 Linux:

```bash
export MYSQL_HOST="127.0.0.1"
export MYSQL_PORT="3306"
export MYSQL_USER="humibot"
export MYSQL_PASSWORD="<db-password>"
export MYSQL_DATABASE="mobile_humidity_robot"
export SERIAL_ENABLED="0"
python server/server.py
```

이 서버는 `.env` 파일을 자동으로 읽지 않습니다. 비밀번호는 현재 셸, 운영체제의 비밀 저장소 또는 배포 환경의 secret 기능으로 주입하세요.

브라우저에서 다음 경로를 확인합니다.

| 경로 | 정상 기준 |
| --- | --- |
| `http://127.0.0.1:8000/` | 대시보드 HTML 표시 |
| `http://127.0.0.1:8000/health` | `ok: true`와 빌드 정보 반환 |
| `http://127.0.0.1:8000/ready` | `ok: true`, `database: ready` 반환 |
| `http://127.0.0.1:8000/logic` | 시스템 로직 페이지 표시 |

`/health`는 프로세스가 응답한다는 뜻이고 `/ready`는 MySQL 연결까지 확인합니다. 배포·시연 준비 판단에는 `/ready`를 사용하세요.

## 5. 샘플 측정으로 서버 확인

서버가 실행 중일 때 두 활성 구역의 정상 측정을 한 번씩 전송합니다. PowerShell에서는 `curl.exe`, macOS·Linux에서는 `curl`을 사용하세요.

```text
curl.exe -X POST http://127.0.0.1:8000/api/readings -H "Content-Type: application/json" -d "{\"zone_id\":\"ZONE2\",\"temperature\":24.0,\"humidity\":70.0}"
curl.exe -X POST http://127.0.0.1:8000/api/readings -H "Content-Type: application/json" -d "{\"zone_id\":\"ZONE99\",\"temperature\":24.0,\"humidity\":70.0}"
```

두 값이 fresh·normal이면 대시보드의 자동 명령이 `RETURN_HOME`이 됩니다. 이어서 한 구역에 기본 하한보다 낮은 새 값을 보내면 해당 구역의 `HUMIDIFY` 임무가 생성되는지 확인할 수 있습니다.

```text
curl.exe -X POST http://127.0.0.1:8000/api/readings -H "Content-Type: application/json" -d "{\"zone_id\":\"ZONE2\",\"temperature\":24.0,\"humidity\":55.0}"
```

이 요청은 DB와 자동 임무 상태를 실제로 변경합니다. 테스트가 끝난 뒤 대시보드에서 상태를 확인하고, 운영 데이터베이스에는 임의 샘플을 보내지 마세요.

## 6. 신뢰 LAN에서 실행

ESP-01 장치가 서버에 접속하려면 서버가 LAN 인터페이스에 바인드되어야 합니다. 다음 설정은 **격리되거나 신뢰할 수 있는 로컬 네트워크에서만** 사용하세요.

```powershell
$env:ROBOT_BIND_HOST = "0.0.0.0"
$env:CONTROL_API_TOKEN = "<long-random-control-token>"
$env:SERIAL_ENABLED = "0"
python server/server.py
```

- 방화벽은 서버 포트를 필요한 로컬 서브넷에만 허용합니다.
- 라우터 포트 포워딩, 공인 인터넷 공개와 공개 Wi-Fi 사용을 금지합니다.
- `CONTROL_API_TOKEN`은 LAN의 위험한 제어 엔드포인트만 Bearer 토큰으로 제한합니다. 센서·로봇 상태 API에는 장치 인증이나 TLS가 없으므로 전체 네트워크 보안 대책이 아닙니다.
- 서버 PC의 현재 LAN 주소는 문서나 커밋에 기록하지 말고 각 장치의 로컬 설정 파일에만 넣습니다.

Windows의 네트워크 전용 실행 스크립트도 사용할 수 있습니다.

```powershell
& server/start_network_only.ps1
```

이 스크립트는 LAN 바인드를 활성화하므로 실행 전에 위 네트워크 경계를 확인해야 합니다.

## 7. 펌웨어의 로컬 네트워크 설정

예제 헤더를 Git에서 제외되는 실제 설정 파일로 복사합니다.

```powershell
Copy-Item firmware/uno_robot_esp01_rfid_relay/robot_network_config.example.h firmware/uno_robot_esp01_rfid_relay/robot_network_config.h
Copy-Item firmware/zone2_esp01_direct/secrets.example.h firmware/zone2_esp01_direct/secrets.h
Copy-Item firmware/zone99_esp01_dht11/secrets.example.h firmware/zone99_esp01_dht11/secrets.h
```

복사한 파일에서 다음 값만 로컬 환경에 맞게 설정합니다.

- Wi-Fi SSID와 비밀번호
- 서버 PC의 현재 LAN 주소와 서버 포트
- SensorUno가 읽은 ZONE2·ZONE99 RFID UID
- 구역 노드의 `ZONE_ID`와 실제 DHT 종류

실제 설정 헤더는 `.gitignore` 대상입니다. `git status`에 나타나면 커밋하지 말고 파일명과 ignore 규칙을 먼저 확인하세요. 자격증명이나 제어 토큰을 `.ino`, README, 스크린샷, 로그에 직접 넣지 않습니다.

자동차의 현재 운영 스케치는 다음 세 개입니다.

| 보드 | 스케치 |
| --- | --- |
| SensorUno | `firmware/uno_robot_esp01_rfid_relay/uno_robot_esp01_rfid_relay.ino` |
| MotorUno | `firmware/uno_line_tracker_motor_controller/uno_line_tracker_motor_controller.ino` |
| ActuatorUno | `firmware/uno_humidity_module_controller/uno_humidity_module_controller.ino` |

2026-08-21 공개 예제 설정의 로컬 SensorUno 빌드는 플래시 `31,006B`(96%), SRAM `1,475B`였고 CI budget은 flash `32,100B`, SRAM `1,600B`입니다. 업로드 전 CI 또는 Arduino CLI 결과를 확인하고, 기능·로그·라이브러리를 바꾼 뒤에는 SRAM과 메인 루프 지연도 다시 측정하세요.

SensorUno와 MotorUno는 4모터 절대 전진/후진에 versioned 명령 `0x11/0x12`를 사용하므로 두 보드를 같은 커밋으로 연속 업로드합니다. 일부만 업로드하면 구형 값 `1/2`가 INVALID이거나 `PROTOCOL_SYNC(7)` handshake가 실패해 주행이 잠기는 것이 정상입니다. ActuatorUno도 같은 릴리스로 맞춘 뒤 세 보드의 안전 부팅을 확인합니다.

## 8. USB 로그와 벤치 점검

USB 시리얼 로그가 필요한 경우 다른 시리얼 모니터를 닫고 실제 포트를 명시합니다.
이 스크립트도 LAN에 바인드하므로 먼저 현재 셸의 `CONTROL_API_TOKEN`을 설정해야 합니다.

```powershell
& server/start_with_usb_logs.ps1 -Port "<serial-port>"
```

한 포트는 한 프로그램만 열 수 있습니다. Arduino IDE 시리얼 모니터와 서버 브리지가 같은 포트를 동시에 사용하지 않게 하세요.

첫 하드웨어 점검은 다음 순서를 지킵니다.

1. 고전력 부하를 분리하고 차체 바퀴를 바닥에서 띄웁니다.
2. 세 Uno가 부팅 시 모터와 모든 릴레이를 끄는지 확인합니다.
3. SensorUno가 MotorUno와 ActuatorUno의 I2C 응답을 구분해 받는지 확인합니다.
4. 자동차를 HOME 마커 조건에 놓고 실제 차체가 ZONE2 방향을 향하는지 사람이 확인합니다.
5. `CALIBRATE_HOME` 완료 ACK가 확인되기 전에는 이동 명령을 보내지 않습니다.
6. 라인센서, 모터 방향, RFID 정지, 액추에이터를 각각 분리 시험합니다.

이 절차는 현재 4모터 구성의 **검증 계획**입니다. 공개된 실물 증거는 이전 2모터 벤치 기록뿐이므로, 현재 커밋은 위 1단계부터 새로 기록해야 합니다. 바닥 트랙 왕복과 고전력 부하는 [현재 한계](limitations.md)의 순서로 진행합니다.

## 문제 해결

| 증상 | 확인할 항목 |
| --- | --- |
| `/health`는 성공하고 `/ready`는 실패 | MySQL 서비스, 전용 사용자 host, 비밀번호, 데이터베이스 존재와 권한 |
| 서버 시작 시 스키마 오류 | 기존 DB 백업 후 오류에 표시된 누락 컬럼과 현재 스키마 확인 |
| LAN 장치가 서버에 접속하지 못함 | LAN 바인드, 방화벽 범위, 장치와 서버의 동일 네트워크 여부, 로컬 설정 헤더 |
| 원격 제어가 `403` | 서버의 `CONTROL_API_TOKEN`과 `Authorization: Bearer ...` 헤더 일치 여부 |
| 구역이 stale 또는 waiting | 두 활성 구역의 최근 측정 시간, `ZONE_ID`, DHT 종류와 Wi-Fi 로그 |
| USB 포트를 열 수 없음 | 다른 시리얼 모니터 종료, 실제 포트 이름, 케이블·드라이버와 보드 연결 |
| 명령은 보이지만 로봇이 움직이지 않음 | 정상적인 calibration interlock일 수 있으므로 HOME 배치와 완료 ACK부터 확인 |

API 요청 형식은 [HTTP API 문서](api.md), 각 검사의 재현 방법은 [테스트 문서](testing.md)를 참고하세요.
