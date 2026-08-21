# HTTP API

Python 서버는 대시보드, 센서 입력, 자동차 명령·ACK와 제한된 수동 제어 API를 제공합니다. 기본 포트는 `8000`이며 아래 예시는 로컬 서버 `http://127.0.0.1:8000`을 사용합니다.

## 보안 경계

이 API는 **격리되거나 신뢰할 수 있는 LAN의 교육용 프로토타입**을 전제로 합니다.

- TLS가 없고 센서·자동차 장치 인증, 요청 서명, nonce와 rate limit이 없습니다.
- 센서 입력과 로봇 상태를 같은 LAN의 다른 클라이언트가 위조할 수 있습니다.
- 인터넷에 직접 노출하거나 포트 포워딩하지 마세요.
- 위험한 제어 API는 loopback 요청만 기본 허용합니다.
- LAN에서 제어 API가 필요하면 서버에 `CONTROL_API_TOKEN`을 설정하고 `Authorization: Bearer <CONTROL_TOKEN>` 헤더를 보냅니다.
- Bearer 토큰은 URL, 로그, README와 펌웨어 공개 예제에 넣지 않습니다.

`CONTROL_API_TOKEN`은 아래 “제어 API”만 보호합니다. 센서·상태 API까지 인증하는 전체 보안 계층이 아니므로 신뢰 LAN 요구사항은 그대로 유지됩니다.

## 엔드포인트 요약

| 메서드 | 경로 | 용도 | LAN 제어 토큰 |
| --- | --- | --- | --- |
| GET | `/` | 웹 대시보드 | 불필요 |
| GET | `/logic` | 시스템 로직 HTML | 불필요 |
| GET | `/health`, `/api/health` | 프로세스·빌드 liveness | 불필요 |
| GET | `/ready`, `/api/ready` | MySQL readiness | 불필요 |
| GET | `/api/dashboard` | 대시보드 전체 JSON | 불필요 |
| POST | `/api/readings` | 현재 형식의 구역 측정 입력 | 불필요 |
| GET·POST | `/api/humidity` | 이전 센서 형식 호환 | 불필요 |
| POST | `/api/events` | 외부 모듈 이벤트·장치 상태 입력 | 불필요 |
| GET | `/api/robot/command` | SensorUno의 현재 명령 폴링 | 불필요 |
| GET | `/api/robot/status` | 자동차 heartbeat·이벤트·명령 ACK | 불필요 |
| GET | `/api/gateway/heartbeat` | 선택형 게이트웨이 heartbeat | 불필요 |
| POST | `/api/control` | AUTO·수동 명령 | 원격 요청에 필요 |
| POST | `/api/settings` | 온·습도 임계값 변경 | 원격 요청에 필요 |
| POST | `/api/serial/reconnect` | USB 시리얼 재연결 | 원격 요청에 필요 |
| POST | `/api/serial/test-rfid` | 합성 RFID 시험 이벤트 | 원격 요청에 필요 |

## 상태 확인

### `GET /health`

DB를 조회하지 않는 liveness입니다.

```json
{
  "ok": true,
  "build": "<build-id>",
  "server_time": 0,
  "features": ["humidity-hysteresis", "stale-all-stop"]
}
```

HTTP `200`이어도 DB가 준비됐다는 뜻은 아닙니다.

### `GET /ready`

설정된 MySQL 데이터베이스에 실제 연결해 간단한 질의를 수행합니다.

준비됨:

```json
{
  "ok": true,
  "build": "<build-id>",
  "database": "ready",
  "error": null,
  "server_time": 0
}
```

준비되지 않음은 HTTP `503`과 `database: unavailable`로 반환됩니다. `error`에는 비밀번호가 아니라 오류 유형만 표시됩니다.

## 구역 측정

### `POST /api/readings`

요청:

```json
{
  "zone_id": "ZONE2",
  "temperature": 24.1,
  "humidity": 55.0
}
```

현재 자동 임무에 참여하는 구역은 `ZONE2`, `ZONE99`입니다.

- `zone_id`: 대문자로 정규화되며 현재 활성 구역이어야 함
- `temperature`: 숫자, `-40..100`
- `humidity`: 숫자, `0..100`

성공 응답에는 정규화한 측정, 판정 상태와 갱신된 자동 임무가 포함됩니다.

```json
{
  "accepted": true,
  "zone_id": "ZONE2",
  "temperature": 24.1,
  "humidity": 55.0,
  "state": "ERROR_LOW",
  "normal": false,
  "action": "HUMIDIFY",
  "message": "<classification-message>",
  "mission": {
    "revision": 123,
    "command": "TASK",
    "target_zone": "ZONE2",
    "action": "HUMIDIFY"
  }
}
```

### `GET·POST /api/humidity`

이전 센서 코드와의 호환 경로입니다. POST는 `zone` 필드를 `zone_id`처럼 받아 같은 DB에 저장하고, GET은 현재 대시보드 JSON을 반환합니다.

```json
{
  "zone": "ZONE2",
  "temperature": 24.1,
  "humidity": 55.0
}
```

새 펌웨어는 `/api/readings`를 사용하세요. 선택형 legacy listener를 켜더라도 dashboard·control 경로는 노출하지 않고 humidity·health·ready 호환 경로만 제공합니다.

## 외부 이벤트

### `POST /api/events`

요청:

```json
{
  "device_id": "AUX_MODULE_1",
  "device_type": "SENSOR_AUX",
  "source": "AUX_MODULE_1",
  "event_type": "MODULE_READY",
  "message": "Auxiliary module ready",
  "data": {
    "channel_count": 2
  }
}
```

`message`는 비어 있지 않아야 합니다. `source`와 `event_type`을 생략하면 서버의 일반 외부 모듈 기본값이 사용되지만, 추적 가능한 장치 이벤트를 위해 명시하는 것을 권장합니다. `device_id`, `device_type`, `data`는 장치 목록과 부가 정보에 사용됩니다. 이벤트 본문에 비밀번호, 토큰, 전체 네트워크 응답이나 개인 정보를 넣지 마세요.

성공:

```json
{"accepted": true}
```

## 자동차 명령과 ACK

### `GET /api/robot/command`

SensorUno가 주기적으로 폴링합니다. wire 응답은 작은 AVR 파서와 호환되도록 정확히 네 필드만 가집니다.

```json
{
  "revision": 123,
  "command": "TASK",
  "target_zone": "ZONE2",
  "action": "HUMIDIFY"
}
```

서버는 이 응답을 실제로 제공한 시점을 delivered revision으로 기록합니다. 주요 자동 명령은 `TASK`, `ALL_STOP`, `RETURN_HOME`입니다. 수동 모드에서는 아래 제어 명령이 같은 네 필드 계약으로 전달됩니다.

### `GET /api/robot/status`

현재 펌웨어 계약 때문에 상태 보고는 query parameter를 사용하는 GET입니다. 이 요청은 heartbeat·이벤트와 ACK 상태를 변경하므로 단순 캐시 가능한 조회로 취급하면 안 됩니다.

heartbeat 예시:

```text
/api/robot/status?phase=MOVING&event=HEARTBEAT&zone=ZONE2&action=HUMIDIFY
```

명령 ACK 예시:

```text
/api/robot/status?phase=TASK_COMPLETE&event=MODULE_COMPLETE&zone=ZONE2&action=HUMIDIFY&ack_revision=123&result=COMPLETED
```

허용 phase:

- `UNKNOWN`
- `IDLE`
- `MOVING`
- `WAITING_RFID`
- `MODULE_RUNNING`
- `TASK_COMPLETE`
- `RETURNING`

허용 ACK result:

- 진행: `ACK`, `EXECUTING`
- 성공: `COMPLETED`
- 실패: `FAILED`, `I2C_ERROR`, `INVALID_ACTION`, `ACT_START_ERROR`, `IGNORED`

ACK는 다음 조건을 모두 만족해야 수용됩니다.

1. 서버가 해당 revision을 자동차에 실제 전달했음
2. 지금도 유효한 명령 revision과 같음
3. revision이 AVR signed long 범위를 넘지 않음
4. result가 허용 목록에 있음

늦거나 임의로 만든 ACK는 상태 이벤트로 기록될 수 있지만 현재 명령의 성공으로 저장되지 않습니다.

성공 응답 예시:

```json
{
  "accepted": true,
  "phase": "TASK_COMPLETE",
  "event": "MODULE_COMPLETE",
  "ack_revision": 123,
  "result": "COMPLETED",
  "ack_accepted": true,
  "ack_rejection": null
}
```

자동 `TASK`의 완료 ACK가 수용되면 서버는 해당 구역의 현재 측정을 소비 처리하고 새 revision의 안전 정지를 저장합니다. 완료 뒤 도착한 새 측정만 다음 임무를 다시 열 수 있습니다.

## 대시보드 데이터

### `GET /api/dashboard`

다음 묶음을 한 번에 반환합니다.

- `thresholds`: 현재 온·습도 범위와 호환용 `low`, `high`
- `zones`: 활성 구역 최신값, fresh/stale, 온도·습도 판정
- `mission`: 자동 임무와 후보·대기 개수
- `history`: 최근 측정
- `events`: 최근 이벤트
- `devices`: 센서·로봇·게이트웨이 온라인 표시
- `serial`: 선택형 USB 브리지 상태
- `robot_network`: 자동차 heartbeat·ACK 상태
- `manual_control`: 현재 수동 제어 상태
- `effective_command`: AUTO 또는 MANUAL 중 실제 명령
- `command_delivery`: 등록·전달·실행·완료·실패·새 측정 대기 상태

이 응답은 관제 UI용이며 작은 마이크로컨트롤러가 파싱할 계약이 아닙니다. SensorUno는 `/api/robot/command`의 네 필드 응답만 사용합니다.

## 제어 API

다음 예시는 JSON 본문에 더해 LAN 요청일 때 아래 헤더가 필요합니다.

```text
Authorization: Bearer <CONTROL_TOKEN>
Content-Type: application/json
```

loopback 요청은 토큰 없이 허용되지만, 실수 방지를 위해 수동 제어 전에 바퀴를 띄우고 고전력 부하를 분리하세요.

### `POST /api/control`

HOME 보정:

```json
{
  "mode": "MANUAL",
  "command": "CALIBRATE_HOME"
}
```

수동 구역 임무:

```json
{
  "mode": "MANUAL",
  "command": "TASK",
  "target_zone": "ZONE2",
  "action": "HUMIDIFY"
}
```

즉시 전체 정지:

```json
{
  "mode": "MANUAL",
  "command": "ALL_STOP"
}
```

자동 모드 복귀:

```json
{
  "mode": "AUTO"
}
```

`ALL_STOP` latch 뒤 자동 모드로 돌아갈 때는 사용자 확인을 명시합니다.

```json
{
  "mode": "AUTO",
  "confirm_all_stop": true
}
```

지원하는 수동 command:

| command | 용도 |
| --- | --- |
| `TASK` | 지정 구역의 `HUMIDIFY`, `DEHUMIDIFY` 또는 이동만 수행 |
| `CALIBRATE_HOME` | HOME 정지 배치와 outbound 방향 동기화 |
| `MOTOR_FWD`, `MOTOR_RETURN`, `MOTOR_STOP` | 네 바퀴 수동 전진, 현재 위치에서 HOME까지의 경로 기반 직선 후진 복귀, 정지 |
| `ALL_STOP` | latch되는 전체 안전 정지 |
| `ACT_HUMIDIFY`, `ACT_DEHUMID`, `ACT_STOP` | 무부하 액추에이터 진단 |
| `RFID_TEST` | 실제 카드가 아닌 합성 도착 이벤트 시험 |
| `I2C_CHECK` | 3-Uno I2C 상태 확인 |

현재 수동 명령의 ACK가 오기 전에는 `ALL_STOP` 외 명령으로 덮어쓸 수 없습니다. 수동 가습·제습이 완료되면 서버는 같은 revision의 반복 가동을 막기 위해 같은 구역의 `TASK / NONE`으로 전환합니다.

`MOTOR_RETURN`은 임의 벤치 역회전 명령이 아닙니다. HOME 보정, 알려진 현재 역과 정상 RFID 상태가 필요합니다. `MOTOR_FWD`는 RFID 경로를 무시하므로 실행 즉시 위치를 `UNKNOWN`으로 잠그며, 그 뒤 복귀하려면 차를 HOME에 회수해 `CALIBRATE_HOME`을 다시 실행해야 합니다.

### `POST /api/settings`

네 값을 항상 함께 보냅니다.

```json
{
  "temperature_min": 18.0,
  "temperature_max": 28.0,
  "humidity_min": 60.0,
  "humidity_max": 80.0
}
```

- 온도 범위: `-40..100`, min < max
- 습도 범위: `0..100`, min < max

DB commit이 성공한 뒤에만 런타임 판정값을 바꿉니다.

### `POST /api/serial/reconnect`

```json
{"port": "<serial-port>"}
```

빈 본문은 자동 포트 탐색을 요청합니다. 실제 포트 이름은 문서나 커밋에 고정하지 마세요.

### `POST /api/serial/test-rfid`

```json
{}
```

합성 시험 이벤트이며 실제 카드 판독, 위치와 제동거리의 증거가 아닙니다.

## 오류 응답

| HTTP 상태 | 의미 |
| --- | --- |
| `400` | 필수 필드 누락, 범위·형식 오류, 허용되지 않은 명령 또는 ACK |
| `403` | 원격 제어 토큰 없음 또는 불일치 |
| `404` | 지원하지 않는 경로 |
| `413` | 요청 본문이 설정된 최대 크기를 초과함 |
| `503` | `/ready` 또는 DB가 필요한 API에서 MySQL 사용 불가 |

JSON 오류는 보통 `{"error":"..."}` 형식입니다. DB 장애 응답은 내부 접속 정보를 숨긴 `{"error":"database unavailable","retryable":true}` 형식을 사용합니다. 최대 본문 크기와 연결 timeout은 환경 변수로 조정할 수 있지만, 값을 키우기 전에 메모리·DoS 위험과 신뢰 LAN 경계를 검토하세요.
