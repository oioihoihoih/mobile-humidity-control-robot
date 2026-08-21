# SimulIDE 논리 프록시

현재 지원 회로는
[`mobile_humidity_robot_3uno_proxy.sim2`](mobile_humidity_robot_3uno_proxy.sim2)다.
이 회로는 3 Uno 프로토콜과 상태 전이를 검토하는 **논리 프록시**이며, 실제
자동차의 RF, 전력, 기구 성능을 시뮬레이션하지 않는다.

- SensorUno: I2C master, 서버·RFID·HOME 입력과 임무 조율 프록시
- MotorUno: I2C slave `0x08`, HOME interlock, 라인 입력과 M1~M4 4WD 출력 프록시
- ActuatorUno: I2C slave `0x09`, D2 DHT22, 가습·펠티어·팬 출력과 LCD 프록시
- 경로 모델: `HOME → ZONE2 → ZONE99`, 복귀는 역순

실행 순서와 버튼 계약은
[`THREE_UNO_PROXY.md`](THREE_UNO_PROXY.md)에 정리되어 있다. 회로가 세 HEX를
상대 경로로 읽으므로 `.sim2` 하나만 복사하지 말고 `simulide` 폴더 구조를
유지한다.

## 프록시가 표현하는 것

| 하드웨어/서비스 계약 | SimulIDE 표현 |
| --- | --- |
| 서버 명령과 복귀 명령 | 버튼 입력 |
| ZONE2/ZONE99 태그 판정 결과 | RFID 이벤트 버튼; 실제 UID 없음 |
| HOME 마커와 보정 | HOME 버튼 + 두 IR 입력 |
| M1/M2 기존 모터 + M3/M4 N20 1:298 | 네 채널별 FORWARD/REVERSE LED 8개 |
| 자동차 DHT22 | ActuatorUno D2에 연결한 SimulIDE `Dht22` 모델 |
| active-low 릴레이와 고전력 부하 | 가습·펠티어·팬 LED |
| LCD1602 I2C 백팩 | `I2CToParallel` + `Hd44780` 16×2 |

DHT22는 운영 역할과 같이 ActuatorUno D2에서 직접 읽고 LCD 첫 줄에 표시한다.
SensorUno의 10바이트 표시 프레임은 상태·구역·flags를 전달하며 기존 온·습도
4바이트는 0인 예약 필드로 유지한다. ESP-01 AT 펌웨어, 실제 HTTP,
RC522 무선 판독, 모터 전류·토크·제동 거리, 릴레이 접점과 펠티어 열 거동은
검증 범위 밖이다. LED가 켜지는 것은 제어 신호가 해당 상태가 됐다는 뜻일 뿐,
실제 부하가 동작했다는 증거가 아니다.

Motor 프록시에서 명령 `0x11`은 M1~M4 전진, 명령 `0x12`는 차체를 돌리지 않는
M1~M4 직선 후진(`REVERSE_HOME`)이다. N20 축 뒤쪽의 HC-SR04는 실물에서
MotorUno `ECHO=D2`, `TRIG=A1`에 연결되고 후진 중 로컬 정지를 담당하지만,
이 회로에는 거리 모델이 없다. 프록시의 D2~D8/D11은 AFMotor 채널 방향을
보여주는 시뮬레이션 전용 LED이므로 실물 핀맵이 아니다. 따라서 15cm 미만·
`STUCK_HIGH` 정지와 18cm 이상 3회 재개는 프록시 검증 결과에 포함하지 않는다.
Motor 프록시는 대신 command 7/status 8 세대 handshake를 재현해
구형 회전 의미가 섞이면 HOME 보정이 열리지 않는 계약을 확인한다.

## 실행 전 검증

저장소 루트에서 다음을 실행한다.

```powershell
python simulide\validate_sim2.py
```

검사기는 회로 XML과 핀 연결뿐 아니라
[`firmware/build-manifest.json`](firmware/build-manifest.json)을 읽어 각 프록시의
소스와 HEX SHA-256을 대조한다. 성공 출력에 아래 문장이 있어야 한다.

```text
all three firmware HEX/source hashes match the build manifest
```

SHA 불일치는 다음 중 하나를 뜻한다.

- 스케치가 HEX 생성 이후 변경됨
- HEX가 다른 소스나 보드 설정으로 빌드됨
- manifest가 현재 artifact와 함께 갱신되지 않음

이 경우 SimulIDE 화면 결과를 유효한 검증으로 기록하지 않는다. Arduino Uno
대상으로 세 프록시를 다시 컴파일하고, 생성된 HEX와 소스의 SHA-256을 계산해
`build-manifest.json`의 경로·해시를 같은 변경에서 갱신한 뒤 검사기를 다시
실행한다. manifest의 `target`도 실제 빌드 FQBN과 일치해야 한다.

PowerShell에서 manifest 값과 별도로 파일 해시를 확인하려면 다음을 사용할 수
있다.

```powershell
Get-FileHash simulide\firmware\*\*.ino -Algorithm SHA256
Get-FileHash simulide\firmware\*\*.ino.hex -Algorithm SHA256
Get-Content simulide\firmware\build-manifest.json
```

전체 오프라인 회귀는 다음 한 명령으로 실행한다.

```powershell
python scripts\check.py
```

## 실행 요약

1. SimulIDE 2에서 지원 회로를 연다.
2. 실행 직후 모든 모터·액추에이터 LED가 OFF인지 확인한다.
3. 두 IR 입력이 HOME 조건일 때 `CALIBRATE_HOME / HOME` 입력을 준다.
4. 임무 버튼과 예상 RFID 이벤트 버튼을 순서대로 입력한다.
5. LED, LCD와 시리얼 로그에서 상태 전이만 확인한다.

보정 전 임무 입력, 잘못된 역 순서, keepalive timeout, 잘못된 Actuator
프레임은 출력 OFF 또는 SAFE_STOP으로 끝나야 한다.

## 지원하지 않는 회로

이 폴더의 다른 회로와 데모 펌웨어는 초기 아이디어 보존용이다. 현재 3 Uno ·
4WD 구조의 회귀 근거, 핀맵 또는 배선 지침으로 사용하지 않는다. 새 검증과
스크린샷은 지원 회로와 build manifest를 통과한 세 프록시만 사용한다.
