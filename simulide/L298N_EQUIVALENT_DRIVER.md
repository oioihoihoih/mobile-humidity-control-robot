# L298N 등가 듀얼 H-브리지 구성

> **레거시 회로 전용:** 이 MOSFET H-브리지는 초기 단일 Uno·4WD 파일의 구성입니다. 현재 3 Uno 회로는 MotorUno I2C `0x08`의 M1~M4 네 채널을 채널별 FORWARD/REVERSE LED 8개로 검증하며, 아래 배선을 사용하지 않습니다. 명령 2도 제자리 회전이 아닌 네 바퀴 직선 후진입니다.

`mobile_humidity_robot_4wd_full_system.sim2`의 모터 드라이버 영역은 SimulIDE에 L298N 전용 모델이 없는 점을 보완한 **L298N 기능 등가 회로**입니다. Uno가 모터에 직접 연결되지 않습니다.

```text
Uno D3 (LEFT_IN1) ─┬─ N-MOSFET: left OUT2 low
                   └─ inverter ─ P-MOSFET: left OUT1 high
Uno D4 (LEFT_IN2) ─┬─ N-MOSFET: left OUT1 low
                   └─ inverter ─ P-MOSFET: left OUT2 high

Uno D5/D6도 위와 같은 방식으로 right OUT1/OUT2를 제어

5 V motor rail ──> P-MOSFET high-side ─┐
                                         ├─ OUT1/OUT2 ─> 좌 2개 또는 우 2개 모터
GND ────────────> N-MOSFET low-side ───┘
```

## 동작표

| 채널 입력 | OUT1 | OUT2 | 모터 상태 |
| --- | --- | --- | --- |
| IN1=0, IN2=0 | 차단 | 차단 | 정지(코스트) |
| IN1=1, IN2=0 | 5V | GND | 정방향 |
| IN1=0, IN2=1 | GND | 5V | 역방향 |
| IN1=1, IN2=1 | 금지 | 금지 | 회로 보호를 위해 펌웨어가 사용하지 않음 |

좌측 채널의 OUT1/OUT2는 앞·뒤 좌측 모터 2개에, 우측 채널의 OUT1/OUT2는 앞·뒤 우측 모터 2개에 연결되어 있습니다. 따라서 4WD 차체에서 좌·우를 독립 제어할 수 있습니다.

## 실물 L298N 배선으로 바꿀 때

시뮬레이션 내부 MOSFET 8개와 버퍼 4개를 실제 L298N 1개로 대체할 수 있습니다.

| Uno | L298N |
| --- | --- |
| D3 | IN1 (좌측 정방향) |
| D4 | IN2 (좌측 역방향) |
| D5 | IN3 (우측 정방향) |
| D6 | IN4 (우측 역방향) |
| 5V 로직 | 5V / GND |
| Li-Po→DC-DC 모터 레일 | `+12V` 모터전원 / GND |

ENA·ENB는 처음에는 점퍼로 HIGH에 두고, 속도 제어가 필요하면 PWM 핀을 추가합니다. 모터 4개를 채널당 2개씩 병렬 연결할 경우 기동전류 합계가 L298N 정격을 넘을 수 있습니다. 실제 시연에는 채널당 충분한 전류 정격이 있는 드라이버를 쓰거나 TB6612FNG 두 개를 사용하는 편이 안전합니다.

이 등가 회로의 5V `Fixed Voltage`는 **모터용 Li-Po + DC-DC 출력의 시뮬레이션 대체**입니다. Uno 전원과 고전력 펠티어 전원까지 실제로 공급하는 회로는 아직 별도 구현이 필요합니다.
