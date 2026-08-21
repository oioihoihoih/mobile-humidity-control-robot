# SimulIDE 실행 검증 기록

> **레거시 검증 기록:** 아래 결과는 초기 단일 Uno/`ZONE1 + ZONE2` 파일에만 해당합니다. 최신 3 Uno 회로의 실행·검증 방법은 [THREE_UNO_PROXY.md](THREE_UNO_PROXY.md)를 참고합니다.

검증 일시: 2026-08-14  
검증 파일: `mobile_humidity_robot_4wd_full_system.sim2`

## 실제 실행 결과

SimulIDE에서 최종 회로를 다시 불러온 뒤 전원을 켜서 아래의 순서로 확인했다.

1. 최종 로드 구간에서 커넥터의 `null endpin` 오류 없이 회로와 Uno 펌웨어가 정상 로드되었다.
2. 시리얼 모니터에 다음 메시지가 실제로 출력되었다.

   ```text
   Mobile humidity robot - 4 wheel simulation ready
   H1=72.0 H2=35.0 target=ZONE1 current=HOME
   MODE=MOVE target=ZONE1
   H1=72.0 H2=35.0 target=ZONE1 current=ZONE1
   MODE=DEHUMIDIFY target=ZONE1
   ```

3. Zone 1의 편차는 `+12 %RH`(72 - 60), Zone 2의 편차는 `-5 %RH`(40 - 35)이다. 따라서 우선순위 로직이 더 심각한 Zone 1을 목표로 고른 것을 확인했다.
4. 6초의 시뮬레이션 이동 프록시 뒤에는 `current=ZONE1`, `MODE=DEHUMIDIFY`가 표시됐다. 제습 릴레이 LED와 펠티어 방열 팬도 활성 상태로 바뀌는 것을 회로 화면에서 확인했다.
5. 회로 화면에서 좌·우 각 2개씩의 DC 모터가 2x2로 분리되어 표시되고, 각각은 두 개의 L298N-등가 H-브리지 출력에 병렬 연결되어 있다.
6. 회로 상에서 Uno D3~D6은 모터에 직접 연결되지 않고 MOSFET 게이트 입력만 제어한다. 모터 전류는 별도의 5 V 모터 레일에서 공급된다.

## 실행 중 발견·수정한 문제

| 문제 | 원인 | 수정 |
| --- | --- | --- |
| 회로 로드 때 다수의 `null endpin` 오류 | 여러 `Node` 항목이 한 XML 줄에 있어 SimulIDE 파서가 첫 항목만 읽음 | 각 `Node`를 독립된 XML 줄로 분리 |
| DHT/부하 일부가 연결되지 않음 | Uno A4에서 LED와 팬으로 직접 분기 | `Node-150` 분기 노드를 추가해 한 핀 연결을 분리 |
| 4개 모터가 겹쳐 보여 차체 구성이 불명확 | 모터 심볼 좌표 중첩 | 전·후, 좌·우 2x2 위치로 분리하고 배선을 연동 이동 |
| RFID 도착 버튼이 시연 중 놓침 | SimulIDE `Push` 입력은 매우 짧은 순간만 LOW | 물리 버튼 배선은 유지하고, 시뮬레이션에서는 6초 이동 프록시와 시리얼 명령을 추가 |

## 시연 시 다음 확인

- 기본 조건에서는 Zone 1을 향해 이동한 뒤 약 6초 후 `DEHUMIDIFY`까지 자동 진행한다.
- 물리 도착 신호와 같은 입력은 RFID proxy 버튼(A0~A2)으로도 연결되어 있다. 펌웨어는 시리얼 명령 `h`·`1`·`2`도 HOME·ZONE1·ZONE2 도착 신호로 처리한다.
- Zone 1 또는 Zone 2의 DHT22 습도를 40~60 %RH로 바꾸면 다음 2.5초 센싱 주기에 정상 상태와 복귀 목표가 재계산된다.
- 반대 구역의 습도를 더 심각하게 바꾸면 다음 2.5초 센싱 주기에 `target`이 해당 구역으로 재계산된다.

> 이 `.sim2`는 실제 부품의 Wi-Fi/서버/RFID/펠티어를 SimulIDE에서 실행 가능한 DHT22·버튼·LED·팬·H-브리지 대체 모델로 표현한 MVP다. 실제 제작 시에는 ESP32 통신과 고전력 부하 회로를 별도 검증한다.
