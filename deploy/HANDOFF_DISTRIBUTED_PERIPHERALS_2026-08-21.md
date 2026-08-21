# 폐기된 주변기기 분배안 — 사용 금지

> 이 문서는 추적성을 위해 파일명만 보존합니다. 아래 분배안은 철회됐으며 현재
> 배선·펌웨어·업로드 지침으로 사용하면 안 됩니다.

현재 기준은 다음과 같습니다.

- SensorUno: DHT22 `DATA=D4`; ESP-01·RC522·경로·I2C master
- MotorUno: M1~M4 4WD, D9/D10 라인센서, HC-SR04 `ECHO=A0`/`TRIG=A1`, HOME interlock, watchdog
- ActuatorUno: A0/A1/D7 릴레이, LCD1602 software-I2C `SDA=D5`, `SCL=D4`; 로컬 DHT 없음
- LCD 온·습도: SensorUno가 보낸 10바이트 telemetry를 ActuatorUno가 표시
- 업로드: SensorUno·MotorUno·ActuatorUno 세 운영 스케치를 같은 커밋으로 모두 업로드

실제 인계 절차와 핀맵은
[`HANDOFF_4WD_REVERSE_2026-08-21.md`](HANDOFF_4WD_REVERSE_2026-08-21.md)를
따릅니다.
