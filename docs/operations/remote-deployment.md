# 서버 안전 배포 체크리스트

이 문서는 `<SERVER_HOST>:<SERVER_PORT>`에 서버를 배포할 때 사용하는 재사용
절차다. 저장소에는 사설 IP, 실제 시리얼 포트, DB 자격 증명을 기록하지 않는다.
배포 중 차량 명령은 `ALL_STOP`에 두고, 배포 성공만으로 AUTO나 모터·릴레이
시험을 시작하지 않는다.

## 배포 전 값

운영 담당자가 로컬 보안 채널에서 다음 값을 확인한다.

```text
SERVER_HOST=<SERVER_HOST>
SERVER_PORT=<SERVER_PORT>
SERIAL_PORT=<SERIAL_PORT>       # network-only면 사용하지 않음
RELEASE_COMMIT=<GIT_COMMIT>
EXPECTED_BUILD=<SERVER_BUILD_ID>
```

`MYSQL_HOST`, `MYSQL_PORT`, `MYSQL_USER`, `MYSQL_PASSWORD`,
`MYSQL_DATABASE`는 기존 서버 환경에서 가져온다. 화면, 채팅, 명령 로그에 값을
출력하지 않는다.

## 배포 artifact 무결성

권위 기준은 Git commit의 `server/` 소스다. 배포 ZIP은 로컬에서 생성하고 Git에
커밋하지 않는다. GitHub Release 또는 안전한 전달 채널에는 ZIP과 SHA-256 파일을
함께 제공한다.

```powershell
$archive = ".\mobile-humidity-robot-server-<release>.zip"
$checksum = "$archive.sha256"
$expected = ((Get-Content -Raw $checksum).Trim() -split '\s+')[0].ToLower()
$actual = (Get-FileHash -LiteralPath $archive -Algorithm SHA256).Hash.ToLower()
if ($actual -ne $expected) { throw "deployment archive SHA-256 mismatch" }
```

checksum이 없거나 일치하지 않으면 압축을 해제하거나 배포하지 않는다. checksum은
전송 오류와 artifact 혼합을 잡는 장치이며, 신뢰할 수 없는 출처를 신뢰하게 만드는
서명은 아니다. release commit도 별도로 확인한다.

SimulIDE의 `simulide/firmware/build-manifest.json`은 프록시 source/HEX용이며
서버 ZIP checksum을 대신하지 않는다.

## 사전 검사

1. 읽기 전용 대시보드 API에서 effective command가 `ALL_STOP`이고 기존 서버가
   정상 응답하는지 확인한다. 전달 상태를 바꾸는 명령 endpoint는 열지 않는다.
2. 운영 DB와 기존 `server.py`, `dashboard.html`, `system_logic.html`, 실행
   환경을 별도 백업 경로에 복사한다.
3. 검증한 ZIP을 새 임시 폴더에 해제한다. 운영 폴더에 바로 덮어쓰지 않는다.
4. 임시 폴더에서 다음을 실행한다.

   ```powershell
   python -m pip install -r .\requirements.txt
   python -m py_compile .\server.py
   python -m unittest .\test_server_logic.py -q
   ```

5. 테스트가 실패하면 운영 프로세스를 변경하지 않고 중단한다.

## 교체 순서

1. 기존 Python 프로세스 중 이 프로젝트의 `server.py` PID만 정상 종료한다.
   모든 `python.exe`를 일괄 종료하지 않는다.
2. 검증한 서버 파일을 운영 폴더에 복사한다. DB, 로그, 환경 파일은 삭제하지
   않는다.
3. USB 게이트웨이가 필요하면 환경의 `<SERIAL_PORT>`로 실행한다.

   ```powershell
   .\start_with_usb_logs.ps1 -Port <SERIAL_PORT>
   ```

   USB 게이트웨이가 필요 없으면 다음을 사용한다.

   ```powershell
   .\start_network_only.ps1
   ```

4. 서버의 bind 주소와 방화벽이 배포 환경 정책에 맞는지 확인한다. 공개 인터넷에
   직접 노출하지 않는다.

## 승인 기준

로컬 포트는 실제 `ROBOT_PORT` 환경 변수에서 읽는다.

```powershell
$port = if ($env:ROBOT_PORT) { $env:ROBOT_PORT } else { "8000" }
$health = Invoke-RestMethod "http://localhost:$port/api/health"
$ready = Invoke-RestMethod "http://localhost:$port/api/ready"
$dashboard = Invoke-RestMethod "http://localhost:$port/api/dashboard"
$health
$ready
$dashboard.effective_command
```

다음을 모두 확인할 때만 서버 배포를 성공으로 기록한다.

- health, ready와 dashboard가 HTTP 200
- health의 `build`가 release에 기록한 `<SERVER_BUILD_ID>`와 일치
- ready 응답의 `ready`가 `true`
- effective command가 `ALL_STOP / HOME / NONE`
- ZONE2와 ZONE99가 존재하되, stale/missing 데이터가 임의 정상값으로 채워지지 않음
- 재시작 뒤 manual ALL_STOP latch와 revision 유지
- 승인된 관리 PC에서 `<SERVER_HOST>:<SERVER_PORT>` 접근 가능
- traceback, schema migration 오류, 자격 증명 노출이 없음

센서가 stale이어서 AUTO 대신 ALL_STOP인 것은 정상 안전 동작이다. 검증을 위해
가짜 측정값을 삽입하거나 임계값을 바꾸지 않는다.

## 롤백

1. 새 `server.py` 프로세스만 종료한다.
2. 백업한 운영 파일과 환경을 복원한다.
3. 기존 실행 방법으로 서버를 시작한다.
4. health/dashboard에서 DB 연결과 ALL_STOP을 확인한다.
5. 실패한 명령, traceback, health 응답, release commit만 보고한다. 자격 증명과
   내부 주소는 가린다.

## 배포 후 기록

- release commit과 build ID
- ZIP SHA-256과 checksum 출처
- 실행 모드(network-only 또는 USB gateway)
- 사용한 시리얼 포트는 공개 문서가 아닌 운영 기록에만 저장
- ZONE2/ZONE99 freshness, effective command와 revision
- 서버 PID와 시작 시간

차량 보정·주행·RFID·고전력 부하 시험은 별도 승인된 하드웨어 절차로 수행한다.
