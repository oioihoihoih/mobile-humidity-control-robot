# 서버 배포 인수인계 템플릿

이 문서는 배포 담당자에게 전달할 최소 정보와 금지 사항을 정의한다. 특정 서버의
IP, 시리얼 포트, 장치 UID, 현재 센서값과 자격 증명은 저장소에 기록하지 않는다.
환경별 값은 승인된 운영 기록에서 `<PLACEHOLDER>`를 채운다.

## 임무

검증된 release를 `<SERVER_HOST>:<SERVER_PORT>`에 배포한다. 기존 Arduino wire
JSON의 `revision`, `command`, `target_zone`, `action` 계약을 보존한다. 배포 중
차량은 `ALL_STOP`을 유지하며 AUTO, `CALIBRATE_HOME`, TASK, 모터·릴레이
시험을 실행하지 않는다.

## 전달 항목

- release commit과 서버 build ID
- `server/` 기반 배포 archive
- archive와 별도 채널로 제공된 SHA-256 checksum
- [`remote-deployment.md`](remote-deployment.md)
- Python 버전과 `requirements.txt`
- 실행 모드: network-only 또는 USB gateway
- 롤백용 기존 서버 백업 위치

생성 archive, 로그, DB dump, 캐시, 화면 캡처와 실제 secrets는 Git에 넣지 않는다.
운영 데이터는 기존 MySQL 인스턴스를 보존하며, schema migration 전에 백업한다.

## 인수인계 값

```text
SERVER_HOST=<SERVER_HOST>
SERVER_PORT=<SERVER_PORT>
SERIAL_PORT=<SERIAL_PORT_OR_DISABLED>
RELEASE_COMMIT=<GIT_COMMIT>
EXPECTED_BUILD=<SERVER_BUILD_ID>
ARCHIVE_SHA256=<SHA256>
BACKUP_PATH=<BACKUP_PATH>
```

MySQL과 Wi-Fi 자격 증명은 값이 아니라 **어디에서 안전하게 주입되는지**만
전달한다. RFID는 `ZONE2=<ZONE2_TAG_UID>`, `ZONE99=<ZONE99_TAG_UID>`와 같은
자리표시자로만 언급하고 실제 UID는 공개 인수인계 문서에 복사하지 않는다.

## 배포 전 읽기 전용 확인

다음을 확인하되 상태를 변경하는 endpoint를 호출하지 않는다.

- health/dashboard 응답 여부
- effective command가 ALL_STOP인지
- DB 연결 상태
- ZONE2/ZONE99 freshness
- 현재 실행 모드와 프로젝트 서버 PID

명령 전달 상태를 갱신할 수 있는 robot command endpoint는 진단 목적으로 직접
열지 않는다.

## artifact 검증

담당자는 archive를 운영 폴더 밖 임시 디렉터리에 내려받고 checksum을 먼저
검증한다.

```powershell
$archive = ".\mobile-humidity-robot-server-<release>.zip"
$checksum = "$archive.sha256"
$expected = ((Get-Content -Raw $checksum).Trim() -split '\s+')[0].ToLower()
$actual = (Get-FileHash -LiteralPath $archive -Algorithm SHA256).Hash.ToLower()
if ($actual -ne $expected) { throw "deployment archive SHA-256 mismatch" }
```

checksum 불일치, 누락, release commit 불명확 중 하나라도 있으면 배포를 중단한다.
압축 해제 후에는 다음 검사를 실행한다.

```powershell
python -m pip install -r .\requirements.txt
python -m py_compile .\server.py
python -m unittest .\test_server_logic.py -q
```

SimulIDE 프록시를 함께 검토하는 경우 저장소 루트에서 다음을 실행한다.

```powershell
python simulide\validate_sim2.py
```

이 명령은 `simulide/firmware/build-manifest.json`에 기록된 source/HEX SHA-256을
실제 파일과 대조한다. 이 manifest는 서버 archive checksum과 목적이 다르다.

## 배포 제한

- 검증 전 운영 폴더에 덮어쓰지 않는다.
- 모든 Python 프로세스를 일괄 종료하지 않는다.
- DB, 로그, 로컬 환경 파일을 삭제하지 않는다.
- 테스트를 위해 센서값·임계값·manual mode를 바꾸지 않는다.
- 배포 완료만으로 차량을 보정하거나 움직이지 않는다.
- 내부 주소, 실제 COM 번호, 실제 UID, 비밀번호를 이슈나 공개 로그에 남기지
  않는다.

## 승인과 롤백

승인 기준과 롤백 순서는
[`remote-deployment.md`](remote-deployment.md)를 그대로 따른다. 배포 후 다음
값만 다음 담당자에게 전달한다.

- health build ID와 release commit
- archive SHA-256 검증 결과
- DB ready 여부
- effective command와 revision
- 구역 freshness
- 실행 모드, 서버 PID와 시작 시간
- 남은 오류의 가려진 로그

하드웨어 상태는 서버 배포 상태와 별개다. 3 Uno · 2WD 차량의 HOME 보정, 라인
주행, RFID, 액추에이터 부하는 각 단계의 OFF와 ACK를 확인하는 별도 감독 시험이
필요하다.
