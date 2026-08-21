# Deployment artifacts

배포용 압축 파일은 로컬에서 생성하며 Git에는 커밋하지 않습니다. 서버의 기준
소스는 [`../server`](../server)이고, 절차는
[`../docs/operations/remote-deployment.md`](../docs/operations/remote-deployment.md)를
따릅니다.

## 릴리스 파일

배포 시점의 소스로 `mobile-humidity-server-<VERSION>.zip`을 만들고 같은 위치에
SHA-256 파일 `mobile-humidity-server-<VERSION>.zip.sha256`을 함께 둡니다. 두
파일은 GitHub Release 같은 변경 불가능한 릴리스 저장소로 전달합니다. 생성된
압축 파일을 저장소에 두지 않으면 오래된 바이너리가 현재 소스로 오인되는 일을
막을 수 있습니다.

Windows PowerShell에서 수신 파일을 검증하는 예시는 다음과 같습니다.

```powershell
$expected = (Get-Content .\mobile-humidity-server-<VERSION>.zip.sha256).Split()[0]
$actual = (Get-FileHash .\mobile-humidity-server-<VERSION>.zip -Algorithm SHA256).Hash
if ($actual -ne $expected) { throw "release archive SHA-256 mismatch" }
```

일치하지 않으면 압축을 풀거나 실행하지 말고 다시 전달받습니다. 서버 주소,
직렬 포트, Wi-Fi 및 DB 자격 증명은 압축 파일에 하드코딩하지 않고 대상 환경의
환경 변수로 설정합니다.

## 펌웨어 프록시 산출물

서버 릴리스 압축 파일의 SHA-256과 SimulIDE 펌웨어 검증은 서로 다른 절차입니다.
SimulIDE 쪽은 [`../simulide/firmware/build-manifest.json`](../simulide/firmware/build-manifest.json)에
기록된 각 Uno 소스와 HEX의 SHA-256을
`python simulide\validate_sim2.py`로 확인합니다. 자세한 의미와 제한은
[`../simulide/README.md`](../simulide/README.md)를 참고합니다.
