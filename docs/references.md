# 문서·저장소 구성 참고 자료

이 문서는 구르미 저장소를 정리할 때 참고한 공개 자료와 적용 원칙을 기록합니다. 코드나 하드웨어 설계를 그대로 복제한 것이 아니라, 첫 화면의 정보 구조와 검증 범위 표현, 문서 분리 방식을 비교했습니다.

## GitHub 공식 기준

- [About READMEs](https://docs.github.com/en/repositories/managing-your-repositorys-settings-and-features/customizing-your-repository/about-readmes): README에는 프로젝트의 목적, 유용성, 시작 방법, 도움을 받을 위치와 관리 주체를 담고 긴 설명은 별도 문서로 분리했습니다.
- [Creating diagrams](https://docs.github.com/en/get-started/writing-on-github/working-with-advanced-formatting/creating-diagrams): GitHub에서 직접 렌더링되는 Mermaid로 구성 요소와 상태 흐름을 표현했습니다.
- [Repository best practices](https://docs.github.com/en/repositories/creating-and-managing-repositories/best-practices-for-repositories): 생성 산출물과 비밀 설정을 제외하고, 재현 가능한 소스·검사·문서를 중심으로 저장소를 구성했습니다.
- [Licensing a repository](https://docs.github.com/en/repositories/managing-your-repositorys-settings-and-features/customizing-your-repository/licensing-a-repository): 팀이 라이선스를 선택하기 전에는 임의의 `LICENSE`를 추가하지 않고 미결정 상태를 명시했습니다.
- [Arduino compile-sketches](https://github.com/arduino/compile-sketches): 세 운영 Uno 스케치의 컴파일과 메모리 보고를 CI에 추가했습니다.

## 공개 로봇 프로젝트에서 참고한 패턴

| 프로젝트 | 참고한 점 | 구르미 적용 |
| --- | --- | --- |
| [autonomous-ai/autonomous-robot](https://github.com/autonomous-ai/autonomous-robot) | 상단 시스템 연결도, 순서가 있는 온보딩, 테스트 환경 구분 | README의 단일 구성도, Quick Start, 날짜·범위가 분명한 검증 문서 |
| [Lenna Mobile Robot ONE](https://github.com/Lenna-Robotics-Research-Lab/Lenna-Mobile-Robot-ONE) | README를 진입점으로 두고 hardware/software/docs를 역할별 분리 | `firmware/`, `server/`, `simulide/`, `tests/`, `docs/` 구조와 문서 인덱스 |
| [Glitch AMR](https://github.com/GradVizor/glitch-amr) | 시스템 구성과 임무 흐름을 별도 시각화하고 실물·시뮬레이션을 구분 | Mermaid 아키텍처·상태도와 SimulIDE 프록시 한계 표기 |
| [Northeastern Capstone TRASH](https://github.com/Capstone-W3/trash_parent_repo) | 문제·PoC 범위·미완료 조건을 분리한 팀프로젝트 설명 | 벤치 검증과 실차 미검증을 첫 화면에서 구분하고 완료 기준을 별도 문서화 |
| [MIT Racecar Simulator](https://github.com/mit-racecar/racecar_simulator) | Dependencies → Install → Quick Start → API/Parameters 순서 | 설치, 서버 실행, API, 테스트 문서의 단계별 읽기 흐름 |

## 모터 실드 전기 기준

- [Adafruit Motor Shield V1 안내](https://learn.adafruit.com/adafruit-motor-shield/overview):
  AFMotor 계열 4채널 L293D 실드의 채널 수, 모터 전원 분리와 채널당 전류 범위를
  확인하는 기준으로 사용했습니다. 실제 보드가 호환 클론이면 부품 표기와 회로를
  별도로 대조해야 합니다.
- [TI L293D 데이터시트](https://www.ti.com/lit/ds/symlink/l293d.pdf):
  연속 출력전류, 피크 조건, 전압강하와 열 한계를 판단하는 권위 자료입니다.
  N20의 정지전류가 확인되지 않은 상태에서 4개 동시 바닥 기동을 허용하는 근거로
  사용하지 않습니다.

## 의도적으로 채택하지 않은 요소

- 실행 근거가 없는 장식용 배지와 고정된 과거 테스트 수치
- 시뮬레이션이나 합성 이미지를 실물 완성 증거처럼 보이게 하는 표현
- README와 HTML 시각화에 같은 세부 로직을 중복 복사하는 구성
- 실제 팀 운영 절차가 없는 상태에서의 형식적인 issue·PR 템플릿과 커뮤니티 파일
- 팀 합의 없이 선택한 라이선스, 추측한 팀원 이름·역할과 검증되지 않은 성능 수치

새 참고 자료를 추가할 때는 어떤 구조적 판단에 사용했는지 함께 기록하고, 현재 소스와 맞지 않는 예전 설계 패턴은 기준 문서로 승격하지 않습니다.
