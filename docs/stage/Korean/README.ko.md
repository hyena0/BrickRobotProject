# 단계별 계획

`PLAN.md` 5절의 각 단계를 자세히 정리한 문서입니다. 한 번에 한 단계씩 진행하고, 완료 기준을 통과하면 바로 다음 단계로 넘어갑니다.

| 단계 | 파일 | Latest |
|---|---|---|
| 0. 설정 | [stage-0-setup.ko.md](stage-0-setup.ko.md) | Day 1 |
| 1. 데이터와 도구 | [stage-1-data-tools.ko.md](stage-1-data-tools.ko.md) | Day 2 |
| 2. 에이전트 v1: 링크와 관절 | [stage-2-links-joints.ko.md](stage-2-links-joints.ko.md) | Day 3 (판단 1) |
| 3. 기어와 연동 | [stage-3-gears-coupling.ko.md](stage-3-gears-coupling.ko.md) | Day 4 |
| 4. 기구학 | [stage-4-kinematics.ko.md](stage-4-kinematics.ko.md) | Day 5 (판단 2) |
| 5. 실물 | [stage-5-physical.ko.md](stage-5-physical.ko.md) | Day 6 |
| 6. 선택 A / B | [stage-6-stretch.ko.md](stage-6-stretch.ko.md) | Day 8 |
| 7. 일반화 | [stage-7-generalization.ko.md](stage-7-generalization.ko.md) | Day 8 |
| 8. 마무리 | [stage-8-ship.ko.md](stage-8-ship.ko.md) | Day 9 정오(ET) |

진행 기록: [LOG.ko.md](LOG.ko.md) (실제 기록은 영어판 `LOG.md`에 작성)

## 단계 진행 방법

1. 단계마다(큰 단계는 작업마다) **새 Muse Code 세션**을 엽니다. `AGENTS.md`는 자동으로 읽힙니다.
2. Muse Code에 어떤 단계 파일의 어떤 작업을 할지 알려줍니다. 긴 대화 기록은 붙여넣지 않습니다.
3. 완료 기준 확인은 직접 실행하고, 결과를 `LOG.md`에 기록합니다.
4. 통과하면 커밋하고 넘어갑니다. "Latest" 날이 먼저 오면 단계 파일에 적힌 대안을 적용합니다.

## 모든 단계에 적용되는 규칙

- 이 문서들은 공개됩니다. **정답 값**(기어비, 링크 길이, 관절 목록, 링크 분할)은 `eval/ground_truth/` 밖의 어떤 파일에도 쓰지 않습니다.
- 모델 파일, 렌더, 부품 번호·위치가 담긴 에이전트 로그는 git 제외 폴더(`assets/private/`, `eval/ground_truth/`, `runs/`)에 둡니다.
- 오래 걸리는 작업은 스크립트로 무인 실행합니다. 로그가 아니라 요약을 읽습니다.
- 기본 모델: `muse-spark-1.3-contributor`, 추론 강도 `medium`. 어려운 설계나 디버깅에만 `high`를 씁니다.
