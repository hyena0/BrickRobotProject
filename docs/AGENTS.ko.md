# AGENTS.md (한국어 참고본)

> 이 파일은 참고용 한국어 번역입니다. Muse Code가 실제로 읽는 파일은 영어 `AGENTS.md`입니다. 규칙을 바꿀 때는 두 파일을 함께 수정하세요.

해커톤 프로젝트: Muse Spark 에이전트가 브릭 단위 스튜디오 데이터에서 레고 로봇의 기구학 체인을 추론하고, FK/IK와 EV3 제어 코드를 생성합니다. 전체 계획: `PLAN.md`. 단계별 세부 계획: `docs/stages/` (지시받은 단계와 작업만 진행).

## 언어

- 사용자는 한국어로 쓸 수 있습니다. 답변은 한국어로 합니다.
- 코드, 주석, 커밋 메시지, 문서, README는 모두 **영어**로 작성합니다.

## 반드시 지킬 규칙

1. **새 작업만.** 이전 프로젝트의 코드를 복사하지 않습니다. 이 저장소에서 모두 새로 작성합니다.
2. **외형과 조립 데이터는 절대 커밋하지 않습니다.** `.io`, `.ldr`, `.mpd`, 렌더, 메시가 들어간 MJCF는 `assets/private/` 또는 `eval/ground_truth/`에 둡니다(git 제외). 부품 번호와 위치가 담긴 에이전트 로그와 도구 출력은 `runs/`에 둡니다(git 제외). 커밋할 수 있는 것은 추상 기구학(관절축, 링크 치수, 기어비, 연동, 가동 범위), IK 코드와 가중치, 기본 도형 MJCF, 생성된 EV3 코드뿐입니다. `PLAN.md` 4a절 참고.
3. **비밀 정보는 절대 커밋하지 않습니다.** API 키는 환경 변수(`META_API_KEY`, `MODEL_API_KEY`)에서 읽습니다. `.env`는 git에서 제외합니다.
4. **정답지는 에이전트에게 주지 않습니다.** `eval/ground_truth/`와 `gt/`(`gt/gt_joints.json` 포함)의 파일은 검증기 전용입니다. 그 내용(관절 목록, 기어비, 링크 분할, 링크 길이)을 에이전트 프롬프트나 도구 출력에 넣지 않습니다. `gt/` 경로는 채점기 실행 인자(`--gt`)로만 전달합니다. `src/`의 추출·추론 코드(`src/verify/` 제외)는 import하거나 읽으면 안 됩니다(`tests/test_gt_isolation.py`로 고정).
5. **Streamlit 사용 금지.** 대시보드를 핵심 기능으로 만들지 않습니다.
6. 요청이 없으면 커밋, 푸시, amend를 하지 않습니다.
7. **작업 범위는 BrickRobotProject 한정.** 추출 세션은 이 저장소 안에서만 작업합니다. 외부 파일을 읽거나, 나열하거나, 의존하지 않습니다.

## 구조

```
src/
  parse/     .io / LDraw 파서 → 부품 (번호, 종류, 위치·방향)
  catalog/   부품 정보 표: 부품 번호 → 분류, 톱니 수, 축 방향, 모터 종류
  tools/     에이전트에 제공하는 도구 (정해진 방식으로 계산)
             get_parts, find_connections, check_gear_mesh, trace_axle,
             rigid_groups, gear_graph, axle_carrier, solve_transmission,
             axes_intersect
  agent/     Muse Spark 에이전트 루프 (도구 호출, 구조화된 JSON 출력)
  kin/       기구학 모델 → MJCF, FK, IK, 모터 공간 변환
  verify/    정답 비교, MuJoCo 끝단 오차
  codegen/   EV3 프로그램 생성기 → generated/<robot>/
  ev3/       브릭 프로그램 (마스터: Wi-Fi MQTT, 슬레이브: 마스터 경유 블루투스),
             브릭 내 안전 가드, PC 드라이버 (포트만 다룸, 프로토콜은 docs/protocol.md)
  viz/       Blender 장면 불러오기, 링크·축 표시 (렌더는 비공개)
  demo/      O/X 데모 (실물 + 시뮬레이션)
  vla/       시연 생성 + LeRobot 변환 + Modal 학습 (선택)
  gen/       절차적 LDraw 팔 생성기 (일반화 시험용)
models/              공개용 기본 도형 MJCF
config/*.local.yaml  하드웨어, 포트 배선 (모터·센서 uid → 포트, 관절 이름 없음),
                     영점 오프셋 (git 제외, 사람이 제공)
eval/ground_truth/   정답 기구학, 링크 분할, MJCF (검증기 전용)
docs/schema.md       기구학 모델 JSON 형식
docs/stages/         단계별 계획과 LOG.md
```

- 도구는 사실을 계산하고, 에이전트는 그 의미를 해석합니다. 도구에 특정 로봇의 답을 하드코딩하지 않습니다.
- 기구학 모델은 단계 사이의 약속입니다. `docs/schema.md`를 항상 맞춰 둡니다. 형식에는 추상 기구학만 담고, 부품 번호나 위치는 넣지 않습니다.
- VLA 학습 장면은 기본 도형 MJCF만 사용하고, 메시는 쓰지 않습니다.
- 안전: 브릭 내 가드(센서, 정지 감지, 하트비트, 블루투스 연결)는 모든 이동에서 켜져 있어야 합니다. 테스트를 통과시키려고 가드를 끄거나 약하게 하지 않습니다.
- 에이전트는 `config/ports.local.yaml`(하드웨어 층)을 볼 수 있습니다. `eval/ground_truth/wiring_semantic.yaml`(어느 포트가 어느 관절을 구동하는지, 센서의 의미)은 절대 보면 안 됩니다.
- IK는 끝단 목표 위치·자세를 받아 관절 각도를 돌려줍니다. 이 인터페이스는 바꾸지 않습니다.

## Model API

- 주소 `https://api.meta.ai/v1`, OpenAI SDK, Responses API.
- 모델: `muse-spark-1.3-contributor`. 허락 없이 Standard나 Muse Wonder로 바꾸지 않습니다.
- Contributor 제한: 분당 150회. 429 오류에는 지수 백오프로 재시도합니다.
- 에이전트 호출마다 `usage`(입력, 캐시, 출력, 추론 토큰)를 `runs/usage.jsonl`에 기록합니다.
- 캐시를 위해 프롬프트 앞부분을 고정합니다(시스템 프롬프트와 도구 정의를 앞에, 바뀌는 데이터를 뒤에).

## 비용과 작업 습관

- 오래 걸리는 작업(데이터 생성, 학습, 일괄 평가)은 스크립트로 무인 실행합니다. 세션에서 로그를 계속 확인하지 말고, 끝나면 짧은 요약만 읽습니다.
- 파일은 필요한 부분만 읽습니다(`head`, `grep`). 큰 파일, 장면, 데이터셋을 통째로 컨텍스트에 넣지 않습니다.
- 모든 생성·학습 실행은 사전 점검(관절 범위, IK 정상 여부, 장면 유효성)으로 시작하고, 실패하면 중단합니다.
- 서로 독립적인 실험은 병렬로 실행합니다.
- Modal 작업에는 반드시 타임아웃을 설정합니다.

## 명령어

```bash
python -m src.parse.ldr assets/private/model.ldr -o runs/<id>/parts.json  # 파싱
python -m src.catalog.draft runs/<id>/parts.json -o src/catalog/parts.yaml  # 부품표 초안
python -m src.tools.wiring --parts runs/<id>/parts.json --catalog src/catalog/parts.yaml --ports config/ports.local.yaml  # 배선 확인
python -m src.agent.run --model-file assets/private/<robot>.io   # 추론 실행
python -m src.verify.run --pred runs/<id>/kinematics.json         # 검증
python -m src.ev3.check                                           # EV3 연결 확인
python -m src.kin.assemble --joints runs/<id>/links_joints_public.json --coupling runs/stage3/pred_C.json --ports config/ports.local.yaml --parts runs/stage1/parts.json --catalog src/catalog/parts.yaml --out models/robot_inferred.json  # 모델 조립
python -m src.kin.mjcf --model models/robot_inferred.json --out models/robot_inferred.xml  # 단순형상 MJCF
python -m src.kin.fk --model models/robot_inferred.json --q 0,0,0,0,0,0  # FK 자세
python -m src.verify.ik_eval --pred models/robot_inferred.json --gt eval/ground_truth/kinematics_gt.json --n 500  # Gate 2 (GT 블라인드)
```

명령어가 바뀌면 이 절을 업데이트합니다.
