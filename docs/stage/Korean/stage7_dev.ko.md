# 7단계 개발 정리 (T-Bot 일반화)

- 목표: 기존 파이프라인으로 새 모델(Rebrickable MOC-85202 "Gantry Robot T-Bot", glaysche)을 분석한다.
- 회귀 방지 기준(전 구간 준수): 모델 전용 처리 금지, MR JOS 결과를 바꾸는 변경 금지, 기존 함수 통째로 바꾸기 금지. 코드 변경마다 `scripts/regress_mrjos.sh` 실행, 기준선과 다르면 즉시 중단·보고. 변경은 하나씩 커밋하고 영문 로그(`docs/stage/English/stage7_dev.md`)에 이유와 회귀 결과를 기록.
- 비공개 규칙: `.io`/LDraw/메시/렌더는 `assets/private/`(git 제외), Modal 업로드 금지. 최종 회귀: `NO DIFFERENCE`.

## T.0 기준선 — 완료

- `runs/baseline/mrjos_before_stage7/`에 MR JOS 분석 결과 12종 동결 + `MANIFEST.sha256`. 태그 `baseline-before-stage7`.
- 비교 스크립트 `scripts/regress_mrjos.sh` 신규 작성(해시·관절 수/축/위치·기어비·연동·Gate 1 재평가·pytest). 초기 비교기 자체 오차 0.35°는 축 정규화 누락이었으며 스크립트 안에서 수정.
- 기준값: Gate 1 6/6, Gate 2 v2 100.0% (n=500), IK gizmo 59/60.

## T.1 파싱·부품표 — 완료 (966부품, 64/64종)

- `.io`가 ZipCrypto 암호화라 Studio LDraw 내보내기(`MOC-85202_glaysche_Gantry_v4.ldr`, MPD 17블록)로 진행. 1단계 파서로 **966부품·64종·모터 3개**, Studio 표시와 정확히 일치.
- 파서 수정(일반 LEGO-MPD 규칙): 공백 포함 하위모델 참조 처리 + 루트 우선 폴백. MR JOS 재파싱 바이트 동일. 테스트 추가.
- 부품표: 누락 17종 데이터 추가 후 LDraw 형상에서 축·구멍·이빨수·피치 기입(11955 8T 확인, 18942 32T·피치 8LDU=3.2mm). 선형 관절 후보(랙·체인·레일) 목록화.
- 승인된 분류 기본값 수정 1건 + 회귀 검사. A17 맞물림은 원인 조사만, 3·4번은 그대로.

## T.2 as-is + T.2b — 완료

- 1~4단계를 힌트·코드 수정 없이 실행. 랙·피니언과 체인 구동 미인식은 예상된 일반화 실패로 기록만. 결과: 278 강체 그룹, 관절 후보 17개(전부 회전 가설), 기어비·연동 없음.
- 사람 분할 파일 검증 PASS: base 845 + top_shuttle 53 + vertical_axis 68 = 966, 원본과 위치 일치, 모터·허브 전부 base.
- `motor_paths.json` 저장(에이전트 제안 + 사람 확인, 출처 기록). 움직임 3개(좌우/상하/전후), 공동 구동 없음.
- Spark 관절 선정 1회 실행(6개 revolute 선정, gaps 기록). 선정 프롬프트의 MR JOS 편향 발견 → 기존 유지 + 일반 프롬프트(`--prompt-profile generic`)를 별도 추가. MR JOS 일반 프롬프트 비교 실행.

## T.3 선형 관절 — 완료

- A: MR JOS 전용 가정 목록화(수정 없음). B: 일반 프롬프트 + 비교표. C: 신규 `src/tools/linear.py`만으로 랙 메시 판정·체인 추적(200링크, 피치 6.4mm)·스프로킷 식별, 스키마에 prismatic 추가(회전 로직 불변).
- 결과: P1/P2/P3 선형 관절. P1 153.6mm/회전(잠정, 포트 미확인). P2/P3는 구동 경로 단절로 null.
- 선형 FK(PoE 이동축)·DLS IK kinds 대응·갠트리 닫힌 해·MJCF slide 추가. 20개 목표 비교: DLS 20/20, 닫힌 해 20/20, 일치, MuJoCo 오차 0.000043mm.

## 마무리 — 완료

- 플런저 분리(사람 확인 2회 반영): 최종 D3 = 회색 랙 18942 + 빨간 액슬 32062 2부품만 전후 이동, 나머지 66부품은 수직축에. D2=66. 가동범위 ±47.2mm(랙이 고정 피니언에서 벗어나지 않는 조건).
- 움직임 비교 그림 3장 + LDraw 팩 검증 렌더 2장(`assets/private/renders/tbot/`).
- T-Bot 비주얼 팩 D0~D3(160만 삼각형, 부품 누락 0, AABB 검사 통과). 외피는 메시 AABB에 맞춤(원인·대응은 영문 로그에 기록).
- 콘솔: 로봇 선택(MR JOS/T-Bot), 가상 전용(실물 연결 차단), mm 패널, 기즈모 IK 추종, LDraw 팩 자동 감지(양쪽 로봇).
- 버그 4건 수정·검증: T-Bot 전환 시 전체 로딩 실패(관절 수 하드코딩), 원점 복귀(폴링 null 덮어쓰기), 배너 가상 표시, 미리보기 400 에러.
- tbot 스모크 실브라우저 **전 항목 PASS**(IK 0.0, 유지 0.0, 에러 0).

## 미확인 종결 (HW 미연결로 검증 불가)

- 포트 A/B/C ↔ 모터 uid 매핑, M1/M2 ↔ 움직임 1/2 배분, P2/P3 mm/회전, P3 대칭 맞물림 가정, 체인 루프 미닫힘 25부품, 랙 짝지 부품 식별.

## 남김 (의도적 미실행)

- connected/virtual 스모크, Blender 미려 렌더, T-Bot 포트 파일, 연동 추론 자동화, 구동 경로 추가 식별.

## 주요 파일 위치

- 기준선·비교: `runs/baseline/mrjos_before_stage7/`, `scripts/regress_mrjos.sh`, 태그 `baseline-before-stage7`.
- T-Bot 원본(비공개): `assets/private/models_ext/tbot/`, 분석: `runs/stage7_tbot_t2/`(링크·모터경로·선형·모델·패키지), 렌더: `assets/private/renders/tbot/`, 팩: `assets/private/visual_pack/tbot/`.
- 코드: `src/parse/ldr.py`, `src/catalog/parts.yaml`, `src/agent/joints_spark.py`(+generic), `src/tools/linear.py`, `src/kin/`(fk·ik·mjcf·model), `src/console/control.py`, `scripts/ui_smoke.py`.
- 상세 영문 로그: `docs/stage/English/stage7_dev.md`.
