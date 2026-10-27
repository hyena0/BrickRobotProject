# 1단계: 데이터와 도구

**Latest:** Day 2 · **목표:** 스튜디오 파일을 구조화된 부품 데이터로 바꾸고, 에이전트에게 정확한 기하 도구를 제공하며, 평가용 정답지를 준비합니다.

## 작업

### 1.1 내보내기와 파싱 (약 2시간) · `src/parse/`

- 스튜디오에서 모델을 LDraw(`.mpd`/`.ldr`)로 내보내 `assets/private/`에 저장.
- LDraw 파싱:
  - 타입 1 줄: `1 <색상> x y z a b c d e f g h i <부품>`. 위치는 LDU 단위(1 LDU = 0.4mm), 3×3 회전 행렬, 부품 파일 이름.
  - 하위 모델(`0 FILE` 블록)은 변환을 합성해서 평탄화.
  - 밀리미터와 오른손 좌표계, **Z축 위쪽**으로 변환 (LDraw는 −Y가 위쪽). 변환 방식은 `docs/schema.md`에 기록.
- 출력 `runs/<id>/parts.json` (비공개): `uid`, `part_id`, `colour`, `position_mm`, `rotation`.
- 하위 모델 이름은 에이전트 입력에서 뺍니다. 디자이너의 묶음 방식이 드러날 수 있습니다.

### 1.2 부품 정보 표 (약 2~3시간) · `src/catalog/parts.yaml`

이 모델에 쓰인 부품 종류(약 84종)와 일반화용 기본 테크닉 부품만 정리합니다.

| 필드 | 의미 |
|---|---|
| `category` | gear, turntable, worm, rack, axle, pin, bush, beam, frame, connector, motor, sensor, brick, other |
| `teeth` | 기어·턴테이블·랙의 톱니 수 |
| `gear_kind` | spur, double-bevel, bevel, crown, worm, internal |
| `axis_local` | 부품 자체 좌표계에서의 회전·삽입 축 |
| `holes` | (로컬 위치, 축, 종류: `axle` / `round`) 목록 |
| `axle_length` | 축 길이 (스터드 단위) |
| `motor` | `large` / `medium`, 출력축 위치와 방향 |

- LDraw 부품 제목(예: "Technic Gear 24 Tooth")으로 `category`와 `teeth`를 자동으로 채우고, 축과 구멍은 직접 확인합니다.
- 빔은 길이로 구멍 위치를 생성합니다 (20 LDU 간격).
- 부품 제목으로 초안을 만들 때 Spark를 써도 됩니다. 모든 항목은 사람이 검토합니다.

### 1.3 기하 도구 (약 3시간) · `src/tools/`

모두 정해진 방식으로 계산하고, 모두 테스트합니다. 도구는 **사실만** 돌려주고 "이것은 관절이다" 같은 결론은 내리지 않습니다.

| 도구 | 반환 |
|---|---|
| `get_parts(category=None, near=None, radius_mm=None)` | uid, part_id, 분류, 위치, 축 |
| `find_connections(uid)` | 이웃 부품과 연결 종류: `axle_in_axle_hole` (회전 고정), `axle_in_round_hole` (자유 회전), `pin_friction`, `pin_frictionless`, `stud`, `contact` |
| `check_gear_mesh(uid_a, uid_b)` | 축 관계(평행/수직/꼬임), 중심 거리, 기대 거리, `meshes: bool`, 맞물림 종류(외접/내접/베벨/웜) |
| `trace_axle(uid)` | 같은 축선 위의 모든 부품 (순서대로, 연결 종류 포함) |
| `rigid_groups(rigid_types)` | 지정한 연결 종류를 고정으로 볼 때의 묶음(union-find): 부품 수, 경계 상자, 포함된 모터와 센서 |

맞물림 규칙 (허용 오차 ±2 LDU):

- 평행한 스퍼 기어와 더블 베벨 기어: 중심 거리 = 1.25 × (T₁ + T₂) LDU (예: 8T + 24T → 40 LDU).
- 수직 베벨, 웜, 랙, 턴테이블, 내접 맞물림: 부품 정보 표의 `gear_kind`별 규칙. 각 규칙은 스튜디오에서 만든 작은 시험 모델로 확인합니다.

### 1.4 기구학 형식 · `docs/schema.md`

공개 문서. 추상 기구학만 담고, 부품 번호나 위치는 넣지 않습니다.

```json
{
  "frame": "base link, Z up, mm",
  "links": [{"id": "L0"}, {"id": "L1"}],
  "joints": [{
    "id": "J1", "type": "revolute", "parent": "L0", "child": "L1",
    "axis": [0, 0, 1], "point_mm": [0, 0, 0],
    "limits_deg": [-180, 180], "zero": "pose in the source file"
  }],
  "actuators": [{"id": "M1", "brick": "slave", "port": "B", "motor": "medium"}],
  "transmission": {
    "coupling": "q = C · m, q in joint degrees, m in motor degrees",
    "C": [[0.0]]
  },
  "end_effector": {"link": "L6", "point_mm": [0, 0, 0]}
}
```

### 1.5 정답지 (비공개) · `eval/ground_truth/`

- `segmentation.json`: 부품별 소속 링크. 분리된 스튜디오 파일에서 만듦.
- `kinematics_gt.json`: 1.4와 같은 형식. 이전 분석 결과, 기존 EV3 코드 상수, 실측으로 채움.
- `robot_gt.xml`: `kinematics_gt.json`으로 만든 기본 도형 MJCF (4단계 작업 4.1의 생성기가 준비됐으면 사용, 아니면 직접 작성).
- `wiring_semantic.yaml`: 배선도의 **의미 층**. 포트마다 어느 관절을 구동하는지, 센서마다 무슨 역할인지(예: 베이스 요 원점). 3단계 구동기 대응과 2단계 센서-관절 대응 평가에 사용.
- 이 폴더의 어떤 내용도 에이전트에게 주지 않습니다.

### 1.6 합성 시험 모델 · `tests/fixtures/`

스튜디오나 스크립트로 만든 작은 LDraw 조각: 맞물린 기어 2개, 아이들러가 있는 기어 트레인, 둥근 구멍을 지나는 축, 턴테이블 관절. 각 조각 옆에 기대하는 도구 출력을 적어 둡니다. 본인 작업이므로 공개합니다.

### 1.7 하드웨어 배선 층 (사람 제공) · `config/ports.local.yaml`

배선도는 두 층으로 나뉩니다. 여기에는 **하드웨어 층**만 넣고, 의미 층은 `eval/ground_truth/wiring_semantic.yaml`에 넣습니다.

```yaml
bricks:
  left:  {role: master, link: wifi_mqtt}
  right: {role: slave,  link: bluetooth_to_left}
motors:            # 모델 속 모터 uid → 실제 포트 (관절 이름 없음)
  - {uid: <parts.json의 모터 uid>, brick: left,  port: A}
sensors:           # 모델 속 센서 uid → 실제 포트와 감지 조건 (의미 없음)
  - {uid: <센서 uid>, brick: left,  port: 1, type: touch, trigger: pressed}
  - {uid: <센서 uid>, brick: right, port: 2, type: color, trigger: "reflect > <기준값>"}
```

- `parts.json`의 모터 6개, 센서 5개를 배선도와 맞춰 한 번만 작성합니다.
- 관절 번호나 "yaw base" 같은 센서 이름은 여기에 **쓰지 않습니다.** 각 모터가 어느 관절을 구동하는지, 각 센서가 어느 관절의 원점인지는 에이전트가 추론해야 합니다.
- 확인: 모델의 모든 모터·센서가 정확히 한 포트에 대응하고, 0단계 `info`의 모든 포트가 정확히 한 부품에 대응.
- README에서 이 파일의 내용은 사람이 제공한 정보로 표시합니다.

### 1.8 시각 확인용 Blender 설정

- Blender에 LDraw 임포터 애드온을 설치하고, 내보낸 모델이 크기와 방향이 맞게 불러와지는지 확인.
- Blender MCP를 Muse Code에 연결 (`~/.config/muse/settings.json`, `mcp_servers.blender`, `mode: optional`). `/mcp`로 확인.
- `src/viz/blender_scene.py` 작성: 모델을 불러오고, 파서와 같은 Z축 위쪽 변환을 적용하고, 각 오브젝트에 부품 `uid`를 붙여 나중에 링크별로 색칠할 수 있게 함.
- 모든 렌더는 `assets/private/`에 둡니다 (외형 정보).

## 완료 기준

- [ ] 파서가 모든 부품을 읽고, 부품 수가 스튜디오와 일치.
- [ ] 부품 정보 표가 모델의 부품 종류를 100% 포함.
- [ ] 모든 도구가 시험 모델 단위 테스트를 통과.
- [ ] 실제 모델 확인: 알고 있는 기어 맞물림 5곳과 자유 회전 연결 3곳을 올바르게 보고.
- [ ] `docs/schema.md` 작성, 정답지 파일이 있고 git에서 제외됨.
- [ ] `config/ports.local.yaml` 작성 완료, `info`와 일치.
- [ ] 부품 uid가 붙은 상태로 Blender에 모델이 불러와짐. `/mcp`에 Blender MCP가 보임.

## 대안

Day 2까지 연결 감지가 불안정하면, `find_connections`를 기어, 축, 턴테이블, 모터로 제한하고 (기어 트레인에는 충분), `rigid_groups`는 보수적인 기본값으로 씁니다.

## 토큰 예상

Muse Code: $1~4 (부품 정보 표와 도구가 가장 큰 코딩 작업).
