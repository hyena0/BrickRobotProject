# 0단계: 설정

**Latest:** Day 1 · **목표:** 동작하는 저장소, 모든 EV3 모터를 안전하게 구동하고 모든 센서를 읽는 경로, 토큰 비용 기준치 측정.

## 통신 구조

```
PC ── Wi-Fi (로컬망) ── MQTT 브로커 (PC의 Mosquitto)
                              │
                      Left 브릭 (마스터) ── 블루투스 ── Right 브릭 (슬레이브)
```

- **Left(마스터)** 브릭이 Wi-Fi로 MQTT 브로커에 연결됩니다.
- **Right(슬레이브)** 브릭은 블루투스로 마스터와 통신하고, 마스터가 MQTT 메시지를 중계합니다.
- 나중에 두 브릭 모두 Wi-Fi에 연결하면 슬레이브가 직접 구독합니다. 토픽과 메시지 형식은 그대로입니다.
- 로컬망만 사용하고 인터넷에는 노출하지 않습니다.

## 사전 조건

- 두 브릭이 Python이 되는 같은 런타임 사용 (ev3dev + `ev3dev2`, 또는 Pybricks MicroPython). API를 섞지 않습니다.
- 마스터에 Wi-Fi 동글이 동작하고 IP가 고정됨. 두 브릭이 블루투스로 페어링됨.
- PC: Python 3.11, Mosquitto 설치, Muse Code 설치, `META_API_KEY` 설정.
- Billing 페이지에 $150 무료 크레딧이 보임.

## 작업

### 0.1 저장소 뼈대 (약 0.5시간)

- `AGENTS.md`의 폴더 구조대로 생성.
- `.gitignore`: `.env`, `assets/private/`, `eval/ground_truth/`, `runs/`, `config/*.local.yaml`, `__pycache__/`, `.venv/`.
- `pyproject.toml`: `openai`, `numpy`, `scipy`, `mujoco`, `pyyaml`, `paho-mqtt`, `pytest`.
- `README.md` 초안: "작성 / 외부 / 기존 자산 / 평가 전용 / 사람 제공" 빈 표.
- 커밋: `Initial scaffold and planning docs`.

### 0.2 MQTT 프로토콜 · `docs/protocol.md`

| 토픽 | 방향 | QoS | 내용 |
|---|---|---|---|
| `robot/cmd/<brick>` | PC → 브릭 | 1 | `{"id", "cmd", "args"}` |
| `robot/ack/<brick>` | 브릭 → PC | 1 | `{"id", "ok", "result" \| "error"}` |
| `robot/state/<brick>` | 브릭 → PC | 0 | 엔코더(도)와 센서 값, 10~20Hz |
| `robot/event/<brick>` | 브릭 → PC | 1 | 가드 작동, 정지 감지, 연결 끊김 |
| `robot/estop` | PC → 전체 | 1 | 즉시 전체 정지 |
| `robot/heartbeat` | PC → 전체 | 0 | 0.5초마다 |
| `robot/status/<brick>` | 브로커 (Last Will) | 1 | `online` / `offline` |

명령: `ping`, `info` (포트, 모터·센서 종류), `run_target`, `run_targets` (동시 시작), `read`, `reset`, `stop`, `set_guards`, `clear_latch`.

브릭은 **포트만** 압니다. 관절 이름, 기어비, 센서의 의미는 모릅니다.

### 0.3 브릭 프로그램 · `src/ev3/brick/`

- `master.py`: MQTT 클라이언트 (ev3dev는 paho-mqtt, Pybricks는 `umqtt`). 자기 명령을 실행하고, `robot/*/right` 메시지를 블루투스로 중계.
- `slave.py`: 마스터의 블루투스 클라이언트. 자기 명령을 실행.
- 공통 `motors.py` / `sensors.py` / `guards.py`.

### 0.4 안전 계층 (각 브릭 안에서 처리, 네트워크를 거치지 않음)

- **센서 가드:** 각 브릭이 자기 센서를 약 10ms마다 확인. 기본 규칙: **원점 복귀 중이 아닐 때** 설정된 센서가 감지되면 그 브릭의 **모든** 모터를 정지하고, 잠금 상태로 전환하고, 이벤트를 보냄. 마스터는 다른 브릭에도 즉시 정지를 전달.
- **원점 복귀 예외:** 원점 복귀 명령에 `{sensor, motor}`가 함께 오고, 그 센서가 감지되면 해당 모터만 정지 (원점 복귀 완료이며 오류가 아님).
- **정지 감지 가드:** 명령 중인 모터의 속도가 300ms 넘게 거의 0이면 정지하고 보고 (센서가 없는 관절용).
- **하트비트 워치독:** 2초간 하트비트가 없으면 전체 정지.
- **블루투스 워치독:** 슬레이브가 마스터와 연결이 끊기면 슬레이브 전체 정지.
- **잠금:** 가드로 정지한 뒤에는 `clear_latch` 전까지 이동 명령을 거부.

### 0.5 PC 드라이버 · `src/ev3/driver.py`

- `config/hardware.local.yaml`(git 제외)을 읽음: 브로커 주소, 브릭 이름(`left` = 마스터, `right` = 슬레이브).
- API: `move_motors({(brick, port): angle}, speed)`, `read_state()`, `stop()`, `home_motor(brick, port, sensor_port, direction, speed)`, `on_event(callback)`.
- **비상 정지:** 어떤 스크립트에서든 키보드 키로 `robot/estop` 발행.
- 백그라운드 스레드에서 하트비트 전송.

### 0.6 연결 확인 · `src/ev3/check.py`

- 두 브릭에 ping, `info` 출력 (모든 모터와 센서 확인).
- 모터마다 저속으로 +10° / −10° 움직이고, 값을 다시 읽어 오차와 왕복 지연 보고 (마스터, 슬레이브 따로).
- 센서 값을 실시간 출력. 터치 센서를 손으로 눌러 이벤트가 오는지 확인.

### 0.7 토큰 기준치

- 시작 전 Billing 잔액 기록. 0.2~0.6을 Muse Code로 작성 (`muse-spark-1.3-contributor`, 강도 `medium`).
- 약 1시간 뒤: 턴 수, 입력·캐시·출력 토큰, 비용을 `LOG.md`에 기록하고 예상치와 비교.

### 0.8 행정

- 디자이너에게 허락 메일 발송. Modal, Baseten 크레딧 신청.

## 안전

- 속도 상한 100°/s, ±10°만 움직임 (관절 범위를 아직 모름).
- 각 모터를 처음 움직일 때는 손으로 팔을 받쳐 둡니다.

## 완료 기준

- [ ] 두 브릭의 모든 모터가 움직이고, 값이 ±3° 안에서 읽힘.
- [ ] 움직이는 중에 터치 센서를 누르면 그 브릭의 모든 모터가 50ms 안에, 다른 브릭은 300ms 안에 정지.
- [ ] `robot/estop`으로 두 브릭이 300ms 안에 정지.
- [ ] PC 스크립트를 강제 종료하면 두 브릭이 정지 (하트비트 워치독).
- [ ] 블루투스를 끊으면 슬레이브가 정지.
- [ ] 토큰 기준치 기록, 디자이너 메일과 크레딧 신청 완료.

## 대안

Day 1이 끝날 때까지 브릭의 MQTT가 불안정하면, 같은 메시지 형식을 마스터의 일반 TCP 소켓으로 주고받습니다.

## Muse Code 지시 예시

> docs/stages/stage-0-setup.md의 0단계 작업 0.2~0.5를 진행해줘. 브릭은 [ev3dev2 / Pybricks]를 쓴다. docs/protocol.md, 안전 계층이 포함된 마스터·슬레이브 브릭 프로그램, PC 드라이버를 작성해줘. 브릭은 로봇을 몰라야 하고 포트만 다뤄야 해. 가짜 모터·센서 클래스와 가짜 블루투스 연결로 PC에서 돌리는 테스트도 추가해줘.
