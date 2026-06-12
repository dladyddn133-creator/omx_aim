# OMX Aim

TurtleBot3 Waffle 위에 OpenManipulator-X (OMX) 를 올린 로봇 탱크의 자동 조준 시스템.

## 시스템 개요

- **Waffle**: 자율 주행 + 정찰 좌표 생성 (Nav2 기반, 개발 중)
- **OMX**: 표적 조준 + 격발 (좌표 기반 거친 조준 → YOLO 기반 정밀 조준)
- **ROS 2 Jazzy** 토픽 인터페이스로 통합

```
사용자 / 와플 → 좌표 publish → OMX 큐 → LOS 검사 → 조준 → 격발
```

## 주요 기능

### 7단계 상태 머신
```
IDLE → AIMING → SCANNING → TRACKING → CONFIRMING → FIRING → COOLDOWN
```

- **AIMING**: map 좌표를 arm_base 좌표로 TF 변환 후 거친 조준 (Point-at IK)
- **SCANNING**: 표적 검출 대기 (2초)
- **TRACKING**: YOLO 검출 + IBVS 정밀 추적
- **CONFIRMING**: 0.5초 deadband 안정 확인 후 격발
- **FIRING**: gripper close/open 으로 격발 시뮬레이션
- **COOLDOWN**: 5초 대기 + home 복귀

### 우선순위 큐 (heapq 기반)
3단계 표적 분류:
- **TARGET** (priority=0): 확정 표적 → 즉시 처리
- **BOUNDARY** (priority=5): 경계 좌표 → LOS 엄격 적용
- **PATROL** (priority=10): 정찰 좌표 → LOS 관대

정렬 기준: `(priority, distance, count)`
- 같은 priority 안에서 가까운 좌표 먼저
- pop 직전에 와플 위치 기반 거리 재계산

### LOS (Line of Sight) 검사
Nav2 의 `/global_costmap/costmap` 활용한 Bresenham 직선 검사.

| LOS 결과 | TARGET | BOUNDARY | PATROL |
|---|---|---|---|
| CLEAR | 처리 | 처리 | 처리 |
| BLOCKED | 시도 | 폐기 | 시도 |
| UNKNOWN | 처리 | 폐기 | 처리 |

폐기된 좌표는 `/omx/target_blocked` 로 알림.

### 좌표계 자동 변환
- 큐는 map 좌표 (절대 좌표) 보관
- AIMING 시점에 TF2 로 map → base_link → arm_base 변환
- 와플 이동/회전해도 같은 좌표 유효

### RViz 시각화
`/omx/queue_markers` 토픽으로 큐 전체를 마커로 표시:
- 빨강 큰 구체: TARGET
- 주황 중간 구체: BOUNDARY
- 노랑 작은 구체: PATROL
- 초록 큰 구체: 현재 처리 중
- 텍스트 라벨: 종류 + 거리

### 안전 장치
- 표적 추적 중 1.5초 잃으면 자동 IDLE 전이 + `/omx/target_lost` 알림
- 격발 중 새 명령 무시 (CONFIRMING/FIRING)
- 중복 좌표 필터링 (`duplicate_threshold_m`)
- ABORT 토픽으로 비상 정지 + 큐 비움
- 모터 각도 한계 + 스텝 크기 제한

## 환경

- Ubuntu 24.04
- Python 3.12
- ROS 2 Jazzy
- LeRobot (Dynamixel 모터 제어)
- ultralytics (YOLO 추론)
- tf2_geometry_msgs (좌표 변환)
- ROS_DOMAIN_ID = 28

## 셋업

```bash
# 가상환경
python3 -m venv ~/venv/omx_ros --system-site-packages
source ~/venv/omx_ros/bin/activate
source /opt/ros/jazzy/setup.bash

# Python 패키지
pip install lerobot ultralytics opencv-contrib-python PyYAML dynamixel-sdk
pip install "numpy<2.0"

# ROS 2 패키지
sudo apt install ros-jazzy-tf2-geometry-msgs

# alias
alias omxenv='source /opt/ros/jazzy/setup.bash && source ~/venv/omx_ros/bin/activate && export ROS_DOMAIN_ID=28 && cd ~/omx_aim'
```

## 폴더 구조

```
omx_aim/
├── omx/
│   ├── __init__.py
│   ├── hardware.py        # Dynamixel 모터 정의 + bus
│   └── config.py          # YAML 설정 로딩 (dataclass)
├── apps/
│   ├── keyboard_teleop.py # 키보드 수동 제어
│   ├── aim_test.py        # 좌표 기반 IK 테스트
│   ├── yolo_test.py       # YOLO + IBVS 테스트
│   ├── track_test.py      # 추적 검증
│   ├── yolo_node.py       # 메인 ROS 2 노드 (큐 + LOS + 격발)
│   ├── target_bridge.py   # 외부 좌표 → OMX 토픽 forward
│   └── waffle_node.py     # (예정) 와플 정찰 노드
├── models/
│   └── best.pt            # YOLO 학습 모델 (enemy 클래스)
├── config.yaml            # 캘리브레이션 + 튜닝
├── INTERFACE.md           # ROS 토픽 인터페이스 명세
├── SETUP.md               # 환경 셋업 상세
└── README.md
```

## 사용

### 단일 노드 테스트

```bash
omxenv

# 키보드 직접 제어
python3 apps/keyboard_teleop.py

# 좌표 입력으로 조준 테스트
python3 apps/aim_test.py --dry-run

# YOLO 검출만 테스트
python3 apps/yolo_test.py --dry-run
```

### 통합 시스템 (Gazebo + RViz)

```bash
# Terminal A: Gazebo + Nav2 (와플 시뮬레이션)
# (별도 launch 파일)

# Terminal B: target_bridge
omxenv
python3 apps/target_bridge.py

# Terminal C: OMX 메인 노드
omxenv
python3 apps/yolo_node.py --dry-run    # OMX 실 장비 없을 때
# 또는
python3 apps/yolo_node.py              # OMX 실 장비 있을 때
```

### 좌표 publish 예시

```bash
# 정찰 좌표 (NORMAL priority)
ros2 topic pub /omx/patrol_in_map geometry_msgs/PointStamped \
  "{header: {frame_id: map}, point: {x: 1.0, y: 0.0, z: 0.3}}" --once

# 긴급 표적 (HIGH priority)
ros2 topic pub /omx/target_in_map geometry_msgs/PointStamped \
  "{header: {frame_id: map}, point: {x: 2.0, y: 1.0, z: 0.3}}" --once

# 경계 좌표 (MID priority, LOS 엄격)
ros2 topic pub /omx/boundary_in_map geometry_msgs/PointStamped \
  "{header: {frame_id: map}, point: {x: 1.5, y: 0.5, z: 0.3}}" --once

# RViz 'Publish Point' (P 키 + 클릭) → 자동으로 긴급 표적으로 전달
```

## ROS 토픽 인터페이스

### Subscribe (외부 → OMX)

| 토픽 | 타입 | 용도 |
|---|---|---|
| `/omx/target_in_map` | PointStamped | TARGET (HIGH) |
| `/omx/boundary_in_map` | PointStamped | BOUNDARY (MID) |
| `/omx/patrol_in_map` | PointStamped | PATROL (LOW) |
| `/omx/control_mode` | String | idle (강제 IDLE) |
| `/omx/arm_enable` | Bool | 자율 검출 토글 |
| `/omx/abort` | Empty | 비상 정지 |
| `/global_costmap/costmap` | OccupancyGrid | LOS 검사용 |

### Publish (OMX → 외부)

| 토픽 | 타입 | 용도 |
|---|---|---|
| `/omx/status` | String | 1Hz 상태 |
| `/omx/state` | String | 상태 변경 시 |
| `/omx/target_detected` | Bool | 매 프레임 검출 여부 |
| `/omx/error_norm` | Point | 화면 중심 오차 |
| `/omx/joint_state` | JointState | 매 프레임 관절 위치 |
| `/omx/fire` | Empty | 격발 1회 |
| `/omx/target_processed` | PointStamped | 격발 완료 좌표 |
| `/omx/target_lost` | PointStamped | 추적 잃음 좌표 |
| `/omx/target_blocked` | PointStamped | LOS 차단 좌표 |
| `/omx/aim_progress` | Float32 | CONFIRMING 진행도 (0~1) |
| `/omx/queue_size` | Int32 | 큐 크기 |
| `/omx/patrol_complete` | Empty | 큐 비었을 때 |
| `/omx/queue_markers` | MarkerArray | RViz 시각화 |

자세한 내용은 [INTERFACE.md](INTERFACE.md).

## 진화 단계

| 단계 | 기능 | 상태 |
|---|---|---|
| A | 우선순위 큐 (heapq), 격발 | 완료 |
| D | map 좌표 기반 큐 (TF 변환) | 완료 |
| F | LOS 검사 + TargetType 카테고리 | 완료 |
| G | 거리 기반 정렬 + RViz 마커 | 완료 |
| 와플 노드 | Nav2 기반 정찰 (단순 waypoint) | 진행 중 |
| E | 큐 매니저 노드 분리 | 예정 |
| H | 후보 위치 + 와플 자율 결정 | 예정 |
| - | LLM 통합 (자연어 명령) | 장기 |
| - | 격발 메커니즘 하드웨어 통합 (별도 MCU) | 장기 |

## 개발 이력

- 2026.06.05: 초기 환경 셋업 + 모듈 + GitHub
- 2026.06.08: 다중 PC 마이그레이션 + ROS 2 노드 시작
- 2026.06.10: 상태 머신 + 우선순위 큐 + target_bridge
- 2026.06.10~: 단계 D/F/G + target_lost/blocked + 팀 중간 발표
- 진행 중: 와플 측 노드 설계 (Nav2 기반 단순 waypoint)

## 라이선스

학생 프로젝트 (개인 학습 목적).