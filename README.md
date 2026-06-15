# OMX Auto-Aim

OpenManipulator-X 기반 자동 조준 시스템. TurtleBot3 Waffle + OMX 6-DOF 팔 (4 모터 사용) + Jetson Orin Nano + ROS 2 Jazzy.

YOLO 로 영상 검출, IBVS (Image-Based Visual Servoing) 로 정밀 조준, Nav2 로 와플 이동, 우선순위 큐 + state machine 으로 다중 좌표 처리.

## 시스템 구성

```
┌─ TurtleBot3 Waffle (mobile base)
│   └─ Nav2 stack (navigation)
│
├─ OpenManipulator-X (4 모터: shoulder_pan/lift, elbow_flex, wrist_flex)
│   └─ Dynamixel U2D2 → /dev/ttyUSB0
│
├─ Jetson Orin Nano 8GB
│   ├─ ROS 2 Jazzy (Ubuntu 24.04, Python 3.12)
│   ├─ YOLO (Ultralytics, custom trained model)
│   ├─ OpenCV
│   └─ omx_aim 패키지 (이 레포)
│
└─ 격발 MCU (예정)
    └─ Jetson GPIO → 트랜지스터 → 발사
```

## 기능

- **다중 좌표 자동 처리** — 우선순위 큐 + state machine
- **3 종 좌표 타입**:
  - `TARGET` (priority 0) — 외부 신뢰 좌표, 최우선
  - `BOUNDARY` (priority 5) — 이동 중 사주 경계 (자동 sweep 생성)
  - `PATROL` (priority 10) — 탐색 좌표
- **CHECK_VIEW + VIEW_POSE** — 현 위치에서 조준 불가하면 와플 이동 위치 자동 계산
- **VIEW_POSE v2** — 12 방향 후보 샘플링 + costmap/LOS/OMX cost 평가
- **TARGET preempt** — 처리 중 PATROL 보다 새 TARGET 우선
- **사주 경계 sweep** — 이동 중 ±45° fan 으로 자동 둘러봄
- **TF / costmap 통합** — Nav2 의 global_costmap 으로 LOS + free space 검사

## 빠른 시작

### 환경 설정

```bash
# alias
alias omxenv='source /opt/ros/jazzy/setup.bash && \
              source ~/venv/omx_ros/bin/activate && \
              export ROS_DOMAIN_ID=28 && \
              cd ~/omx_aim'
```

### 실행

```bash
# 터미널 1: Gazebo + Nav2 + AMCL (별도 세팅)
# ros2 launch turtlebot3_gazebo ...

# 터미널 2: waffle_node (Nav2 클라이언트)
omxenv
python3 apps/waffle_node.py

# 터미널 3: yolo_node (메인)
omxenv
python3 apps/yolo_node.py

# OMX 없이 테스트
python3 apps/yolo_node.py --dry-run
```

### 좌표 발행 (예시)

```bash
# PATROL (탐색)
ros2 topic pub /omx/patrol_in_map geometry_msgs/PointStamped \
  "{header: {frame_id: map}, point: {x: 3.0, y: 1.0, z: 0.3}}" --once

# TARGET (즉시 처리)
ros2 topic pub /omx/target_in_map geometry_msgs/PointStamped \
  "{header: {frame_id: map}, point: {x: 1.5, y: 0.5, z: 0.3}}" --once

# 긴급 정지
ros2 topic pub /omx/abort std_msgs/Empty "{}" --once

# BOUNDARY 자동 생성 토글
ros2 topic pub /omx/boundary_enable std_msgs/String "{data: 'all off'}" --once
```

## 코드 구조

```
omx_aim/
├── config.yaml              # 모든 설정
├── INTERFACE_v4.md          # 인터페이스 명세 (토픽, state, 큐, config)
├── README.md                # 이 파일
│
├── omx/                     # 핵심 로직 (ROS 의존성 없음)
│   ├── types.py             # State, TargetType, LOSResult, TargetEntry
│   ├── state_machine.py     # StateMachine (큐 + state 전이)
│   ├── boundary_gen.py      # BoundaryGenerator (사주 경계)
│   ├── yolo_detector.py     # YoloDetector
│   ├── controller.py        # OmxController (Dynamixel + IBVS)
│   ├── config.py            # dataclass + load_config
│   └── hardware.py          # 저수준 DXL bus
│
├── apps/                    # ROS 노드
│   ├── yolo_node.py         # OmxYoloNode 메인
│   ├── waffle_node.py       # Nav2 클라이언트
│   ├── target_bridge.py
│   ├── keyboard_teleop.py
│   ├── aim_test.py
│   └── track_test.py
│
└── models/
    └── best.pt              # YOLO 학습 모델
```

핵심 로직 (`omx/`) 은 ROS 의존성 없이 콜백 주입 패턴으로 분리. `OmxYoloNode` 가 ROS pub/sub + TF + costmap 만 담당.

## 토픽 요약

자세한 내용은 [`INTERFACE_v4.md`](INTERFACE.md) 참조.

### 입력 (외부 → yolo_node)
- `/omx/target_in_map` — TARGET 좌표
- `/omx/patrol_in_map` — PATROL 좌표
- `/omx/boundary_in_map` — BOUNDARY 좌표 (디버그)
- `/omx/abort` — 긴급 정지
- `/omx/boundary_enable` — BOUNDARY 자동 생성 토글

### 출력 (yolo_node → 외부)
- `/omx/fire` — 격발 신호 (외부 MCU)
- `/omx/nav_goal` — waffle 이동 목표
- `/omx/state` — state machine 상태
- `/omx/target_processed`, `/omx/target_lost`, `/omx/target_not_found` — 알림

## 시각화

RViz 에서:
- `/omx/queue_markers` — 큐 안의 좌표 (TARGET 빨강, BOUNDARY 주황, PATROL 노랑)
- `/omx/nav_goal` — VIEW_POSE
- `/global_costmap/costmap` — VIEW_POSE v2 후보 평가에 사용

OpenCV 창:
- 영상 + bbox + state + 큐 크기 + AIM/SCAN/LOST 진행도 바

## 진화 단계

| Stage | 내용 |
|---|---|
| A/D/F/G | 큐 기본, LOS 검사, 거리 정렬, RViz 마커 |
| H1 | waffle_node 분리 (Nav2 클라이언트) |
| H2 | CHECK_VIEW + VIEW_POSE v1 + WAITING_NAV + 큐 분리 |
| H3 | TARGET preempt (PATROL 폐기/큐 복귀), TARGET miss 알림 |
| H4 | BoundaryGenerator 통합, 자동 sweep + 토글 토픽, TTL |
| H5 | VIEW_POSE v2 — 12 후보 샘플링 + cost 평가 |
| R1~R6 | 코드 모듈 분리 (omx/types, state_machine, boundary_gen, yolo_detector, controller) |

다음 후보: 격발 MCU 펌웨어, LLM 명령 해석, 다중 로봇 협력 (외부 트랙).

## 의존성

- ROS 2 Jazzy
- Ubuntu 24.04
- Python 3.12
- Ultralytics YOLO
- OpenCV
- Dynamixel SDK
- Nav2 (TurtleBot3)
- tf2_geometry_msgs

```bash
sudo apt install ros-jazzy-tf2-geometry-msgs ros-jazzy-nav2-msgs
pip install ultralytics opencv-python
```