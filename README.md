# OMX Auto-Aim Subsystem

<div align="center">

**Custom YOLO11n과 IBVS를 이용한  
OpenManipulator-X 자동 조준 및 안전 타격 시스템**

<br>

![ROS2](https://img.shields.io/badge/ROS_2-Jazzy-22314E?style=flat-square&logo=ros&logoColor=white)
![YOLO](https://img.shields.io/badge/YOLO-11n-00FFFF?style=flat-square)
![Jetson](https://img.shields.io/badge/Jetson-Orin_Nano-76B900?style=flat-square&logo=nvidia&logoColor=white)
![Control](https://img.shields.io/badge/Control-IBVS_PD-orange?style=flat-square)
![Robot Arm](https://img.shields.io/badge/Robot_Arm-OpenManipulator--X-blue?style=flat-square)

</div>

> [!NOTE]
> 이 저장소는  
> [ROS 2 Multi-Robot Autonomous Exploration & Response System](https://github.com/dladyddn133-creator/ROS2-Project)의  
> **Leader Waffle 자동 조준 및 타격 서브시스템**을 상세하게 정리한 저장소입니다.

---

## Demo

<div align="center">

[![YOLO11n and IBVS Auto-Aim Demo](docs/media/ibvs_thumbnail.jpg)](https://youtu.be/t03m8PHifMo)

<br>

[▶ 전체 IBVS 자동 조준 및 타격 영상 보기](https://youtu.be/t03m8PHifMo)

</div>

> **Control flow:**  
> 위험 좌표 수신 → Point-at IK 개략 조준 → YOLO11n 표적 검출 →  
> IBVS PD 정밀 제어 → 안정성 확인 → GPIO 타격 → Home 복귀

---

## Overview

`OMX Auto-Aim`은 Leader Waffle에 장착된 OpenManipulator-X가  
지도상 위험 좌표를 전달받아 표적을 탐색하고, 영상 기반 제어를 통해 정밀하게 조준한 뒤 안전 조건을 확인하여 타격하는 ROS 2 서브시스템입니다.

지도 좌표를 이용한 **Point-at IK 개략 조준**과 카메라 영상을 이용한  
**YOLO11n + IBVS 정밀 조준**을 결합했습니다.

또한 긴급 표적 선점, 조준 위치 재탐색, Deadband 안정성 확인,  
Cooldown 및 타격 비활성화 기능을 상태 머신에 통합했습니다.

### Core Pipeline

```text
Target Coordinate in Map
        ↓
Target Priority Queue
        ↓
Nav2 Positioning / View Pose
        ↓
Point-at IK
        ↓
Custom YOLO11n Detection
        ↓
IBVS Pan / Tilt PD Control
        ↓
Target Stability Confirmation
        ↓
GPIO Firing
        ↓
Cooldown and Home Position
```

---

## 시스템 구성

```
[Burger]                        [Desktop]                    [Waffle (Jetson)]
SLAM + risk_map                 map_relay                    yolo_node
heartbeat + pose                auto_initialpose             waffle_node
       |                        scout_watchdog               fire_node
       +--/scout/map--------->  Nav2 + RViz                  target_bridge
       +--/risk/risk_map----->                               scan_processor
       +--/scout/heartbeat-->                                patrol_planner
                                     |                       turtlebot3_node
                                     +---/nav_goal--->       OMX motors
                                     <---/nav_result---      발사 메커니즘
```

## 기능

### 조준 파이프라인

1. **Point-at IK** — map 좌표를 OMX 관절 각도로 변환해 개략 조준
2. **YOLO 검출** — 조준 방향에서 표적 탐색
3. **IBVS 정밀 추적** — 화면 오차 기반 PD 제어로 표적을 중앙에 고정
4. **안전 확인** — deadband 안에서 일정 시간 유지되어야 격발

### 좌표 관리

- **3 종 우선순위 큐**: TARGET(0) / BOUNDARY(5) / PATROL(10)
- **8 상태 머신**: IDLE / WAITING_NAV / AIMING / SCANNING / TRACKING / CONFIRMING / FIRING / COOLDOWN
- **TARGET preempt**: 정찰 중 긴급 표적이 들어오면 진행 중인 작업을 밀어냄
- **자동 시야 확보**: 현 위치에서 조준 불가하면(CHECK_VIEW) 표적 주변 12 방향 후보를 cost 평가해(VIEW_POSE) Nav2 로 이동
- **사주 경계 sweep**: 이동 중 ±45° 방위를 순차적으로 둘러봄

### Burger 협력

- **map relay**: `/scout/map` → `/map` (Nav2 입력, latched)
- **patrol planner**: risk_map 의 hotspot 을 NMS 로 추출해 PATROL 좌표 발행
- **auto initial pose**: map 수신 시 AMCL 자동 초기화
- **scout watchdog**: Burger heartbeat 가 끊기면 마지막 위치 주변을 TARGET 으로 수색

### 격발

- **fire_node**: `/omx/fire` → GPIO 펄스 → 발사 메커니즘
- **안전 기능**: cooldown, `/omx/fire_disable` 잠금, 부팅/종료 시 LOW 보장
- 격발 펄스 동안 조준 자세를 유지한 뒤 home 복귀 (팔이 먼저 빠지지 않도록)

### 디버그 대시보드

`yolo_node` 안에서 Flask 스레드로 동작. 브라우저에서 실시간 영상 + 상태 확인.

- MJPEG 영상 스트림 + SSE 상태 push (Live / Ops 2탭)
- 헤드리스 SSH 환경에서 조준 상태를 눈으로 확인하기 위해 만들었다

---

## Build and Run

<details>
<summary><strong>설치 및 실행 명령어 펼쳐보기</strong></summary>

<br>

### 빌드

레포 자체가 colcon 워크스페이스이고, ROS 패키지는 `src/omx_aim/` 에 있다.

```bash
cd ~/omx_aim
colcon build --symlink-install   # config.yaml / launch 수정 후 재빌드 불필요
source install/setup.bash
```

> YOLO 가중치(`models/best.pt`)는 용량 문제로 저장소에 포함되지 않는다. 클론 후 `src/omx_aim/models/` 에 직접 배치할 것.

### 환경 alias

```bash
# Jetson
alias omxenv='source /opt/ros/jazzy/setup.bash && \
              source ~/venv/omx_ros/bin/activate && \
              source ~/omx_aim/install/setup.bash && \
              export ROS_DOMAIN_ID=20 && \
              cd ~/omx_aim'
```

### 실행 — Jetson 측

```bash
# 1. TurtleBot3 bringup
omxenv
export TURTLEBOT3_MODEL=waffle_pi
ros2 launch turtlebot3_bringup robot.launch.py
ros2 service call /motor_power std_srvs/srv/SetBool "{data: true}"

# 2. waffle_node + yolo_node + fire_node + target_bridge + scan_processor + patrol_planner
ros2 launch omx_aim jetson.launch.py

# 디버그 대시보드까지 켜기 → http://<jetson-ip>:8080/
ros2 launch omx_aim jetson.launch.py debug_stream:=true
ros2 launch omx_aim jetson.launch.py debug_stream:=true debug_port:=8090
```

### 실행 — Desktop 측

```bash
# 1. Nav2 (와플 켜진 후)
export TURTLEBOT3_MODEL=waffle_pi
ros2 launch nav2_bringup bringup_launch.py \
  map:=/tmp/nav2_dummy/dummy.yaml use_sim_time:=False

# 2. RViz
rviz2

# 3. 필요한 노드 개별 실행
ros2 run omx_aim map_relay          # /scout/map -> /map
ros2 run omx_aim auto_initialpose   # AMCL 초기화
ros2 run omx_aim scout_watchdog     # Burger 사망 감지
```

`desktop.launch.py` 는 위 세 노드가 주석 처리된 상태이며, 활성화된 것은 patrol_planner 뿐이다. patrol_planner 는 `jetson.launch.py` 에도 들어있으므로 **둘 중 한 곳에서만** 실행할 것.

### 시뮬 (Burger 없이)

```bash
ros2 launch omx_aim sim.launch.py   # fake_static_map + fake_risk_map
```

시뮬은 `/scout/risk_map` 으로 발행하므로 patrol_planner 쪽 토픽을 맞춰줘야 한다.

```bash
ros2 run omx_aim patrol_planner --ros-args -p risk_topic:=/scout/risk_map
```

### 좌표 발행

```bash
# TARGET (즉시 처리)
ros2 topic pub /omx/target_in_map geometry_msgs/PointStamped \
  "{header: {frame_id: map}, point: {x: 1.5, y: 0.5, z: 0.0}}" --once

# PATROL (정찰)
ros2 topic pub /omx/patrol_in_map geometry_msgs/PointStamped \
  "{header: {frame_id: map}, point: {x: 3.0, y: 1.0, z: 0.0}}" --once

# 자율 검출 ON (IDLE 상태에서 표적 보이면 자동 추적)
ros2 topic pub /omx/arm_enable std_msgs/Bool "{data: true}" --once

# 긴급 정지
ros2 topic pub /omx/abort std_msgs/Empty "{}" --once
ros2 topic pub /omx/fire_disable std_msgs/Bool "{data: true}" --once
```

RViz 의 `Publish Point`(P 키)로 맵을 클릭해도 TARGET 으로 들어간다.
</details>

---

## ROS 2 Interfaces and Visualization

<details>
<summary><strong>주요 토픽 및 RViz 시각화 정보 펼쳐보기</strong></summary>

<br>

전체 토픽 목록, 메시지 타입 및 인터페이스 계약은  
[`INTERFACE.md`](INTERFACE.md)를 참고하세요.

### Input Topics

| Topic | Description |
|---|---|
| `/scout/map` | Active Scout가 생성한 SLAM 지도 |
| `/risk/risk_map` | 객체 검출 결과를 누적한 위험 지도 |
| `/scout/heartbeat` | Active Scout 동작 상태 |
| `/scout/pose` | Active Scout의 현재 위치 |
| `/omx/target_in_map` | 즉시 대응할 표적 좌표 |
| `/omx/patrol_in_map` | 순찰 대상 좌표 |
| `/omx/abort` | 현재 조준 및 이동 작업 긴급 중단 |
| `/omx/arm_enable` | 자동 표적 검출 및 조준 활성화 |
| `/omx/fire_disable` | 타격 장치 비활성화 |
| `/omx/control_mode` | OMX 제어 모드 변경 |

### Output Topics

| Topic | Description |
|---|---|
| `/omx/fire` | `fire_node`로 전달되는 타격 신호 |
| `/omx/nav_goal` | Leader Waffle의 Nav2 이동 목표 |
| `/omx/nav_cancel` | 진행 중인 Nav2 이동 취소 |
| `/omx/state` | 자동 조준 상태 머신의 현재 상태 |
| `/omx/status` | OMX 서브시스템 동작 상태 |
| `/omx/queue_size` | 대기 중인 좌표 작업 수 |

### RViz Visualization

| Topic | Description |
|---|---|
| `/map` | Nav2 입력 지도 |
| `/risk/risk_map` | 위험도 히트맵 |
| `/global_costmap/costmap` | Nav2 Global Costmap |
| `/patrol_planner/markers` | 순찰 후보 위치 |
| `/scout_watchdog/markers` | Scout 장애 발생 시 수색 후보 |
| `/omx/queue_markers` | 우선순위 큐에 등록된 좌표 |

</details>

---

## Engineering Evolution

프로젝트를 실제 로봇에 적용하면서 발생한 문제를 분석하고,  
제어 구조와 소프트웨어 아키텍처를 단계적으로 개선했습니다.

| Problem | Improvement | Result |
|---|---|---|
| 하나의 노드에 이동·조준·타격 로직이 집중됨 | `waffle_node`와 OMX 제어 모듈을 분리 | 이동 제어와 로봇팔 제어의 책임 분리 |
| 현재 위치에서 로봇팔이 표적을 볼 수 없는 상황 발생 | `CHECK_VIEW`와 `VIEW_POSE` 상태 추가 | 조준 가능한 위치로 자동 재이동 |
| 단일 View Pose만으로는 장애물과 Costmap 대응이 어려움 | 표적 주변 12개 후보 위치 생성 및 Cost 평가 | 이동 가능한 조준 위치를 자동 선택 |
| 순찰 중 긴급 표적이 입력되면 대응이 지연됨 | TARGET 우선순위와 작업 선점 기능 적용 | 진행 중인 PATROL 작업을 중단하고 표적 우선 대응 |
| 실제 OMX 관절 방향과 계산 결과가 일치하지 않음 | 관절별 Motor Sign과 조준 방향 보정 | 실물 로봇 기준 Pan/Tilt 제어 안정화 |
| 화면 중심 기준 Deadband가 좌우·상하에서 동일하게 동작하지 않음 | 비대칭 Deadband와 PD Gain 개별 튜닝 | 표적 중심 정렬 시 진동과 과도 응답 감소 |
| 타격 신호 발생 직후 로봇팔이 먼저 Home으로 복귀함 | 타격 Pulse 동안 조준 자세 유지 | 타격 동작 완료 후 안전하게 Home 복귀 |
| SSH 환경에서 카메라와 상태 확인이 어려움 | Flask MJPEG 및 SSE 대시보드 구현 | 원격 환경에서 영상과 상태 머신 실시간 확인 |

### Architecture Refactoring

초기에는 ROS 2 노드와 제어 알고리즘이 강하게 결합되어 있었지만,  
개발 과정에서 다음과 같이 구조를 분리했습니다.

```text
ROS 2 Interface Layer
    ├── Topic Subscription / Publication
    ├── TF and Coordinate Conversion
    ├── Nav2 Integration
    └── Hardware Interface
                ↓
OMX Core Logic
    ├── Priority Queue
    ├── State Machine
    ├── Point-at IK
    ├── IBVS PD Control
    └── Safety Conditions
```

이를 통해 ROS 2 통신 로직과 조준 알고리즘을 분리하고,  
상태 머신과 제어 로직을 독립적으로 수정하고 검증할 수 있도록 구성했습니다.

---

## Known Limitations

실물 로봇 시스템 특성상 다음과 같은 제약이 남아 있습니다.

| Limitation | Cause and Consideration |
|---|---|
| `patrol_planner` 중복 실행 가능성 | Jetson과 Desktop Launch에서 동시에 실행하면 동일 좌표가 중복 발행될 수 있으므로 한쪽에서만 실행해야 함 |
| Desktop Launch 일부 노드 수동 실행 필요 | `desktop.launch.py`에서 일부 노드가 비활성화되어 있어 실행 구성을 확인해야 함 |
| Nav2 또는 AMCL 종료 시 좌표 변환 실패 | `map → odom` TF가 사라지면 지도 좌표 기반 조준과 이동을 수행할 수 없음 |
| 가깝고 높은 표적의 조준 범위 제한 | OpenManipulator-X의 `shoulder_lift` 관절 한계로 일부 좌표는 도달 불가능 |
| YOLO 모델의 환경 의존성 | 프로젝트 실험 환경에서 수집한 단일 표적 데이터로 학습되어 새로운 배경과 조명에서는 추가 학습이 필요할 수 있음 |
| 네트워크 상태에 따른 영상 지연 | Flask MJPEG 스트리밍과 ROS 2 토픽을 동시에 사용할 경우 Wi-Fi 대역폭의 영향을 받을 수 있음 |

> 전체 인터페이스 제약과 세부 구현 내용은  
> [`INTERFACE.md`](INTERFACE.md)의 부록을 참고하세요.

---

## 문서

| 문서 | 내용 |
|---|---|
| [`INTERFACE.md`](INTERFACE.md) | 토픽 / 상태 머신 / 큐 정책 / 파라미터 계약 |
| [`SETUP.md`](SETUP.md) | 설치 · 하드웨어 셋업 · 트러블슈팅 |
| [`config/config.yaml`](src/omx_aim/config/config.yaml) | 모든 런타임 값의 단일 출처 |

## 의존성

```bash
sudo apt install \
  ros-jazzy-turtlebot3* \
  ros-jazzy-nav2-bringup \
  ros-jazzy-teleop-twist-keyboard \
  ros-jazzy-tf2-geometry-msgs

pip install ultralytics opencv-python flask dynamixel-sdk Jetson.GPIO
```

Jetson 환경은 패키지 버전 조합이 까다로워 `requirements.jetpack72.lock` 에 검증된 스냅샷을 남겨두었다. 자세한 절차는 [`SETUP.md`](SETUP.md) 참조.