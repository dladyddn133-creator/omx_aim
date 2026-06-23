# OMX Auto-Aim

OpenManipulator-X 기반 자동 조준 시스템. TurtleBot3 Burger (정찰) + Waffle (사격) 협력 + OMX 4-DOF 팔 + Jetson Orin Nano + ROS 2 Jazzy.

Burger 가 SLAM 으로 맵을 만들고 위험도를 평가, Waffle 이 그 정보로 정찰 + 자동 조준 + 격발까지 수행하는 정찰-대응 시스템.

## 시스템 구성

```
[Burger]                          [Desktop]                       [Waffle (Jetson)]
SLAM + risk_map                   domain_bridge                   yolo_node
       |                          map_relay                       waffle_node
       +--/scout/map----------->  patrol_planner --/patrol-->     fire_node
       +--/scout/risk_map----->   auto_initialpose                turtlebot3_node
                                  Nav2 + RViz                     OMX motors
                                       |                          fire mechanism
                                       +---/nav_goal--->
                                       <---/nav_result---
```

## 기능

### 핵심
- **3 종 좌표 우선순위 큐**: TARGET(0) / BOUNDARY(5) / PATROL(10)
- **8 상태 머신**: IDLE / WAITING_NAV / AIMING / SCANNING / TRACKING / CONFIRMING / FIRING / COOLDOWN
- **4 단계 조준 파이프라인**:
  1. Point-at IK (좌표 -> 각도)
  2. YOLO 검출
  3. IBVS 정밀 추적
  4. 안전 확인 (0.5초 hold)
- **자동 시야 확보**: CHECK_VIEW -> VIEW_POSE v2 (12 방향 후보 + cost 평가) -> Nav2 이동
- **사주 경계 sweep**: 이동 중 ±45° 방위 둘러봄
- **TARGET preempt**: 정찰 중 긴급 표적 우선

### Burger 협력 (신규)
- **map relay**: /scout/map -> /map (Nav2 입력)
- **patrol planner**: risk_map 의 hotspot 을 NMS + decay 로 PATROL 좌표 발행
- **auto initial pose**: map 수신 시 자동 AMCL 초기화

### 격발 (신규)
- **fire_node**: /omx/fire -> GPIO 펄스 -> 발사 메커니즘
- **안전 기능**: cooldown, disable 토픽, 부팅 시 LOW

## 빠른 시작

### 환경 alias

```bash
# Jetson
alias omxenv='source /opt/ros/jazzy/setup.bash && \
              source ~/venv/omx_ros/bin/activate && \
              export ROS_DOMAIN_ID=20 && \
              cd ~/omx_aim'
```

### 실행 - Desktop 측

```bash
# 1. Domain bridge (Burger 와 통신)
ros2 run domain_bridge domain_bridge configs/scout_bridge.yaml

# 2. Map relay
python3 apps/map_relay.py

# 3. Patrol planner
python3 apps/patrol_planner.py

# 4. Auto initial pose
python3 apps/auto_initialpose.py

# 5. Nav2 (와플 켜진 후)
export TURTLEBOT3_MODEL=waffle_pi
ros2 launch nav2_bringup bringup_launch.py \
  map:=/tmp/nav2_dummy/dummy.yaml use_sim_time:=False

# 6. RViz
rviz2
```

### 실행 - Jetson 측

```bash
# 1. TurtleBot3 bringup
omxenv
export TURTLEBOT3_MODEL=waffle_pi
ros2 launch turtlebot3_bringup robot.launch.py
ros2 service call /motor_power std_srvs/srv/SetBool "{data: true}"

# 2. waffle_node
python3 apps/waffle_node.py

# 3. yolo_node
python3 apps/yolo_node.py --no-display

# 4. fire_node
python3 apps/fire_node.py
```

### 시뮬 (Burger 없이)

Burger 가 없으면 가짜 map + risk_map 발행:

```bash
python3 apps/fake_static_map.py     # /scout/map 시뮬
python3 apps/fake_risk_map.py       # /scout/risk_map 시뮬
```

### 좌표 발행

```bash
# TARGET (즉시 처리)
ros2 topic pub /omx/target_in_map geometry_msgs/PointStamped \
  "{header: {frame_id: map}, point: {x: 1.5, y: 0.5, z: 0.0}}" --once

# PATROL (정찰)
ros2 topic pub /omx/patrol_in_map geometry_msgs/PointStamped \
  "{header: {frame_id: map}, point: {x: 3.0, y: 1.0, z: 0.0}}" --once

# 자동 조준 ON
ros2 topic pub /omx/arm_enable std_msgs/Bool "{data: true}" --once

# 긴급 정지
ros2 topic pub /omx/abort std_msgs/Empty "{}" --once
```

## 토픽 요약

자세한 내용: `INTERFACE_v5.md`

### 외부 입력
- /scout/map, /scout/risk_map - Burger
- /omx/target_in_map, /omx/patrol_in_map - 좌표
- /omx/abort, /omx/arm_enable, /omx/fire_disable - 제어

### 외부 출력
- /omx/fire - 격발 신호 (fire_node 수신)
- /omx/nav_goal - Nav2 이동
- /omx/state, /omx/target_processed - 상태/완료

## 시각화

RViz:
- /map - Nav2 입력
- /scout/risk_map - 위험도 히트맵
- /global_costmap/costmap - Nav2 cost map
- /patrol_planner/markers - PATROL 후보 + decay 영역
- /omx/queue_markers - 큐 안의 좌표

## 진화 단계

| Stage | 내용 |
|---|---|
| A/D/F/G | 큐, LOS, 거리 정렬, RViz |
| H1 | waffle_node 분리 |
| H2 | CHECK_VIEW + VIEW_POSE v1 + WAITING_NAV |
| H3 | TARGET preempt + miss 알림 |
| H4 | BoundaryGenerator sweep + TTL |
| H5 | VIEW_POSE v2 (12 후보 + cost) |
| R1~R6 | 모듈 분리 |
| Burger 통합 | map_relay + patrol_planner + decay + auto_initialpose |
| 격발 통합 | fire_node + GPIO + 안전 기능 |
| 운영 보정 | motor sign 보정, deadband 비대칭, 2D 운영 |

## 다음 후보

- on_abort 의 nav_cancel 통합 (안전 이슈)
- boundary_scan_timeout_sec 적용
- LLM 명령 해석 (자연어 -> 좌표)
- 실 와플 동작 확인 (배터리 해결 후)

## 의존성

```bash
sudo apt install \
  ros-jazzy-turtlebot3* \
  ros-jazzy-nav2-bringup \
  ros-jazzy-domain-bridge \
  ros-jazzy-teleop-twist-keyboard \
  ros-jazzy-tf2-geometry-msgs

pip install ultralytics opencv-python Jetson.GPIO
```