# omx_msgs

OMX 자동 조준 시스템의 커스텀 ROS 2 메시지 정의.

## 현재 상태

**미사용** (단계 2 기준). 현재 코드는 표준 메시지 (`std_msgs`, `geometry_msgs`)를 직접 사용.

이 폴더의 `.msg` 파일들은:
1. **인터페이스 명세 참고용** - 어떤 데이터가 오가는지 한곳에 정의
2. **미래 omx_msgs ROS 2 패키지 활성화 시 사용**

## 메시지 정의

### OmxStatus.msg

OMX 노드 상태 통합 메시지. 현재 `/omx/status` (String) 를 미래 확장.

| 필드 | 타입 | 의미 |
|---|---|---|
| status | string | "ready"/"tracking"/"lost"/... |
| dry_run | bool | dry-run 모드 여부 |
| mode | string | "manual"/"coarse"/"fine"/"idle" |
| yaw | float64 | 현재 yaw (rad) |
| pitch | float64 | 현재 pitch (rad) |
| fps | float32 | 처리 빈도 |

### TargetDetection.msg

YOLO 검출 + 오차 통합. 현재 `/omx/target_detected` + `/omx/error_norm` 을 통합.

| 필드 | 타입 | 의미 |
|---|---|---|
| header | std_msgs/Header | 측정 시각 |
| detected | bool | 검출 여부 |
| error_x | float64 | 가로 오차 (-1~+1) |
| error_y | float64 | 세로 오차 (-1~+1) |
| confidence | float32 | 신뢰도 (0~1) |
| bbox_width | uint32 | 박스 너비 (픽셀) |
| bbox_height | uint32 | 박스 높이 (픽셀) |
| target_class | uint16 | YOLO 클래스 ID |

### ControlCommand.msg

외부 제어 명령. 현재 `/omx/control_mode` + `/omx/target_coord` 를 통합.

| 필드 | 타입 | 의미 |
|---|---|---|
| mode | uint8 | 0=IDLE, 1=MANUAL, 2=COARSE, 3=FINE |
| target_coord | geometry_msgs/Point | coarse aim 좌표 |
| auto_fine_after_coarse | bool | coarse 후 자동 fine 전환 |

## 활성화 방법 (미래)

인터페이스 안정화 후 ROS 패키지로 활성화하려면:

### 1) ROS 워크스페이스 만들기

```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
```

### 2) 이 폴더를 ROS 패키지로 복사

```bash
# omx_aim/msgs/ 의 .msg 파일과 package.xml, CMakeLists.txt 를 새 패키지로
cp -r ~/omx_aim/msgs/ ~/ros2_ws/src/omx_msgs/

# 폴더 구조 정리
cd ~/ros2_ws/src/omx_msgs
mkdir -p msg
mv *.msg msg/
```

최종 구조:
```
~/ros2_ws/src/omx_msgs/
├── package.xml
├── CMakeLists.txt
└── msg/
    ├── OmxStatus.msg
    ├── TargetDetection.msg
    └── ControlCommand.msg
```

### 3) 빌드

```bash
cd ~/ros2_ws
source /opt/ros/jazzy/setup.bash
colcon build --packages-select omx_msgs

# 빌드 결과 환경 source
source install/setup.bash
```

### 4) yolo_node.py 코드 변경

표준 메시지 대신 커스텀 메시지 사용:

```python
# Before
from std_msgs.msg import String, Bool
from geometry_msgs.msg import Point

# After
from omx_msgs.msg import OmxStatus, TargetDetection, ControlCommand
```

각 publisher/subscriber 도 그에 맞춰 수정.

### 5) 확인

```bash
ros2 interface list | grep omx
# omx_msgs/msg/OmxStatus
# omx_msgs/msg/TargetDetection
# omx_msgs/msg/ControlCommand

ros2 interface show omx_msgs/msg/OmxStatus
# 메시지 정의 보임
```

## 왜 지금은 안 쓰나?

학생 프로젝트 단계에서:

1. **인터페이스가 자주 바뀝니다** - 매번 colcon build 부담
2. **표준 메시지로 충분** - Point, String, Bool 로 다 표현 가능
3. **팀원 부담 최소화** - 추가 빌드 없이 사용 가능
4. **새 PC 셋업 단순** - ROS 워크스페이스 빌드 필요 없음

인터페이스 안정화 (단계 5 이후) 시점에 활성화 검토.

## 그래도 지금 활성화하고 싶다면

위 단계 1-5 진행. 다만 매 변경마다 빌드 필요.

```bash
# 변경 후 빌드
cd ~/ros2_ws
colcon build --packages-select omx_msgs
source install/setup.bash

# yolo_node 재실행
omxenv
python3 ~/omx_aim/apps/yolo_node.py
```