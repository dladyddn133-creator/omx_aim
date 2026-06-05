# OMX Aim

TurtleBot3 와플 위에 OpenManipulator-X 를 올린 로봇 탱크의 자동 조준 시스템.

## 구성

- 좌표 기반 거친 조준 (Point-at IK)
- 카메라 + YOLO 기반 정밀 조준 (IBVS)
- ROS 2 Jazzy 통합

## 환경

- Ubuntu 24.04
- Python 3.12
- ROS 2 Jazzy
- LeRobot (모터 제어)
- ultralytics (YOLO 추론)

## 셋업

```bash
python3 -m venv ~/venv/omx_ros --system-site-packages
source ~/venv/omx_ros/bin/activate
source /opt/ros/jazzy/setup.bash

pip install lerobot ultralytics opencv-contrib-python PyYAML dynamixel-sdk
pip install "numpy<2.0"

# alias for convenience
alias omxenv='source /opt/ros/jazzy/setup.bash && source ~/venv/omx_ros/bin/activate && cd ~/omx_aim'
```

## 폴더 구조
## 사용

```bash
omxenv

python3 keyboard_teleop.py
python3 omx_aim.py
python3 omx_yolo_test.py
python3 omx_track.py
```
