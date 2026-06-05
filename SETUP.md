# Setup Guide

새 PC 또는 보드에서 OMX Aim 시스템을 셋업하는 가이드.

## 환경

- Ubuntu 24.04 LTS
- Python 3.12
- ROS 2 Jazzy
- OpenManipulator-X (with ROBOTIS OpenRB-150)

---

## 1. 시스템 패키지 확인

이미 ROS 2 Jazzy 설치된 PC 기준. 없으면 ROS 2 공식 가이드 따라 설치.

```bash
# 확인
python3 --version          # Python 3.12.x
ros2 --version             # ros2 cli 2.x
nvidia-smi | head -5       # GPU (있는 경우)
```

---

## 2. Python 가상환경

ROS 2 Jazzy 의 rclpy 등 시스템 패키지를 상속받기 위해 `--system-site-packages` 필수.

```bash
# venv 생성
python3 -m venv ~/venv/omx_ros --system-site-packages

# 활성화
source /opt/ros/jazzy/setup.bash
source ~/venv/omx_ros/bin/activate

# 확인
which python3              # /home/USER/venv/omx_ros/bin/python3
python3 -c "import rclpy; print('OK')"
```

---

## 3. 편의 alias 추가

매번 ROS + venv 활성화를 한 줄로:

```bash
echo "alias omxenv='source /opt/ros/jazzy/setup.bash && source ~/venv/omx_ros/bin/activate && cd ~/omx_aim'" >> ~/.bashrc
source ~/.bashrc

# 사용
omxenv
```

---

## 4. Python 의존성 설치

```bash
omxenv

pip install --upgrade pip

# 핵심 패키지
pip install lerobot
pip install ultralytics
pip install opencv-contrib-python
pip install PyYAML
pip install dynamixel-sdk

# numpy 다운그레이드 (ROS Jazzy 시스템 OpenCV 4.6 과 호환)
pip install "numpy<2.0"

# OpenCV 충돌 정리 (필요 시)
pip uninstall -y opencv-python opencv-python-headless
```

### 검증

```bash
python3 -c "
import cv2
import torch
import rclpy
from ultralytics import YOLO
from lerobot.motors import Motor, MotorNormMode
from lerobot.motors.dynamixel import DynamixelMotorsBus, OperatingMode, DriveMode
import dynamixel_sdk

print('All imports OK')
print('OpenCV:', cv2.__version__)
print('Torch:', torch.__version__, '| CUDA:', torch.cuda.is_available())
"
```

기대 출력:
```
All imports OK
OpenCV: 4.6.0
Torch: 2.x.x | CUDA: True
```

---

## 5. 코드 가져오기

```bash
cd ~
git clone https://github.com/dladyddn133-creator/omx_aim.git
cd omx_aim

# 동작 확인
python3 -c "from omx.hardware import build_bus; from omx.config import load_config; cfg = load_config(); print('Config:', cfg.motor.port)"
```

---

## 6. OMX 하드웨어 연결 셋업

### 6-1. dialout 그룹 권한

USB 시리얼 장치 접근 권한:

```bash
# 현재 그룹 확인
groups | grep dialout

# 없으면 추가
sudo usermod -aG dialout $USER

# 적용: 로그아웃 → 재로그인 (또는 재부팅)
# 확인
groups | grep dialout
```

### 6-2. USB 인식 확인

OMX 연결 + 전원 ON 후:

```bash
ls /dev/ttyACM* /dev/ttyUSB* 2>&1
```

기대 출력 (보드에 따라 다름):
- **OpenRB-150**: `/dev/ttyACM0`
- **U2D2** (FTDI 어댑터): `/dev/ttyUSB0`

### 6-3. USB 디바이스 정보 확인

```bash
# 보드 식별 정보
udevadm info -a -n /dev/ttyACM0 | grep -E "ATTRS\{(idVendor|idProduct|product|manufacturer|serial)\}" | head -10
```

기록할 정보 (PC 마다 다를 수 있음):

```
ATTRS{idVendor}=="2f5d"
ATTRS{idProduct}=="2202"
ATTRS{manufacturer}=="ROBOTIS"
ATTRS{product}=="OpenRB-150"
ATTRS{serial}=="1ED83624503059384C2E3120FF08072F"   # 보드마다 다름
```

### 6-4. udev rule 만들기

`/dev/omx_follower` 심볼릭 링크가 만들어지도록:

```bash
# 위에서 얻은 serial 값으로 교체
sudo tee /etc/udev/rules.d/99-omx-follower.rules > /dev/null << 'EOF'
# OMX-AI follower arm - ROBOTIS OpenRB-150
SUBSYSTEM=="tty", ATTRS{idVendor}=="2f5d", ATTRS{idProduct}=="2202", ATTRS{serial}=="1ED83624503059384C2E3120FF08072F", SYMLINK+="omx_follower", MODE="0666"
EOF
```

다른 보드(U2D2)면:
```
SUBSYSTEM=="tty", ATTRS{idVendor}=="0403", ATTRS{idProduct}=="6014", ATTRS{serial}=="...", SYMLINK+="omx_follower", MODE="0666"
```

### 6-5. 규칙 적용

```bash
sudo udevadm control --reload-rules
sudo udevadm trigger

# USB 한 번 뽑았다 다시 꽂으면 확실
# 확인
ls -la /dev/omx_follower
```

기대 출력:
```
lrwxrwxrwx 1 root root 7 ... /dev/omx_follower -> ttyACM0
```

### 6-6. 통신 검증

```bash
omxenv

python3 -c "
from omx.hardware import build_bus
try:
    bus = build_bus()    # 기본 포트 /dev/omx_follower 사용
    bus.connect()
    pos = bus.sync_read('Present_Position', normalize=False)
    print('OMX 연결 OK')
    print('모터 위치:')
    for name, p in pos.items():
        print(f'  {name:14s}: {p}')
    bus.disconnect()
except Exception as e:
    print('FAIL:', type(e).__name__, ':', e)
"
```

기대 출력:
```
OMX 연결 OK
모터 위치:
  shoulder_pan  : 2075
  shoulder_lift : 1257
  ...
```

---

## 7. 동작 검증

```bash
omxenv

# 키보드 teleop (가장 단순)
python3 apps/keyboard_teleop.py

# 좌표 조준 (dry-run)
python3 apps/aim_test.py --dry-run

# YOLO 추적 (dry-run)
python3 apps/yolo_test.py --dry-run

# ROS 노드 (dry-run)
python3 apps/yolo_node.py --dry-run
```

다른 터미널에서:
```bash
source /opt/ros/jazzy/setup.bash
ros2 node list
# /omx_yolo_node 보여야 정상
```

---

## 트러블슈팅

### USB 인식 안 됨

```bash
# 커널 로그 확인
sudo dmesg | tail -20
# USB 케이블, OMX 전원 확인
```

### Permission denied (시리얼 포트)

```bash
# dialout 그룹 다시 확인
groups | grep dialout

# 안 들어있으면
sudo usermod -aG dialout $USER
# 재로그인 또는
newgrp dialout
```

### /dev/omx_follower 안 생김

```bash
# udev rule 확인
cat /etc/udev/rules.d/99-omx-follower.rules

# serial 번호 다시 확인 (틀렸을 수 있음)
udevadm info -a -n /dev/ttyACM0 | grep serial

# 규칙 다시 적용
sudo udevadm control --reload-rules
sudo udevadm trigger
# USB 뽑고 다시 꽂기
```

### 모터 응답 없음 (motor check failed)

확인 사항:
- OMX 외부 전원 (12V) 켜져 있는지
- 모터 데이지 체인 연결 (모든 케이블 단단히)
- 모터 ID 가 11~16 (OMX 표준)

baud rate 다양하게 시도:
```bash
python3 -c "
import dynamixel_sdk as dxl
for baud in [57600, 115200, 1000000, 2000000]:
    port = dxl.PortHandler('/dev/ttyACM0')
    packet = dxl.PacketHandler(2.0)
    port.openPort()
    port.setBaudRate(baud)
    found = []
    for motor_id in range(1, 21):
        _, result, _ = packet.ping(port, motor_id)
        if result == dxl.COMM_SUCCESS:
            found.append(motor_id)
    port.closePort()
    if found:
        print(f'baud={baud}: motors {found}')
"
```

### numpy 충돌

```
ImportError: numpy.core.multiarray failed to import
```

해결:
```bash
pip install "numpy<2.0"
```

### opencv import 시 다른 버전

확인:
```bash
python3 -c "import cv2; print(cv2.__file__, cv2.__version__)"
```

기대: `/usr/lib/python3/dist-packages/cv2.../...so` (시스템 ROS, 4.6.0)

venv 에 깔린 opencv-contrib-python 이 우선이면 충돌. 정리:
```bash
pip uninstall -y opencv-contrib-python opencv-python opencv-python-headless
```

---

## 8. 캘리브레이션 (필요 시)

OMX 팔을 분해 또는 모터 펌웨어 리셋했다면 캘리브 다시:

```bash
omxenv

# 팔을 "곧게 정면 수평" 자세로 손으로 옮긴 뒤
python3 apps/aim_test.py --measure-home
```

출력된 raw tick 값들을 `config.yaml` 의 `calibration.home` 에 복사:

```yaml
calibration:
  home:
    shoulder_pan:  2075       # 출력값으로 교체
    shoulder_lift: 1257
    elbow_flex:    2749
    wrist_flex:    1692
    wrist_roll:    2024
    gripper:       3217
```

---

## 호환성 메모

### 다른 보드 사용 시

이 가이드는 **OpenRB-150** 기준. 다른 보드는:

| 보드 | 인터페이스 | 일반 이름 | idVendor | idProduct |
|---|---|---|---|---|
| OpenRB-150 | USB CDC | ttyACM | 2f5d | 2202 |
| U2D2 | FTDI | ttyUSB | 0403 | 6014 |
| OpenCR | USB CDC | ttyACM | 다름 | 다름 |

`udevadm info` 로 본인 보드 정보 확인하고 udev rule 에 반영.

### 다른 OS

- Ubuntu 22.04 → ROS 2 Humble 사용 권장 (Jazzy 비호환)
- Ubuntu 24.04 → ROS 2 Jazzy (현재 가이드)
- macOS, Windows → LeRobot/ROS 2 호환성 불안정. 비추.

---

## 셋업 완료 체크리스트

- [ ] Ubuntu 24.04 + Python 3.12 + ROS 2 Jazzy
- [ ] venv 생성 (`~/venv/omx_ros` with `--system-site-packages`)
- [ ] alias `omxenv` 추가
- [ ] 의존성 설치 (lerobot, ultralytics, opencv, PyYAML, dynamixel-sdk, numpy<2.0)
- [ ] 코드 clone (`~/omx_aim`)
- [ ] dialout 그룹 권한
- [ ] USB 디바이스 정보 확인
- [ ] udev rule 생성 → `/dev/omx_follower` 심볼릭 링크
- [ ] `python3 apps/keyboard_teleop.py` 동작 확인
- [ ] ROS 노드 동작 확인 (`ros2 node list` 에 `/omx_yolo_node`)
