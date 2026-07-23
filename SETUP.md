# Setup Guide

새 PC 또는 Jetson Orin Nano에서 OMX Aim 시스템을 셋업하는 가이드.

## 지원 플랫폼

- **옵션 A — 일반 PC**: Ubuntu 24.04 LTS + Python 3.12 + ROS 2 Jazzy
- **옵션 B — Jetson Orin Nano**: JetPack 7.2 (L4T 39.2) + Ubuntu 24.04 + CUDA 13.2

공통 하드웨어:
- OpenManipulator-X (with ROBOTIS OpenRB-150)

**섹션 표기:**
- 헤더에 표시가 없으면 두 플랫폼 공통
- `[A]` 또는 `[B]` 표시된 서브섹션은 해당 플랫폼만

---

## 1. 시스템 요구사항 확인

### [A] 일반 PC

이미 ROS 2 Jazzy 설치된 PC 기준. 없으면 ROS 2 공식 가이드 따라 설치.

```bash
python3 --version          # 3.12.x
ros2 --version
nvidia-smi | head -5        # GPU (있는 경우)
```

### [B] Jetson Orin Nano

JetPack 7.2 이상 필수. 확인:

```bash
sudo apt show nvidia-jetpack 2>/dev/null | grep Version    # 7.2-xxx 이상
cat /etc/nv_tegra_release                                    # R39.x
lsb_release -a                                               # 24.04
python3 --version                                            # 3.12.x
ros2 --version
```

JetPack 7.2 미만이면 이 가이드가 안 맞음. NVIDIA SDK Manager로 flash 필요.

**주의:** 젯슨에는 `nvidia-smi`가 없음 (호스트 데스크톱 GPU용 도구). 대신 `jtop` (jetson-stats) 또는 `sudo tegrastats` 사용.

---

## 2. Python 가상환경

ROS 2 Jazzy 의 rclpy 등 시스템 패키지 상속을 위해 `--system-site-packages` 필수.

```bash
python3 -m venv ~/venv/omx_ros --system-site-packages

source /opt/ros/jazzy/setup.bash
source ~/venv/omx_ros/bin/activate

which python3              # /home/USER/venv/omx_ros/bin/python3
python3 -c "import rclpy; print('OK')"
```

---

## 3. 편의 alias 추가

한 줄로 ROS + venv 활성화 + 프로젝트 폴더 이동:

```bash
cat >> ~/.bashrc << 'EOF'

# OMX Aim 환경 (ROS 2 Jazzy + venv + 젯슨 경고 억제)
alias omxenv='source /opt/ros/jazzy/setup.bash \
  && source ~/venv/omx_ros/bin/activate \
  && cd ~/omx_aim \
  && export QT_QPA_PLATFORM=offscreen \
  && export PYTHONWARNINGS="ignore::UserWarning:torch.cuda"'
EOF

source ~/.bashrc
omxenv
```

`QT_QPA_PLATFORM=offscreen`은 헤드리스 (모니터 없이) 실행 시 Qt 경고 억제.
`PYTHONWARNINGS`는 젯슨의 sm_87 경고 억제.
GUI 창 띄우고 싶으면 그 셸에서만 `unset QT_QPA_PLATFORM`.

---

## 4. Python 의존성 설치

### [A] 일반 PC

```bash
omxenv
pip install --upgrade pip

pip install lerobot ultralytics opencv-contrib-python PyYAML dynamixel-sdk
pip install "numpy<2.0"

# venv에 깔린 opencv 우선 → 시스템 ROS OpenCV 와 충돌 방지 (필요 시)
pip uninstall -y opencv-python opencv-python-headless
```

### [B] Jetson Orin Nano

JetPack 7.2 부터 상류 PyTorch SBSA wheel 지원. **CUDA 13.2 인덱스 필수**.

```bash
omxenv
pip install --upgrade pip

# PyTorch (CUDA 13.2). --index-url 없이 하면 CPU 버전 깔림. 절대 잊지 말 것.
pip install torch torchvision --index-url https://download.pytorch.org/whl/cu132

# 나머지
pip install lerobot ultralytics PyYAML dynamixel-sdk

# numpy 반드시 1.x (Ubuntu 24.04 시스템 matplotlib/OpenCV 가 numpy 1.x로 컴파일됨)
pip install "numpy<2.0"

# pip OpenCV 다 제거 (JetPack의 CUDA-OpenCV 사용, --system-site-packages 로 자동 상속)
pip uninstall -y opencv-python opencv-contrib-python opencv-python-headless
```

**Wheel에 CUDA 런타임 포함:** `cu132` wheel은 cuda-toolkit-13.2.1, cuBLAS, cuDNN을 의존성으로 자동 설치. 따라서 `/usr/local/cuda`나 `nvcc`가 PATH에 없어도 PyTorch는 GPU 씀. 학습/추론용은 이 정도로 충분.

### 검증 (공통)

```bash
python3 -c "
import cv2, torch, rclpy
from ultralytics import YOLO
from lerobot.motors import Motor, MotorNormMode
from lerobot.motors.dynamixel import DynamixelMotorsBus, OperatingMode, DriveMode
import dynamixel_sdk

print('All imports OK')
print('OpenCV:', cv2.__version__)
print('Torch:', torch.__version__, '| CUDA:', torch.cuda.is_available())
print('Numpy:', __import__('numpy').__version__)
"
```

**기대 출력 (플랫폼별):**

| 항목 | [A] PC | [B] Jetson |
|---|---|---|
| OpenCV | 4.6.0 (시스템 ROS) | 4.6.0 또는 JetPack 버전 |
| Torch | 2.x (일반) | 2.12+cu132 이상 |
| CUDA | True (GPU 있으면) | True |
| Numpy | 1.26.x | 1.26.x |

**[B] 젯슨 특이사항:** GPU 텐서 생성 시 이런 UserWarning 나오는데 **정상**임:
```
Found GPU0 Orin which is of compute capability (CC) 8.7.
```
sm_87 최적화 커널이 wheel에 빠져있어 PTX JIT로 컴파일해서 씀. 첫 실행 시 컴파일 시간 30초~2분 추가되지만 이후 캐시(`~/.nv/ComputeCache/`)로 빨라짐. `omxenv` alias에 `PYTHONWARNINGS` 넣어놨으니 경고 안 보임.

---

## 5. 코드 가져오기

```bash
cd ~
git clone https://github.com/dladyddn133-creator/omx_aim.git
cd omx_aim

python3 -c "from omx.hardware import build_bus; from omx.config import load_config; cfg = load_config(); print('Config:', cfg.motor.port)"
```

**주의:** YOLO 모델 파일(`models/best.pt`)은 `.gitignore` 대상이라 clone에 안 들어옴. 별도로 scp/HuggingFace/드라이브 등에서 받아 `~/omx_aim/models/best.pt`에 두기.

---

## 6. OMX 하드웨어 연결 셋업

### 6-1. dialout 그룹 권한

```bash
groups | grep dialout && echo "OK" || sudo usermod -aG dialout $USER
# usermod 실행했으면 재로그인 또는 재부팅 필요
```

### 6-2. USB 인식 확인

OMX 연결 + 전원 ON 후:

```bash
ls /dev/ttyACM* /dev/ttyUSB* 2>&1
```

기대 출력:
- **OpenRB-150**: `/dev/ttyACM*`
- **U2D2** (FTDI 어댑터): `/dev/ttyUSB*`

TurtleBot3 Waffle 같이 연결된 경우 OpenCR도 ttyACM으로 잡히므로 여러 개 보일 수 있음. 다음 단계에서 serial 값으로 구분.

### 6-3. USB 디바이스 정보 확인

```bash
# 각 포트 확인 (여러 개면 반복)
udevadm info -a -n /dev/ttyACM0 2>/dev/null | grep -E 'ATTRS\{(idVendor|idProduct|product|manufacturer|serial)\}' | head -10
```

OMX (OpenRB-150) 값 예시:

```
ATTRS{idVendor}=="2f5d"
ATTRS{idProduct}=="2202"
ATTRS{product}=="OpenRB-150"
ATTRS{manufacturer}=="ROBOTIS"
ATTRS{serial}=="B64D42C1503059384C2E3120FF09242C"   # ← 보드마다 다름, 본인 값 사용
```

TurtleBot3 Waffle (OpenCR) 은 `idVendor=0483, idProduct=5740`으로 구분 가능 (serial은 placeholder라 vendor+product로 매칭).

### 6-4. udev rule 만들기

**OMX (OpenRB-150):** 본인 보드의 serial 값 넣기

```bash
sudo tee /etc/udev/rules.d/99-omx-follower.rules > /dev/null << 'EOF'
# OMX-AIM follower arm - ROBOTIS OpenRB-150
SUBSYSTEM=="tty", ATTRS{idVendor}=="2f5d", ATTRS{idProduct}=="2202", ATTRS{serial}=="본인_보드_SERIAL", SYMLINK+="omx_follower", MODE="0666"
EOF
```

**TurtleBot3 Waffle (선택, OpenCR):**

```bash
sudo tee /etc/udev/rules.d/99-waffle-opencr.rules > /dev/null << 'EOF'
# TurtleBot3 Waffle - ROBOTIS OpenCR
SUBSYSTEM=="tty", ATTRS{idVendor}=="0483", ATTRS{idProduct}=="5740", SYMLINK+="opencr", MODE="0666"
EOF
```

**U2D2 사용 시:**

```
SUBSYSTEM=="tty", ATTRS{idVendor}=="0403", ATTRS{idProduct}=="6014", ATTRS{serial}=="...", SYMLINK+="omx_follower", MODE="0666"
```

### 6-5. 규칙 적용

```bash
sudo udevadm control --reload-rules
sudo udevadm trigger
# USB 뽑았다 다시 꽂으면 확실

ls -la /dev/omx_follower
```

기대: `lrwxrwxrwx ... /dev/omx_follower -> ttyACM?`

### 6-6. 통신 검증

```bash
omxenv

python3 -c "
from omx.hardware import build_bus
try:
    bus = build_bus()
    bus.connect()
    pos = bus.sync_read('Present_Position', normalize=False)
    print('OMX 연결 OK')
    for name, p in pos.items():
        print(f'  {name:14s}: {p}')
    bus.disconnect()
except Exception as e:
    print('FAIL:', type(e).__name__, ':', e)
"
```

기대: 6개 모터 (shoulder_pan, shoulder_lift, elbow_flex, wrist_flex, wrist_roll, gripper) 위치 값 출력.

---

## 7. 동작 검증

```bash
omxenv

python3 apps/keyboard_teleop.py    # 키보드 teleop
python3 apps/aim_test.py --dry-run # 좌표 조준
python3 apps/yolo_test.py --dry-run # YOLO 추적
python3 apps/yolo_node.py --dry-run # ROS 노드
```

다른 터미널에서 ROS 노드 확인:
```bash
source /opt/ros/jazzy/setup.bash
ros2 node list                     # /omx_yolo_node 보여야 정상
```

---

## 트러블슈팅

### 공통

#### USB 인식 안 됨
```bash
sudo dmesg | tail -20
# 커널 로그 확인. USB 케이블, OMX 전원 확인.
```

#### Permission denied (시리얼 포트)
```bash
groups | grep dialout
# 없으면: sudo usermod -aG dialout $USER + 재로그인
```

#### /dev/omx_follower 안 생김
```bash
cat /etc/udev/rules.d/99-omx-follower.rules
udevadm info -a -n /dev/ttyACM0 | grep serial   # serial 값 재확인
sudo udevadm control --reload-rules && sudo udevadm trigger
```

#### 모터 응답 없음
확인:
- OMX 외부 전원 (12V) ON
- 모터 데이지체인 연결
- 모터 ID 11~16 (OMX 표준)

baud rate 스캔:
```bash
python3 -c "
import dynamixel_sdk as dxl
for baud in [57600, 115200, 1000000, 2000000]:
    port = dxl.PortHandler('/dev/omx_follower')
    packet = dxl.PacketHandler(2.0)
    port.openPort(); port.setBaudRate(baud)
    found = [i for i in range(1,21) if packet.ping(port, i)[1] == dxl.COMM_SUCCESS]
    port.closePort()
    if found: print(f'baud={baud}: motors {found}')
"
```

#### Ctrl+C로 종료 후 모터 굳어있음
정상. Dynamixel은 torque 켜진 상태에서 마지막 goal position 유지함. 풀려면:
```bash
python3 -c "
from omx.hardware import build_bus
bus = build_bus(); bus.connect()
bus.disable_torque()
bus.disconnect()
print('Torque disabled')
"
```
팔 잡고 안전한 자세로 옮긴 뒤 놓기 (중력 낙하 주의).

### [B] Jetson 특화

#### `numpy.core.multiarray failed to import` 크래시
증상: `A module that was compiled using NumPy 1.x cannot be run in NumPy 2.5.1`
원인: 팀원이 pip으로 뭔가 설치하면서 numpy가 2.x로 자동 승격됨. Ubuntu 24.04 시스템 matplotlib/OpenCV는 numpy 1.x로 컴파일되어 있어 ABI 불일치.
해결:
```bash
omxenv
pip install "numpy<2.0"
```

#### `Torch: 2.x+cpu` — PyTorch가 CPU 버전으로 재설치됨
원인: 팀원이 `pip install torch` 를 인덱스 없이 실행. PyPI의 기본 wheel은 aarch64 CPU 버전.
해결:
```bash
omxenv
pip uninstall -y torch torchvision torchaudio
pip install torch torchvision --index-url https://download.pytorch.org/whl/cu132
```

#### `UserWarning: Found GPU0 Orin which is of compute capability (CC) 8.7`
**정상, 무시.** sm_87 최적화 커널이 wheel에 없어 PTX JIT로 컴파일. 실제로는 GPU 씀. `PYTHONWARNINGS`로 억제 (`omxenv` alias에 이미 포함).

#### `qt.qpa.xcb: could not connect to display`
**정상 (헤드리스 실행).** 노드 동작에는 무관. GUI 없이 돌리려면 `QT_QPA_PLATFORM=offscreen` (`omxenv` alias에 이미 포함).

#### 첫 YOLO 추론이 매우 느림 / 노드 타임아웃
sm_87 PTX JIT 컴파일 때문. 한 번 warmup 하면 캐시 저장돼서 이후 빠름:
```bash
python3 -c "
from ultralytics import YOLO
import numpy as np
m = YOLO('models/best.pt')
dummy = np.random.randint(0,255,(640,640,3),dtype=np.uint8)
m(dummy, device=0, verbose=False)   # 30초~2분 걸림 (JIT)
m(dummy, device=0, verbose=False)   # 이후 빠름
print('Warmup done. Cache at ~/.nv/ComputeCache/')
"
```

#### 팀원이 뭔가 만져서 환경 망가짐
```bash
omxenv
pip install -r requirements.jetpack72.lock \
  --index-url https://download.pytorch.org/whl/cu132 \
  --extra-index-url https://pypi.org/simple
```
그 후 검증 (섹션 4 검증 스크립트).

---

## 팀 협업 규칙 [B]

⚠️ **이 젯슨에서 절대 금지:**
- `pip install torch|numpy|opencv-python|ultralytics|lerobot` (인덱스 옵션 없이)
- 시스템 apt로 `python3-*` 패키지 새로 설치 (기존 것 변경)

✅ **의존성 추가/변경 필요하면:**
1. `omxenv && pip install ...` 로 시험 설치
2. 동작 확인
3. `pip freeze > requirements.jetpack72.lock` 로 갱신
4. Git commit + push
5. 팀원에게 슬랙/카톡으로 공지

## 환경 스냅샷 생성

지금 잘 도는 상태를 잠금:

```bash
omxenv
cd ~/omx_aim
pip freeze > requirements.jetpack72.lock
git add requirements.jetpack72.lock
git commit -m "Update JetPack 7.2 dependency lock"
git push
```

---

## 8. 캘리브레이션 (필요 시)

OMX 팔을 분해 또는 모터 펌웨어 리셋했다면:

```bash
omxenv
python3 apps/aim_test.py --measure-home
```

팔을 "곧게 정면 수평" 자세로 손으로 옮긴 뒤 실행. 출력된 raw tick 값을 `config.yaml`의 `calibration.home`에 복사.

---

## 호환성 메모

### 다른 보드 사용 시

| 보드 | 인터페이스 | 이름 | idVendor | idProduct |
|---|---|---|---|---|
| OpenRB-150 | USB CDC | ttyACM | 2f5d | 2202 |
| U2D2 | FTDI | ttyUSB | 0403 | 6014 |
| OpenCR | USB CDC | ttyACM | 0483 | 5740 |

`udevadm info` 로 본인 보드 정보 확인하고 udev rule 에 반영.

### 다른 OS/JetPack

- Ubuntu 22.04 → ROS 2 Humble 사용 권장 (Jazzy 비호환)
- Ubuntu 24.04 → ROS 2 Jazzy (현재 가이드)
- JetPack 6.x → PyTorch 설치 방식 다름 (jetson-ai-lab 인덱스 등). 이 가이드 안 맞음.
- JetPack 7.2+ → 이 가이드 (SBSA 표준 wheel 지원)

---

## 셋업 완료 체크리스트

- [ ] 시스템 요구사항 확인 (`ros2 --version`, JetPack/Ubuntu 버전)
- [ ] venv 생성 (`~/venv/omx_ros` with `--system-site-packages`)
- [ ] alias `omxenv` 추가 (QT/PYTHONWARNINGS 포함)
- [ ] Python 의존성 설치 완료
- [ ] 검증 스크립트 `All imports OK` + CUDA True [B]
- [ ] 코드 clone (`~/omx_aim`)
- [ ] models/best.pt 배치
- [ ] dialout 그룹 권한
- [ ] USB 디바이스 정보 확인 + udev rule 생성
- [ ] `/dev/omx_follower` 심볼릭 링크 확인
- [ ] 통신 검증 (모터 6개 위치값 출력)
- [ ] `python3 apps/keyboard_teleop.py` 동작 확인
- [ ] ROS 노드 동작 확인 (`ros2 node list`에 `/omx_yolo_node`)
- [ ] [B] `requirements.jetpack72.lock` 커밋