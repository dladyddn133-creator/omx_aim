#!/usr/bin/env python3
"""OMX-Follower YOLO tracking (단일 클래스).

omx_track_test.py 의 마우스 ROI 트래킹을 YOLO 검출로 교체.
지정된 한 클래스만 검출하고, 여러 개면 confidence 가장 높은 것을 추적.

config.yaml 에 다음 섹션 추가 필요:

  yolo:
    model_path: yolov8n.pt    # 또는 ./models/best.pt
    target_class: 67          # 검출할 클래스 ID (67=cell phone)
    conf_threshold: 0.5       # 최소 confidence
    imgsz: 640                # 추론 해상도 (낮을수록 빠름)

키 조작:
  p   : OMX 명령 일시정지 (검출은 계속)
  h   : Home 복귀
  ESC : 종료
"""

from __future__ import annotations

import argparse
import math
import sys
import time

from omx.hardware import build_bus, get_dxl_symbols, ARM_MOTORS
from omx.config import load_config, Config

try:
    import cv2
except ImportError:
    print("OpenCV 가 필요합니다: pip install opencv-contrib-python")
    raise

try:
    from ultralytics import YOLO
except ImportError:
    print("ultralytics 가 필요합니다: pip install ultralytics")
    raise


TICKS_PER_REV = 4096
RAD2TICK = TICKS_PER_REV / (2.0 * math.pi)


# ===========================================================
# OMX 제어 (omx_track_test.py 와 동일)
# ===========================================================

class TrackController:
    def __init__(self, cfg: Config, dry_run: bool = False):
        self.cfg = cfg
        self.dry_run = dry_run
        self.bus = None if dry_run else build_bus(cfg.motor.port)
        self.yaw = 0.0
        self.pitch = 0.0

    def connect(self) -> None:
        if self.dry_run:
            print("[dry-run] OMX 연결 생략.")
            return
        s = get_dxl_symbols()
        OperatingMode = s["OperatingMode"]

        self.bus.connect()
        with self.bus.torque_disabled():
            self.bus.configure_motors(return_delay_time=0)
            for m in ARM_MOTORS:
                self.bus.write("Operating_Mode", m,
                               OperatingMode.EXTENDED_POSITION.value,
                               normalize=False)
            for m in ARM_MOTORS:
                self.bus.write("Profile_Velocity", m,
                               self.cfg.motor.profile_velocity, normalize=False)
                self.bus.write("Profile_Acceleration", m,
                               self.cfg.motor.profile_acceleration, normalize=False)
        self.bus.enable_torque(num_retry=3)
        print("OMX 연결됨.")

    def disconnect(self) -> None:
        if self.dry_run or self.bus is None or not self.bus.is_connected:
            return
        self.bus.disconnect(disable_torque=True)

    def go_home(self) -> None:
        if self.dry_run:
            print("[dry-run] Home 으로 이동했다고 가정.")
            self.yaw = 0.0
            self.pitch = 0.0
            return
        home = self.cfg.calibration.home
        for m in ARM_MOTORS:
            self.bus.write("Goal_Position", m, home[m], normalize=False)
        time.sleep(2.0)
        self.yaw = 0.0
        self.pitch = 0.0
        print("Home 도달.")

    def step(self, delta_yaw: float, delta_pitch: float) -> tuple[float, float]:
        max_step = self.cfg.safety.max_step_rad
        delta_yaw = max(-max_step, min(max_step, delta_yaw))
        delta_pitch = max(-max_step, min(max_step, delta_pitch))

        new_yaw = self.yaw + delta_yaw
        new_pitch = self.pitch + delta_pitch

        limits = self.cfg.safety.angle_limits_rad
        lo, hi = limits["shoulder_pan"]
        new_yaw = max(lo, min(hi, new_yaw))
        lo, hi = limits["shoulder_lift"]
        new_pitch = max(lo, min(hi, new_pitch))

        self.yaw = new_yaw
        self.pitch = new_pitch

        if not self.dry_run:
            home = self.cfg.calibration.home
            sign = self.cfg.calibration.sign
            yaw_tick = int(round(home["shoulder_pan"]
                                 + sign["shoulder_pan"] * new_yaw * RAD2TICK))
            pitch_tick = int(round(home["shoulder_lift"]
                                   + sign["shoulder_lift"] * new_pitch * RAD2TICK))
            self.bus.write("Goal_Position", "shoulder_pan",
                           yaw_tick, normalize=False)
            self.bus.write("Goal_Position", "shoulder_lift",
                           pitch_tick, normalize=False)

        return new_yaw, new_pitch


# ===========================================================
# YOLO 검출
# ===========================================================

class YoloDetector:
    """단일 클래스만 검출하는 간단한 wrapper."""

    def __init__(self, model_path: str, target_class: int,
                 conf_threshold: float = 0.5, imgsz: int = 640):
        print(f"YOLO 모델 로드: {model_path}")
        self.model = YOLO(model_path)
        self.target_class = target_class
        self.conf_threshold = conf_threshold
        self.imgsz = imgsz
        self.class_name = self.model.names.get(target_class, f"class_{target_class}")
        print(f"검출 클래스: {target_class} ({self.class_name})")

    def detect(self, frame) -> tuple[int, int, int, int, float] | None:
        """프레임에서 target_class 의 가장 confident 한 박스 반환.
        
        반환: (x, y, w, h, conf) 또는 표적 없으면 None
        """
        results = self.model.predict(
            frame,
            imgsz=self.imgsz,
            conf=self.conf_threshold,
            classes=[self.target_class],  # 이 클래스만 추론 결과에 포함
            verbose=False,
        )
        
        # 첫 번째 결과만 사용 (single image)
        boxes = results[0].boxes
        if boxes is None or len(boxes) == 0:
            return None
        
        # confidence 가장 높은 것 선택
        confs = boxes.conf.cpu().numpy()
        best_idx = confs.argmax()
        
        # xyxy 형식 (x1, y1, x2, y2) -> xywh 변환
        xyxy = boxes.xyxy[best_idx].cpu().numpy()
        x1, y1, x2, y2 = xyxy
        x, y = int(x1), int(y1)
        w, h = int(x2 - x1), int(y2 - y1)
        conf = float(confs[best_idx])
        
        return (x, y, w, h, conf)


# ===========================================================
# 메인 루프
# ===========================================================

def run(cfg: Config, dry_run: bool) -> int:
    cam_idx = cfg.ibvs.camera_index
    cap = cv2.VideoCapture(cam_idx)
    if not cap.isOpened():
        print(f"카메라 {cam_idx} 열기 실패.")
        return 1
    print("카메라 열림.")

    # YOLO 검출기 초기화
    yolo_cfg = cfg.yolo
    detector = YoloDetector(
        model_path=yolo_cfg.model_path,
        target_class=yolo_cfg.target_class,
        conf_threshold=yolo_cfg.conf_threshold,
        imgsz=yolo_cfg.imgsz,
    )

    ctrl = TrackController(cfg, dry_run=dry_run)
    try:
        ctrl.connect()
        ctrl.go_home()
    except Exception as e:
        print(f"OMX 초기화 실패: {e}")
        cap.release()
        return 1

    paused = False
    last_control_t = 0.0
    control_period = 1.0 / cfg.ibvs.control_hz
    deadband = cfg.ibvs.deadband

    # FPS 측정용
    fps_t = time.time()
    fps_n = 0
    fps_disp = 0.0

    try:
        while True:
            ok, frame = cap.read()
            if not ok:
                print("프레임 읽기 실패.")
                break

            h, w = frame.shape[:2]
            cx, cy = w / 2.0, h / 2.0

            # 화면 중심 + 데드밴드 표시
            cv2.drawMarker(frame, (int(cx), int(cy)),
                           (0, 255, 255), cv2.MARKER_CROSS, 20, 1)
            dz_x = int(deadband * cx)
            dz_y = int(deadband * cy)
            cv2.rectangle(frame,
                          (int(cx) - dz_x, int(cy) - dz_y),
                          (int(cx) + dz_x, int(cy) + dz_y),
                          (80, 80, 80), 1)

            # YOLO 검출
            detection = detector.detect(frame)
            error_norm = None

            if detection is not None:
                x, y, bw, bh, conf = detection
                cv2.rectangle(frame, (x, y), (x + bw, y + bh),
                              (0, 255, 0), 2)
                obj_x = x + bw / 2.0
                obj_y = y + bh / 2.0
                cv2.circle(frame, (int(obj_x), int(obj_y)),
                           4, (0, 255, 0), -1)
                cv2.line(frame, (int(cx), int(cy)),
                         (int(obj_x), int(obj_y)),
                         (0, 200, 0), 1)
                cv2.putText(frame,
                            f"{detector.class_name} {conf:.2f}",
                            (x, max(y - 8, 16)),
                            cv2.FONT_HERSHEY_SIMPLEX,
                            0.6, (0, 255, 0), 2)
                error_norm = ((obj_x - cx) / cx, (obj_y - cy) / cy)
            else:
                cv2.putText(frame, f"NO {detector.class_name.upper()}",
                            (10, 60), cv2.FONT_HERSHEY_SIMPLEX,
                            0.7, (0, 0, 255), 2)

            # 상태 표시
            status = "PAUSED" if paused else ("DRY-RUN" if dry_run else "LIVE")
            cv2.putText(frame,
                        f"[{status}]  yaw={math.degrees(ctrl.yaw):+.1f}deg "
                        f" pitch={math.degrees(ctrl.pitch):+.1f}deg  "
                        f"fps={fps_disp:.1f}",
                        (10, 25), cv2.FONT_HERSHEY_SIMPLEX,
                        0.6, (255, 255, 255), 2)
            if error_norm is not None:
                cv2.putText(frame,
                            f"err=({error_norm[0]:+.2f}, {error_norm[1]:+.2f})",
                            (10, h - 15), cv2.FONT_HERSHEY_SIMPLEX,
                            0.6, (255, 255, 255), 2)
            cv2.putText(frame, "p:pause  h:home  ESC:quit",
                        (10, h - 40), cv2.FONT_HERSHEY_SIMPLEX,
                        0.5, (180, 180, 180), 1)

            # FPS 계산
            fps_n += 1
            now = time.time()
            if now - fps_t >= 1.0:
                fps_disp = fps_n / (now - fps_t)
                fps_t = now
                fps_n = 0

            # 제어 step
            if (error_norm is not None and not paused
                    and now - last_control_t >= control_period):
                ex, ey = error_norm
                if abs(ex) < deadband:
                    ex = 0.0
                if abs(ey) < deadband:
                    ey = 0.0
                if ex != 0.0 or ey != 0.0:
                    delta_yaw = cfg.ibvs.sign_vs_x * cfg.ibvs.kp_yaw * ex
                    delta_pitch = cfg.ibvs.sign_vs_y * cfg.ibvs.kp_pitch * ey
                    ctrl.step(delta_yaw, delta_pitch)
                last_control_t = now

            cv2.imshow("OMX YOLO tracker", frame)

            key = cv2.waitKey(1) & 0xFF
            if key == 27:  # ESC
                break
            elif key == ord("p"):
                paused = not paused
                print("일시정지" if paused else "재개")
            elif key == ord("h"):
                try:
                    ctrl.go_home()
                except KeyboardInterrupt:
                    print("Home 이동 중단.")
    finally:
        cap.release()
        cv2.destroyAllWindows()
        ctrl.disconnect()

    return 0


# ===========================================================
# Entry
# ===========================================================

def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(
        description="OMX YOLO tracking (single class).",
        formatter_class=argparse.RawTextHelpFormatter,
    )
    p.add_argument("--config", default=None,
                   help="config.yaml 경로 (default: ./config.yaml)")
    p.add_argument("--dry-run", action="store_true",
                   help="OMX 없이 카메라+검출만")
    return p.parse_args()


def main() -> int:
    args = parse_args()
    try:
        cfg = load_config(args.config)
    except (FileNotFoundError, ValueError) as e:
        print(f"Config 로드 실패: {e}", file=sys.stderr)
        return 1
    
    # yolo 섹션 있는지 확인
    if not hasattr(cfg, "yolo"):
        print("config.yaml 에 yolo 섹션이 필요합니다.", file=sys.stderr)
        print("아래를 config.yaml 에 추가하세요:\n", file=sys.stderr)
        print("yolo:", file=sys.stderr)
        print("  model_path: yolov8n.pt", file=sys.stderr)
        print("  target_class: 67          # 67=cell phone, 39=bottle, 0=person", file=sys.stderr)
        print("  conf_threshold: 0.5", file=sys.stderr)
        print("  imgsz: 640", file=sys.stderr)
        return 1
    
    return run(cfg, args.dry_run)


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except KeyboardInterrupt:
        print("\n중단됨.")
        sys.exit(0)