#!/usr/bin/env python3
"""OMX-Follower visual tracking (config.yaml 적용).

변경점:
- 모든 IBVS / 캘리브 / 안전 값을 config.yaml 에서 로드
- omx_aim_test 에서 import 하던 상수들 제거 (이제 둘 다 같은 config 사용)
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


TICKS_PER_REV = 4096
RAD2TICK = TICKS_PER_REV / (2.0 * math.pi)


# ===========================================================
# Tracker 헬퍼
# ===========================================================

def create_tracker():
    for fn_name in ("TrackerCSRT_create", "TrackerCSRT.create"):
        obj = cv2
        try:
            for part in fn_name.split("."):
                obj = getattr(obj, part)
            return obj()
        except AttributeError:
            continue
    try:
        return cv2.legacy.TrackerCSRT_create()
    except AttributeError:
        raise RuntimeError(
            "TrackerCSRT 를 찾을 수 없습니다. "
            "pip install opencv-contrib-python 로 설치하세요."
        )


# ===========================================================
# OMX 제어 (joint1, joint2 만 사용)
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
# 메인 루프
# ===========================================================

def run(cfg: Config, dry_run: bool) -> int:
    cam_idx = cfg.ibvs.camera_index
    cap = cv2.VideoCapture(cam_idx)
    if not cap.isOpened():
        print(f"카메라 {cam_idx} 열기 실패.")
        return 1
    print("카메라 열림. 's' 로 ROI 선택.")

    ctrl = TrackController(cfg, dry_run=dry_run)
    try:
        ctrl.connect()
        ctrl.go_home()
    except Exception as e:
        print(f"OMX 초기화 실패: {e}")
        cap.release()
        return 1

    tracker = None
    paused = False
    last_control_t = 0.0
    control_period = 1.0 / cfg.ibvs.control_hz
    deadband = cfg.ibvs.deadband

    try:
        while True:
            ok, frame = cap.read()
            if not ok:
                print("프레임 읽기 실패.")
                break

            h, w = frame.shape[:2]
            cx, cy = w / 2.0, h / 2.0

            cv2.drawMarker(frame, (int(cx), int(cy)),
                           (0, 255, 255), cv2.MARKER_CROSS, 20, 1)
            dz_x = int(deadband * cx)
            dz_y = int(deadband * cy)
            cv2.rectangle(frame,
                          (int(cx) - dz_x, int(cy) - dz_y),
                          (int(cx) + dz_x, int(cy) + dz_y),
                          (80, 80, 80), 1)

            error_norm = None
            if tracker is not None:
                success, bbox = tracker.update(frame)
                if success:
                    x, y, bw, bh = [int(v) for v in bbox]
                    cv2.rectangle(frame, (x, y), (x + bw, y + bh),
                                  (0, 255, 0), 2)
                    obj_x = x + bw / 2.0
                    obj_y = y + bh / 2.0
                    cv2.circle(frame, (int(obj_x), int(obj_y)),
                               4, (0, 255, 0), -1)
                    cv2.line(frame, (int(cx), int(cy)),
                             (int(obj_x), int(obj_y)),
                             (0, 200, 0), 1)
                    error_norm = ((obj_x - cx) / cx, (obj_y - cy) / cy)
                else:
                    cv2.putText(frame, "TRACKING LOST",
                                (10, 60), cv2.FONT_HERSHEY_SIMPLEX,
                                0.7, (0, 0, 255), 2)

            status = "PAUSED" if paused else ("DRY-RUN" if dry_run else "LIVE")
            cv2.putText(frame,
                        f"[{status}]  yaw={math.degrees(ctrl.yaw):+.1f}deg "
                        f" pitch={math.degrees(ctrl.pitch):+.1f}deg",
                        (10, 25), cv2.FONT_HERSHEY_SIMPLEX,
                        0.6, (255, 255, 255), 2)
            if error_norm is not None:
                cv2.putText(frame,
                            f"err=({error_norm[0]:+.2f}, {error_norm[1]:+.2f})",
                            (10, h - 15), cv2.FONT_HERSHEY_SIMPLEX,
                            0.6, (255, 255, 255), 2)
            cv2.putText(frame, "s:select  p:pause  r:reset  h:home  ESC:quit",
                        (10, h - 40), cv2.FONT_HERSHEY_SIMPLEX,
                        0.5, (180, 180, 180), 1)

            now = time.time()
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

            cv2.imshow("OMX tracker", frame)

            key = cv2.waitKey(1) & 0xFF
            if key == 27:
                break
            elif key == ord("s"):
                print("ROI 박스 드래그 후 SPACE/Enter, 취소는 'c'")
                bbox = cv2.selectROI("OMX tracker", frame,
                                     showCrosshair=True, fromCenter=False)
                if bbox != (0, 0, 0, 0):
                    tracker = create_tracker()
                    tracker.init(frame, bbox)
                    print(f"트래커 시작: bbox={bbox}")
                else:
                    print("ROI 취소.")
            elif key == ord("p"):
                paused = not paused
                print("일시정지" if paused else "재개")
            elif key == ord("r"):
                tracker = None
                print("트래커 리셋.")
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
        description="OMX visual tracking (IBVS).",
        formatter_class=argparse.RawTextHelpFormatter,
    )
    p.add_argument("--config", default=None,
                   help="config.yaml 경로 (default: ./config.yaml)")
    p.add_argument("--dry-run", action="store_true",
                   help="OMX 없이 카메라+트래킹만")
    return p.parse_args()


def main() -> int:
    args = parse_args()
    try:
        cfg = load_config(args.config)
    except (FileNotFoundError, ValueError) as e:
        print(f"Config 로드 실패: {e}", file=sys.stderr)
        return 1
    return run(cfg, args.dry_run)


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except KeyboardInterrupt:
        print("\n중단됨.")
        sys.exit(0)