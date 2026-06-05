#!/usr/bin/env python3
"""OMX YOLO tracker as ROS 2 node (단계 1: 최소 wrap).

기존 yolo_test.py 와 동작 동일.
ROS 2 의 rclpy.Node 로 감싸기만 함.

토픽은 아직 없음 (다음 단계에서 추가).
ros2 node list 에 'omx_yolo_node' 로 보이는 게 이 단계의 목표.

실행:
    omxenv      # ROS + venv 활성화
    python3 apps/yolo_node.py
    
    # 다른 터미널에서 확인
    ros2 node list
"""

from __future__ import annotations

import sys
from pathlib import Path

# omx 패키지를 import 할 수 있게 프로젝트 루트 추가
sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

import math
import time

import cv2
import rclpy
from rclpy.node import Node

from ultralytics import YOLO

from omx.hardware import build_bus, get_dxl_symbols, ARM_MOTORS
from omx.config import load_config, Config


TICKS_PER_REV = 4096
RAD2TICK = TICKS_PER_REV / (2.0 * math.pi)


# ===========================================================
# OMX 제어 (기존 TrackController 와 동일)
# ===========================================================

class TrackController:
    """기존 yolo_test.py 의 TrackController 와 동일. 나중에 omx/ 패키지로 옮길 예정."""
    
    def __init__(self, cfg: Config, dry_run: bool = False, logger=None):
        self.cfg = cfg
        self.dry_run = dry_run
        self.bus = None if dry_run else build_bus(cfg.motor.port)
        self.yaw = 0.0
        self.pitch = 0.0
        self.logger = logger

    def _log(self, msg):
        if self.logger:
            self.logger.info(msg)
        else:
            print(msg)

    def connect(self) -> None:
        if self.dry_run:
            self._log("[dry-run] OMX 연결 생략")
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
        self._log("OMX 연결 완료")

    def disconnect(self) -> None:
        if self.dry_run or self.bus is None or not self.bus.is_connected:
            return
        self.bus.disconnect(disable_torque=True)
        self._log("OMX 연결 해제")

    def go_home(self) -> None:
        if self.dry_run:
            self.yaw = 0.0
            self.pitch = 0.0
            self._log("[dry-run] Home 이동 시뮬레이션")
            return
        home = self.cfg.calibration.home
        for m in ARM_MOTORS:
            self.bus.write("Goal_Position", m, home[m], normalize=False)
        time.sleep(2.0)
        self.yaw = 0.0
        self.pitch = 0.0
        self._log("Home 도달")

    def step(self, delta_yaw: float, delta_pitch: float):
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


# ===========================================================
# ROS 2 Node
# ===========================================================

class OmxYoloNode(Node):
    def __init__(self, dry_run: bool = False):
        super().__init__('omx_yolo_node')
        
        # 1) Config 로드
        self.cfg = load_config()
        self.get_logger().info(f"Config loaded. port={self.cfg.motor.port}")
        
        # 2) 카메라 초기화
        cam_idx = self.cfg.ibvs.camera_index
        self.cap = cv2.VideoCapture(cam_idx)
        if not self.cap.isOpened():
            raise RuntimeError(f"카메라 {cam_idx} 열기 실패")
        self.get_logger().info(f"카메라 {cam_idx} 열림")
        
        # 3) YOLO 모델 로드
        self.model = YOLO(self.cfg.yolo.model_path)
        self.target_class = self.cfg.yolo.target_class
        cls_name = self.model.names.get(self.target_class, f"cls_{self.target_class}")
        self.get_logger().info(f"YOLO 모델 로드: {self.cfg.yolo.model_path}")
        self.get_logger().info(f"검출 클래스: {self.target_class} ({cls_name})")
        self.class_name = cls_name
        
        # 4) OMX 컨트롤러 (logger 넘김)
        self.ctrl = TrackController(self.cfg, dry_run=dry_run,
                                     logger=self.get_logger())
        self.ctrl.connect()
        self.ctrl.go_home()
        
        # 5) 상태
        self.dry_run = dry_run
        self.paused = False
        self.last_control_t = 0.0
        self.control_period = 1.0 / self.cfg.ibvs.control_hz
        
        # FPS
        self.fps_t = time.time()
        self.fps_n = 0
        self.fps_disp = 0.0
        
        # 6) 메인 루프 타이머 (control_hz 로 호출)
        self.timer = self.create_timer(self.control_period, self.loop)
        self.get_logger().info(
            f"Timer 시작: {self.cfg.ibvs.control_hz} Hz")
        self.get_logger().info("=== Node ready ===")

    def loop(self):
        """타이머 콜백 - 매 프레임 처리."""
        ok, frame = self.cap.read()
        if not ok:
            self.get_logger().warn("프레임 읽기 실패")
            return

        h, w = frame.shape[:2]
        cx, cy = w / 2.0, h / 2.0
        deadband = self.cfg.ibvs.deadband

        # 화면 중심 + deadband 표시
        cv2.drawMarker(frame, (int(cx), int(cy)),
                       (0, 255, 255), cv2.MARKER_CROSS, 20, 1)
        dz_x = int(deadband * cx)
        dz_y = int(deadband * cy)
        cv2.rectangle(frame,
                      (int(cx) - dz_x, int(cy) - dz_y),
                      (int(cx) + dz_x, int(cy) + dz_y),
                      (80, 80, 80), 1)

        # YOLO 검출
        results = self.model.predict(
            frame,
            imgsz=self.cfg.yolo.imgsz,
            conf=self.cfg.yolo.conf_threshold,
            classes=[self.target_class],
            verbose=False,
        )
        boxes = results[0].boxes
        error_norm = None

        if boxes is not None and len(boxes) > 0:
            confs = boxes.conf.cpu().numpy()
            idx = confs.argmax()
            xyxy = boxes.xyxy[idx].cpu().numpy()
            x1, y1, x2, y2 = [int(v) for v in xyxy]
            bw, bh = x2 - x1, y2 - y1
            conf = float(confs[idx])

            cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
            obj_x = (x1 + x2) / 2.0
            obj_y = (y1 + y2) / 2.0
            cv2.circle(frame, (int(obj_x), int(obj_y)),
                       4, (0, 255, 0), -1)
            cv2.line(frame, (int(cx), int(cy)),
                     (int(obj_x), int(obj_y)),
                     (0, 200, 0), 1)
            cv2.putText(frame, f"{self.class_name} {conf:.2f}",
                        (x1, max(y1 - 8, 16)),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.6, (0, 255, 0), 2)
            error_norm = ((obj_x - cx) / cx, (obj_y - cy) / cy)
        else:
            cv2.putText(frame, f"NO {self.class_name.upper()}",
                        (10, 60), cv2.FONT_HERSHEY_SIMPLEX,
                        0.7, (0, 0, 255), 2)

        # 상태 표시
        status = "PAUSED" if self.paused else ("DRY-RUN" if self.dry_run else "LIVE")
        cv2.putText(frame,
                    f"[{status}] yaw={math.degrees(self.ctrl.yaw):+.1f}deg "
                    f"pitch={math.degrees(self.ctrl.pitch):+.1f}deg "
                    f"fps={self.fps_disp:.1f}",
                    (10, 25), cv2.FONT_HERSHEY_SIMPLEX,
                    0.6, (255, 255, 255), 2)
        cv2.putText(frame, "p:pause  h:home  ESC:quit",
                    (10, h - 15), cv2.FONT_HERSHEY_SIMPLEX,
                    0.5, (180, 180, 180), 1)

        # FPS
        self.fps_n += 1
        now = time.time()
        if now - self.fps_t >= 1.0:
            self.fps_disp = self.fps_n / (now - self.fps_t)
            self.fps_t = now
            self.fps_n = 0

        # IBVS 제어
        if (error_norm is not None and not self.paused
                and now - self.last_control_t >= self.control_period):
            ex, ey = error_norm
            if abs(ex) < deadband:
                ex = 0.0
            if abs(ey) < deadband:
                ey = 0.0
            if ex != 0.0 or ey != 0.0:
                delta_yaw = self.cfg.ibvs.sign_vs_x * self.cfg.ibvs.kp_yaw * ex
                delta_pitch = self.cfg.ibvs.sign_vs_y * self.cfg.ibvs.kp_pitch * ey
                self.ctrl.step(delta_yaw, delta_pitch)
            self.last_control_t = now

        # 화면 출력
        cv2.imshow("OMX YOLO node", frame)
        key = cv2.waitKey(1) & 0xFF
        if key == 27:  # ESC
            self.get_logger().info("ESC 눌림. 종료")
            rclpy.shutdown()
        elif key == ord('p'):
            self.paused = not self.paused
            self.get_logger().info("일시정지" if self.paused else "재개")
        elif key == ord('h'):
            self.ctrl.go_home()

    def destroy_node(self):
        if hasattr(self, 'cap') and self.cap:
            self.cap.release()
        cv2.destroyAllWindows()
        if hasattr(self, 'ctrl'):
            self.ctrl.disconnect()
        super().destroy_node()


# ===========================================================
# Entry
# ===========================================================

def main(args=None):
    import argparse
    parser = argparse.ArgumentParser(description="OMX YOLO ROS 2 node")
    parser.add_argument("--dry-run", action="store_true",
                        help="OMX 없이 카메라 + 검출만")
    cli_args, _ = parser.parse_known_args()
    
    rclpy.init(args=args)
    
    try:
        node = OmxYoloNode(dry_run=cli_args.dry_run)
        try:
            rclpy.spin(node)
        finally:
            node.destroy_node()
    except KeyboardInterrupt:
        print("\n중단됨")
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()