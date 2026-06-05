#!/usr/bin/env python3
"""OMX YOLO tracker as ROS 2 node (단계 2: Publisher 추가).

기능:
- YOLO 검출 + IBVS 추적 (기존 동작)
- ROS 토픽으로 상태 publish

Publish 토픽:
- /omx/status          (std_msgs/String, 1 Hz)
    "ready"      : 노드 시작됨, 동작 대기
    "tracking"   : YOLO 가 표적 검출 중, OMX 추적 중
    "lost"       : 표적 안 보임
    "paused"     : 일시정지
    "dry_run_*"  : dry-run 모드 (현재 상태 prefix)
    
- /omx/target_detected (std_msgs/Bool, 매 프레임)
    True / False
    
- /omx/error_norm      (geometry_msgs/Point, 매 프레임)
    표적 - 화면중심 정규화 오차 (-1 ~ +1)
    x, y 만 사용. z = 0
    표적 없을 때는 publish 안 함

- /omx/joint_state     (sensor_msgs/JointState, 매 프레임)
    OMX 6개 관절 현재 각도 (rad)

실행:
    omxenv
    python3 apps/yolo_node.py --dry-run         # 모터 없이
    python3 apps/yolo_node.py                    # 실제 동작

검증 (다른 터미널):
    source /opt/ros/jazzy/setup.bash
    ros2 node list                               # /omx_yolo_node 보임
    ros2 topic list                              # /omx/* 토픽들 보임
    ros2 topic echo /omx/status                  # 1초마다 상태
    ros2 topic echo /omx/error_norm              # 검출 시 오차
    ros2 topic hz /omx/target_detected           # 주기 측정

키 조작 (카메라 창 활성 시):
    p   : OMX 명령 일시정지 (검출은 계속)
    h   : Home 복귀
    ESC : 종료
"""

from __future__ import annotations

import sys
from pathlib import Path

# omx 패키지 import 가능하게 프로젝트 루트 추가
sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

import math
import time

import cv2
import rclpy
from rclpy.node import Node

from std_msgs.msg import String, Bool
from geometry_msgs.msg import Point
from sensor_msgs.msg import JointState

from ultralytics import YOLO

from omx.hardware import build_bus, get_dxl_symbols, ARM_MOTORS, MOTOR_ORDER
from omx.config import load_config, Config


# ===========================================================
# 상수
# ===========================================================

TICKS_PER_REV = 4096
RAD2TICK = TICKS_PER_REV / (2.0 * math.pi)


# ===========================================================
# OMX 제어
# ===========================================================

class TrackController:
    """OMX 모터 제어 wrapper."""

    def __init__(self, cfg: Config, dry_run: bool = False, logger=None):
        self.cfg = cfg
        self.dry_run = dry_run
        self.bus = None if dry_run else build_bus(cfg.motor.port)
        self.yaw = 0.0
        self.pitch = 0.0
        self.logger = logger

    def _log(self, msg, level="info"):
        if self.logger is not None:
            getattr(self.logger, level)(msg)
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
        """IBVS 한 step 만큼 회전."""
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

    def read_joint_positions_rad(self) -> dict[str, float]:
        """현재 모든 관절 각도 (rad). dry-run 시 내부 상태만."""
        if self.dry_run or self.bus is None or not self.bus.is_connected:
            return {
                "shoulder_pan":  self.yaw,
                "shoulder_lift": self.pitch,
                "elbow_flex":    0.0,
                "wrist_flex":    0.0,
                "wrist_roll":    0.0,
                "gripper":       0.0,
            }
        
        ticks = self.bus.sync_read("Present_Position", normalize=False)
        home = self.cfg.calibration.home
        sign = self.cfg.calibration.sign
        result = {}
        for name in MOTOR_ORDER:
            if name == "gripper":
                result[name] = float(ticks[name])
                continue
            result[name] = (ticks[name] - home[name]) / RAD2TICK / sign.get(name, 1)
        return result


# ===========================================================
# ROS 2 Node
# ===========================================================

class OmxYoloNode(Node):
    def __init__(self, dry_run: bool = False):
        super().__init__('omx_yolo_node')
        
        # ----- 1. Config -----
        self.cfg = load_config()
        self.get_logger().info(f"Config loaded. port={self.cfg.motor.port}")
        
        # ----- 2. Camera -----
        cam_idx = self.cfg.ibvs.camera_index
        self.cap = cv2.VideoCapture(cam_idx)
        if not self.cap.isOpened():
            raise RuntimeError(f"카메라 {cam_idx} 열기 실패")
        self.get_logger().info(f"카메라 {cam_idx} 열림")
        
        # ----- 3. YOLO -----
        self.model = YOLO(self.cfg.yolo.model_path)
        self.target_class = self.cfg.yolo.target_class
        self.class_name = self.model.names.get(
            self.target_class, f"cls_{self.target_class}")
        self.get_logger().info(f"YOLO 로드: {self.cfg.yolo.model_path}")
        self.get_logger().info(
            f"검출 클래스: {self.target_class} ({self.class_name})")
        
        # ----- 4. OMX -----
        self.ctrl = TrackController(self.cfg, dry_run=dry_run,
                                     logger=self.get_logger())
        self.ctrl.connect()
        self.ctrl.go_home()
        
        # ----- 5. State -----
        self.dry_run = dry_run
        self.paused = False
        self.last_control_t = 0.0
        self.control_period = 1.0 / self.cfg.ibvs.control_hz
        self.current_status = "ready"
        
        # FPS
        self.fps_t = time.time()
        self.fps_n = 0
        self.fps_disp = 0.0
        
        # ----- 6. ROS Publishers -----
        self.pub_status = self.create_publisher(String, '/omx/status', 10)
        self.pub_detected = self.create_publisher(Bool, '/omx/target_detected', 10)
        self.pub_error = self.create_publisher(Point, '/omx/error_norm', 10)
        self.pub_joint = self.create_publisher(JointState, '/omx/joint_state', 10)
        
        # ----- 7. Timer -----
        self.timer = self.create_timer(self.control_period, self.loop)
        self.status_timer = self.create_timer(1.0, self.publish_status)
        
        self.get_logger().info(
            f"Timer: 메인 {self.cfg.ibvs.control_hz} Hz, 상태 1 Hz")
        self.get_logger().info("=== Node ready ===")
        self.get_logger().info(
            "Publish: /omx/status, /omx/target_detected, "
            "/omx/error_norm, /omx/joint_state")

    # -------------------------------------------------------
    # Publishers
    # -------------------------------------------------------

    def publish_status(self):
        """1 Hz: 현재 상태."""
        msg = String()
        if self.paused:
            msg.data = "paused"
        elif self.dry_run:
            msg.data = f"dry_run_{self.current_status}"
        else:
            msg.data = self.current_status
        self.pub_status.publish(msg)

    def publish_detected(self, detected: bool):
        msg = Bool()
        msg.data = detected
        self.pub_detected.publish(msg)

    def publish_error(self, ex: float, ey: float):
        msg = Point()
        msg.x = float(ex)
        msg.y = float(ey)
        msg.z = 0.0
        self.pub_error.publish(msg)

    def publish_joint_state(self):
        try:
            positions = self.ctrl.read_joint_positions_rad()
        except Exception:
            return
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = list(positions.keys())
        msg.position = list(positions.values())
        self.pub_joint.publish(msg)

    # -------------------------------------------------------
    # 메인 루프
    # -------------------------------------------------------

    def loop(self):
        ok, frame = self.cap.read()
        if not ok:
            self.get_logger().warn("프레임 읽기 실패")
            return

        h, w = frame.shape[:2]
        cx, cy = w / 2.0, h / 2.0
        deadband = self.cfg.ibvs.deadband

        # 화면 가이드
        cv2.drawMarker(frame, (int(cx), int(cy)),
                       (0, 255, 255), cv2.MARKER_CROSS, 20, 1)
        dz_x = int(deadband * cx)
        dz_y = int(deadband * cy)
        cv2.rectangle(frame,
                      (int(cx) - dz_x, int(cy) - dz_y),
                      (int(cx) + dz_x, int(cy) + dz_y),
                      (80, 80, 80), 1)

        # YOLO 추론
        results = self.model.predict(
            frame,
            imgsz=self.cfg.yolo.imgsz,
            conf=self.cfg.yolo.conf_threshold,
            classes=[self.target_class],
            verbose=False,
        )
        boxes = results[0].boxes
        error_norm = None
        detected = False

        if boxes is not None and len(boxes) > 0:
            confs = boxes.conf.cpu().numpy()
            idx = confs.argmax()
            xyxy = boxes.xyxy[idx].cpu().numpy()
            x1, y1, x2, y2 = [int(v) for v in xyxy]
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
            detected = True
            self.current_status = "tracking"
        else:
            cv2.putText(frame, f"NO {self.class_name.upper()}",
                        (10, 60), cv2.FONT_HERSHEY_SIMPLEX,
                        0.7, (0, 0, 255), 2)
            self.current_status = "lost"

        # ROS publish
        self.publish_detected(detected)
        if error_norm is not None:
            self.publish_error(error_norm[0], error_norm[1])
        self.publish_joint_state()

        # 화면 텍스트
        status_txt = ("PAUSED" if self.paused
                      else ("DRY-RUN" if self.dry_run else "LIVE"))
        cv2.putText(frame,
                    f"[{status_txt}] yaw={math.degrees(self.ctrl.yaw):+.1f} "
                    f"pitch={math.degrees(self.ctrl.pitch):+.1f} "
                    f"fps={self.fps_disp:.1f}",
                    (10, 25), cv2.FONT_HERSHEY_SIMPLEX,
                    0.6, (255, 255, 255), 2)
        if error_norm is not None:
            cv2.putText(frame,
                        f"err=({error_norm[0]:+.2f}, {error_norm[1]:+.2f})",
                        (10, h - 40), cv2.FONT_HERSHEY_SIMPLEX,
                        0.55, (255, 255, 255), 2)
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

        # IBVS step
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

        cv2.imshow("OMX YOLO node", frame)
        key = cv2.waitKey(1) & 0xFF
        if key == 27:
            self.get_logger().info("ESC. 종료.")
            rclpy.shutdown()
        elif key == ord('p'):
            self.paused = not self.paused
            self.get_logger().info("일시정지" if self.paused else "재개")
        elif key == ord('h'):
            self.get_logger().info("Home 복귀")
            self.ctrl.go_home()

    # -------------------------------------------------------
    # Cleanup
    # -------------------------------------------------------

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
    cli_args, ros_args = parser.parse_known_args()
    
    rclpy.init(args=ros_args)
    
    try:
        node = OmxYoloNode(dry_run=cli_args.dry_run)
        try:
            rclpy.spin(node)
        finally:
            node.destroy_node()
    except KeyboardInterrupt:
        print("\n중단됨.")
    except Exception as e:
        print(f"노드 에러: {type(e).__name__}: {e}")
        import traceback
        traceback.print_exc()
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()