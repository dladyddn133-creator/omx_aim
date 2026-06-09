#!/usr/bin/env python3
"""OMX YOLO tracker with state machine and fire control.

상태 머신:
    IDLE        - 좌표 받음 (AIMING) 또는 armed & 검출 (TRACKING)
    AIMING      - coarse IK 조준 -> TRACKING
    TRACKING    - YOLO + IBVS, deadband 진입 -> CONFIRMING
    CONFIRMING  - 0.5초 유지 -> FIRING / 이탈 -> TRACKING
    FIRING      - gripper 격발 -> COOLDOWN
    COOLDOWN    - 5초 대기 + home -> IDLE

내부 클래스:
    YoloDetector  - 카메라 + YOLO 검출
    OmxController - OMX 모터 제어 + IK + IBVS + 격발
    StateMachine  - 상태 전이 로직
    OmxYoloNode   - ROS 통합

Publish:
    /omx/status              std_msgs/String        1 Hz
    /omx/state               std_msgs/String        상태 변경 시
    /omx/target_detected     std_msgs/Bool          매 프레임
    /omx/error_norm          geometry_msgs/Point    검출 시
    /omx/joint_state         sensor_msgs/JointState 매 프레임
    /omx/fire                std_msgs/Empty         격발 1회
    /omx/target_processed    geometry_msgs/Point    처리 완료
    /omx/aim_progress        std_msgs/Float32       CONFIRMING 진행도

Subscribe:
    /omx/control_mode        std_msgs/String        모드 (idle)
    /omx/target_coord        geometry_msgs/Point    표적 좌표
    /omx/arm_enable          std_msgs/Bool          자율 검출 허용
    /omx/abort               std_msgs/Empty         비상 정지

키:
    p   - pause
    a   - arm/disarm 토글
    h   - home (수동)
    ESC - 종료
"""

from __future__ import annotations

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

import math
import time
from enum import Enum
from typing import Optional

import cv2
import rclpy
from rclpy.node import Node

from std_msgs.msg import String, Bool, Float32, Empty
from geometry_msgs.msg import Point
from sensor_msgs.msg import JointState

from ultralytics import YOLO

from omx.hardware import build_bus, get_dxl_symbols, ARM_MOTORS, MOTOR_ORDER
from omx.config import load_config, Config


TICKS_PER_REV = 4096
RAD2TICK = TICKS_PER_REV / (2.0 * math.pi)


class State(Enum):
    IDLE = "idle"
    AIMING = "aiming"
    TRACKING = "tracking"
    CONFIRMING = "confirming"
    FIRING = "firing"
    COOLDOWN = "cooldown"


# ===========================================================
# YoloDetector
# ===========================================================

class YoloDetector:
    """카메라 frame 캡처 + YOLO 검출."""

    def __init__(self, cfg: Config, logger=None):
        self.cfg = cfg
        self.logger = logger

        cam_idx = cfg.ibvs.camera_index
        self.cap = cv2.VideoCapture(cam_idx)
        if not self.cap.isOpened():
            raise RuntimeError(f"카메라 {cam_idx} 열기 실패")
        self._log(f"카메라 {cam_idx} 열림")

        self.model = YOLO(cfg.yolo.model_path)
        self.target_class = cfg.yolo.target_class
        self.class_name = self.model.names.get(
            self.target_class, f"cls_{self.target_class}")
        self._log(f"YOLO 로드: {cfg.yolo.model_path}, "
                  f"클래스 {self.target_class} ({self.class_name})")

    def _log(self, msg):
        if self.logger:
            self.logger.info(msg)
        else:
            print(msg)

    def read_frame(self):
        ok, frame = self.cap.read()
        return frame if ok else None

    def detect(self, frame):
        """반환: (detected, error_norm, bbox, conf)."""
        h, w = frame.shape[:2]
        cx, cy = w / 2.0, h / 2.0

        results = self.model.predict(
            frame,
            imgsz=self.cfg.yolo.imgsz,
            conf=self.cfg.yolo.conf_threshold,
            classes=[self.target_class],
            verbose=False,
        )
        boxes = results[0].boxes

        if boxes is None or len(boxes) == 0:
            return False, None, None, None

        confs = boxes.conf.cpu().numpy()
        idx = confs.argmax()
        xyxy = boxes.xyxy[idx].cpu().numpy()
        x1, y1, x2, y2 = [int(v) for v in xyxy]
        conf = float(confs[idx])

        obj_x = (x1 + x2) / 2.0
        obj_y = (y1 + y2) / 2.0
        ex = (obj_x - cx) / cx
        ey = (obj_y - cy) / cy

        return True, (ex, ey), (x1, y1, x2, y2), conf

    def release(self):
        if self.cap:
            self.cap.release()


# ===========================================================
# OmxController
# ===========================================================

class OmxController:
    """OMX 모터 제어 + IK + IBVS + 격발."""

    def __init__(self, cfg: Config, dry_run: bool = False, logger=None):
        self.cfg = cfg
        self.dry_run = dry_run
        self.bus = None if dry_run else build_bus(cfg.motor.port)
        self.yaw = 0.0
        self.pitch = 0.0
        self.logger = logger

    def _log(self, msg, level="info"):
        if self.logger:
            getattr(self.logger, level)(msg)
        else:
            print(msg)

    def connect(self):
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
            # self.bus.write("Operating_Mode", "gripper",
            #                OperatingMode.CURRENT_POSITION.value,
            #                normalize=False)
            for m in MOTOR_ORDER:
                self.bus.write("Profile_Velocity", m,
                               self.cfg.motor.profile_velocity, normalize=False)
                self.bus.write("Profile_Acceleration", m,
                               self.cfg.motor.profile_acceleration, normalize=False)
        self.bus.enable_torque(num_retry=3)
        self._log("OMX 연결 완료")

    def disconnect(self):
        if self.dry_run or self.bus is None or not self.bus.is_connected:
            return
        self.bus.disconnect(disable_torque=True)
        self._log("OMX 연결 해제")

    def go_home(self):
        if self.dry_run:
            self.yaw = 0.0
            self.pitch = 0.0
            self._log("[dry-run] Home 이동 시뮬레이션")
            return
        home = self.cfg.calibration.home
        for m in MOTOR_ORDER:
            self.bus.write("Goal_Position", m, home[m], normalize=False)
        time.sleep(2.0)
        self.yaw = 0.0
        self.pitch = 0.0
        self._log("Home 도달")

    def aim_at_coord(self, x: float, y: float, z: float):
        """Point-at IK 조준."""
        if x == 0.0 and y == 0.0 and z == 0.0:
            self._log("원점 좌표는 가리킬 수 없음", "warn")
            return

        new_yaw = math.atan2(y, x)
        new_pitch = math.atan2(z, math.hypot(x, y))

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
            for m in ("elbow_flex", "wrist_flex", "wrist_roll"):
                self.bus.write("Goal_Position", m, home[m], normalize=False)
            self.bus.write("Goal_Position", "shoulder_pan",
                           yaw_tick, normalize=False)
            self.bus.write("Goal_Position", "shoulder_lift",
                           pitch_tick, normalize=False)

        self._log(f"Coarse aim: yaw={math.degrees(new_yaw):.1f}, "
                  f"pitch={math.degrees(new_pitch):.1f}")

    def step_ibvs(self, error_x: float, error_y: float) -> bool:
        max_step = self.cfg.safety.max_step_rad
        deadband = self.cfg.ibvs.deadband

        ex = 0.0 if abs(error_x) < deadband else error_x
        ey = 0.0 if abs(error_y) < deadband else error_y

        if ex == 0.0 and ey == 0.0:
            return False

        delta_yaw = self.cfg.ibvs.sign_vs_x * self.cfg.ibvs.kp_yaw * ex
        delta_pitch = self.cfg.ibvs.sign_vs_y * self.cfg.ibvs.kp_pitch * ey

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
        return True

    def fire(self):
        """gripper 닫았다 펴서 격발 시뮬레이션."""
        if self.dry_run:
            self._log("[dry-run] 격발 시뮬레이션")
            # time.sleep(self.cfg.fire.gripper_close_duration)
            # time.sleep(self.cfg.fire.gripper_open_duration)
            return

        # self.bus.write("Goal_Position", "gripper",
        #                self.cfg.fire.gripper_close_pos, normalize=False)
        # time.sleep(self.cfg.fire.gripper_close_duration)

        # self.bus.write("Goal_Position", "gripper",
        #                self.cfg.fire.gripper_open_pos, normalize=False)
        # time.sleep(self.cfg.fire.gripper_open_duration)

        self._log("격발 완료")

    def read_joint_positions_rad(self):
        if self.dry_run or self.bus is None or not self.bus.is_connected:
            return {
                "shoulder_pan": self.yaw,
                "shoulder_lift": self.pitch,
                "elbow_flex": 0.0,
                "wrist_flex": 0.0,
                "wrist_roll": 0.0,
                # "gripper": 0.0,
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
# StateMachine
# ===========================================================

class StateMachine:
    """OMX 상태 전이 관리.
    
    미래 큐 도입 시 target_coord 관리 로직만 변경하면 됨.
    """

    def __init__(self, cfg: Config, logger=None):
        self.cfg = cfg
        self.logger = logger
        self.state = State.IDLE

        # 현재 처리 중인 표적 (미래 큐로 발전)
        self.target_coord: Optional[tuple] = None
        
        self.confirm_start_t: float = 0.0
        self.confirm_progress: float = 0.0
        self.cooldown_until: float = 0.0
        self.cooldown_home_sent: bool = False

        self.armed = cfg.autotrack.default_armed if cfg.autotrack else False
        self.last_processed: Optional[tuple] = None

    def _log(self, msg):
        if self.logger:
            self.logger.info(msg)
        else:
            print(msg)

    def transition(self, new_state: State):
        if self.state != new_state:
            self._log(f"State: {self.state.value} -> {new_state.value}")
            self.state = new_state

    def on_target_coord(self, coord) -> bool:
        """외부 좌표 수신. accepted 반환."""
        # Option A: CONFIRMING 이후는 무시
        if self.state in (State.CONFIRMING, State.FIRING, State.COOLDOWN):
            self._log(f"좌표 무시 (state={self.state.value}, 격발 우선)")
            return False

        if self._is_duplicate(coord):
            self._log(f"좌표 무시 (이미 처리한 표적)")
            return False

        self.target_coord = coord
        self._log(f"표적 좌표 받음: {coord}")

        if self.state in (State.IDLE, State.TRACKING):
            self.transition(State.AIMING)
        return True

    def _is_duplicate(self, coord) -> bool:
        if not self.last_processed or not self.cfg.autotrack:
            return False
        dx = coord[0] - self.last_processed[0]
        dy = coord[1] - self.last_processed[1]
        dz = coord[2] - self.last_processed[2]
        dist = math.sqrt(dx*dx + dy*dy + dz*dz)
        return dist < self.cfg.autotrack.duplicate_threshold_m

    def on_abort(self):
        self._log("ABORT - IDLE 로 전이")
        self.transition(State.IDLE)
        self.target_coord = None
        self.confirm_progress = 0.0
        self.cooldown_home_sent = False

    def on_arm_enable(self, armed: bool):
        self.armed = armed
        self._log(f"Armed: {armed}")

    def update(self, detected: bool, error_norm, now: float) -> dict:
        """매 frame 호출. 다음 action 반환."""
        action = {
            'action': 'wait',
            'state': self.state,
            'target_coord': None,
            'error': None,
            'confirm_progress': 0.0,
        }

        if self.state == State.IDLE:
            if self.armed and detected:
                self._log("Autonomous detection -> TRACKING")
                self.transition(State.TRACKING)

        elif self.state == State.AIMING:
            if self.target_coord:
                action['action'] = 'aim'
                action['target_coord'] = self.target_coord
                self.last_processed = self.target_coord
                self.transition(State.TRACKING)

        elif self.state == State.TRACKING:
            if detected:
                ex, ey = error_norm
                deadband = self.cfg.ibvs.deadband
                if abs(ex) < deadband and abs(ey) < deadband:
                    self._log("표적 deadband 진입 -> CONFIRMING")
                    self.transition(State.CONFIRMING)
                    self.confirm_start_t = now
                    self.confirm_progress = 0.0
                else:
                    action['action'] = 'track'
                    action['error'] = error_norm

        elif self.state == State.CONFIRMING:
            if not detected:
                self._log("CONFIRMING 중 표적 사라짐 -> TRACKING")
                self.transition(State.TRACKING)
                self.confirm_progress = 0.0
            else:
                ex, ey = error_norm
                confirm_db = (self.cfg.ibvs.deadband
                              * self.cfg.fire.confirm_deadband_scale)
                if abs(ex) > confirm_db or abs(ey) > confirm_db:
                    self._log("CONFIRMING 중 이탈 -> TRACKING")
                    self.transition(State.TRACKING)
                    self.confirm_progress = 0.0
                else:
                    elapsed = now - self.confirm_start_t
                    self.confirm_progress = min(
                        1.0, elapsed / self.cfg.fire.hold_time_sec)
                    if elapsed >= self.cfg.fire.hold_time_sec:
                        self._log(f"조준 {self.cfg.fire.hold_time_sec}s 유지 "
                                  f"-> FIRING")
                        self.transition(State.FIRING)
                        self.confirm_progress = 1.0

        elif self.state == State.FIRING:
            action['action'] = 'fire'
            self.transition(State.COOLDOWN)
            self.cooldown_until = now + self.cfg.fire.cooldown_sec
            self.cooldown_home_sent = False

        elif self.state == State.COOLDOWN:
            if now >= self.cooldown_until:
                self._log("Cooldown 끝 -> IDLE")
                self.transition(State.IDLE)
                self.target_coord = None
                self.confirm_progress = 0.0
                self.cooldown_home_sent = False
            else:
                if not self.cooldown_home_sent:
                    action['action'] = 'home'
                    self.cooldown_home_sent = True

        action['state'] = self.state
        action['confirm_progress'] = self.confirm_progress
        return action


# ===========================================================
# OmxYoloNode (ROS 통합)
# ===========================================================

class OmxYoloNode(Node):
    def __init__(self, dry_run: bool = False):
        super().__init__('omx_yolo_node')

        self.cfg = load_config()
        self.dry_run = dry_run
        self.get_logger().info(f"Config loaded. port={self.cfg.motor.port}")

        if self.cfg.fire is None:
            raise RuntimeError("config.yaml 에 fire 섹션 필요")
        if self.cfg.yolo is None:
            raise RuntimeError("config.yaml 에 yolo 섹션 필요")
        if self.cfg.autotrack is None:
            raise RuntimeError("config.yaml 에 autotrack 섹션 필요")

        self.detector = YoloDetector(self.cfg, logger=self.get_logger())
        self.ctrl = OmxController(self.cfg, dry_run=dry_run,
                                    logger=self.get_logger())
        self.sm = StateMachine(self.cfg, logger=self.get_logger())

        self.ctrl.connect()
        self.ctrl.go_home()

        self.paused = False
        self.control_period = 1.0 / self.cfg.ibvs.control_hz

        self.fps_t = time.time()
        self.fps_n = 0
        self.fps_disp = 0.0

        # Publishers
        self.pub_status = self.create_publisher(String, '/omx/status', 10)
        self.pub_state = self.create_publisher(String, '/omx/state', 10)
        self.pub_detected = self.create_publisher(Bool, '/omx/target_detected', 10)
        self.pub_error = self.create_publisher(Point, '/omx/error_norm', 10)
        self.pub_joint = self.create_publisher(JointState, '/omx/joint_state', 10)
        self.pub_fire = self.create_publisher(Empty, '/omx/fire', 10)
        self.pub_processed = self.create_publisher(Point, '/omx/target_processed', 10)
        self.pub_progress = self.create_publisher(Float32, '/omx/aim_progress', 10)

        # Subscribers
        self.create_subscription(String, '/omx/control_mode',
                                 self.on_control_mode, 10)
        self.create_subscription(Point, '/omx/target_coord',
                                 self.on_target_coord, 10)
        self.create_subscription(Bool, '/omx/arm_enable',
                                 self.on_arm_enable, 10)
        self.create_subscription(Empty, '/omx/abort',
                                 self.on_abort, 10)

        # Timer
        self.timer = self.create_timer(self.control_period, self.loop)
        self.status_timer = self.create_timer(1.0, self.publish_status)

        self._last_state = self.sm.state

        self.get_logger().info(
            f"Timer: 메인 {self.cfg.ibvs.control_hz} Hz, 상태 1 Hz")
        self.get_logger().info(f"Initial armed: {self.sm.armed}")
        self.get_logger().info("=== Node ready ===")

    # ----- Subscribers -----

    def on_control_mode(self, msg):
        if msg.data == "idle":
            self.sm.on_abort()
            self.ctrl.go_home()

    def on_target_coord(self, msg):
        coord = (msg.x, msg.y, msg.z)
        self.sm.on_target_coord(coord)

    def on_arm_enable(self, msg):
        self.sm.on_arm_enable(msg.data)

    def on_abort(self, msg):
        self.sm.on_abort()
        self.ctrl.go_home()

    # ----- Publishers -----

    def publish_status(self):
        msg = String()
        prefix = ""
        if self.dry_run:
            prefix = "dry_run_"
        if self.paused:
            prefix = "paused_"
        msg.data = f"{prefix}{self.sm.state.value}"
        self.pub_status.publish(msg)

    def publish_state_change(self):
        if self.sm.state != self._last_state:
            msg = String()
            msg.data = self.sm.state.value
            self.pub_state.publish(msg)
            self._last_state = self.sm.state

    def publish_detected(self, detected):
        msg = Bool()
        msg.data = detected
        self.pub_detected.publish(msg)

    def publish_error(self, ex, ey):
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

    def publish_progress(self, p):
        msg = Float32()
        msg.data = float(p)
        self.pub_progress.publish(msg)

    def publish_fire(self):
        self.pub_fire.publish(Empty())

    def publish_processed(self, coord):
        if coord is None:
            return
        msg = Point()
        msg.x, msg.y, msg.z = coord
        self.pub_processed.publish(msg)

    # ----- Main loop -----

    def loop(self):
        frame = self.detector.read_frame()
        if frame is None:
            self.get_logger().warn("프레임 읽기 실패")
            return

        detected, error_norm, bbox, conf = self.detector.detect(frame)

        now = time.time()
        action = self.sm.update(detected, error_norm, now)

        # action 수행
        if not self.paused:
            if action['action'] == 'aim':
                self.ctrl.aim_at_coord(*action['target_coord'])
            elif action['action'] == 'track':
                self.ctrl.step_ibvs(*action['error'])
            elif action['action'] == 'fire':
                processed = self.sm.target_coord
                self.publish_fire()
                self.ctrl.fire()
                self.publish_processed(processed)
            elif action['action'] == 'home':
                self.ctrl.go_home()

        # Publish
        self.publish_detected(detected)
        if error_norm is not None:
            self.publish_error(error_norm[0], error_norm[1])
        self.publish_joint_state()
        self.publish_progress(action.get('confirm_progress', 0.0))
        self.publish_state_change()

        # 시각화
        self.visualize(frame, detected, error_norm, bbox, conf, action)

        # 키
        key = cv2.waitKey(1) & 0xFF
        self._handle_key(key)

        # FPS
        self.fps_n += 1
        if now - self.fps_t >= 1.0:
            self.fps_disp = self.fps_n / (now - self.fps_t)
            self.fps_t = now
            self.fps_n = 0

    def visualize(self, frame, detected, error_norm, bbox, conf, action):
        h, w = frame.shape[:2]
        cx, cy = w / 2.0, h / 2.0
        deadband = self.cfg.ibvs.deadband

        # 중심 + deadband
        cv2.drawMarker(frame, (int(cx), int(cy)),
                       (0, 255, 255), cv2.MARKER_CROSS, 20, 1)
        dz_x = int(deadband * cx)
        dz_y = int(deadband * cy)
        cv2.rectangle(frame,
                      (int(cx) - dz_x, int(cy) - dz_y),
                      (int(cx) + dz_x, int(cy) + dz_y),
                      (80, 80, 80), 1)

        # 검출 박스 (상태별 색)
        if detected and bbox:
            x1, y1, x2, y2 = bbox
            state_color = {
                State.IDLE: (180, 180, 180),
                State.AIMING: (255, 200, 0),
                State.TRACKING: (0, 255, 0),
                State.CONFIRMING: (0, 165, 255),
                State.FIRING: (0, 0, 255),
                State.COOLDOWN: (200, 100, 200),
            }
            color = state_color.get(self.sm.state, (255, 255, 255))
            cv2.rectangle(frame, (x1, y1), (x2, y2), color, 2)
            obj_x = (x1 + x2) / 2.0
            obj_y = (y1 + y2) / 2.0
            cv2.circle(frame, (int(obj_x), int(obj_y)), 4, color, -1)
            cv2.line(frame, (int(cx), int(cy)),
                     (int(obj_x), int(obj_y)), color, 1)
            cv2.putText(frame, f"{self.detector.class_name} {conf:.2f}",
                        (x1, max(y1 - 8, 16)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)

        # 상태 표시
        state_txt = f"[{self.sm.state.value.upper()}]"
        if self.paused:
            state_txt = f"[PAUSED|{self.sm.state.value}]"
        if self.dry_run:
            state_txt = f"[DRY|{self.sm.state.value}]"

        armed_txt = "ARMED" if self.sm.armed else "DISARMED"

        cv2.putText(frame,
                    f"{state_txt} {armed_txt} "
                    f"yaw={math.degrees(self.ctrl.yaw):+.1f} "
                    f"pitch={math.degrees(self.ctrl.pitch):+.1f} "
                    f"fps={self.fps_disp:.1f}",
                    (10, 25), cv2.FONT_HERSHEY_SIMPLEX,
                    0.55, (255, 255, 255), 2)

        # CONFIRMING 진행도 바
        progress = action.get('confirm_progress', 0.0)
        if progress > 0 or self.sm.state == State.CONFIRMING:
            bar_x, bar_y, bar_w, bar_h = 10, h - 60, 200, 15
            cv2.rectangle(frame, (bar_x, bar_y),
                         (bar_x + bar_w, bar_y + bar_h),
                         (100, 100, 100), 1)
            cv2.rectangle(frame, (bar_x, bar_y),
                         (bar_x + int(bar_w * progress), bar_y + bar_h),
                         (0, 165, 255), -1)
            cv2.putText(frame, f"AIM {progress*100:.0f}%",
                        (bar_x + bar_w + 10, bar_y + 12),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 255, 255), 1)

        if error_norm is not None:
            cv2.putText(frame,
                        f"err=({error_norm[0]:+.2f}, {error_norm[1]:+.2f})",
                        (10, h - 40), cv2.FONT_HERSHEY_SIMPLEX,
                        0.5, (255, 255, 255), 1)

        cv2.putText(frame, "p:pause a:arm h:home ESC:quit",
                    (10, h - 15), cv2.FONT_HERSHEY_SIMPLEX,
                    0.45, (180, 180, 180), 1)

        cv2.imshow("OMX YOLO node", frame)

    def _handle_key(self, key):
        if key == 27:
            self.get_logger().info("ESC. 종료.")
            rclpy.shutdown()
        elif key == ord('p'):
            self.paused = not self.paused
            self.get_logger().info("일시정지" if self.paused else "재개")
        elif key == ord('a'):
            self.sm.armed = not self.sm.armed
            self.get_logger().info(f"Armed: {self.sm.armed}")
        elif key == ord('h'):
            self.get_logger().info("Home 복귀 (수동)")
            self.sm.on_abort()
            self.ctrl.go_home()

    def destroy_node(self):
        if hasattr(self, 'detector'):
            self.detector.release()
        cv2.destroyAllWindows()
        if hasattr(self, 'ctrl'):
            self.ctrl.disconnect()
        super().destroy_node()


# ===========================================================
# Entry
# ===========================================================

def main(args=None):
    import argparse
    parser = argparse.ArgumentParser(
        description="OMX YOLO ROS 2 node with state machine + fire")
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