#!/usr/bin/env python3
"""OMX YOLO tracker - 단계 G: 거리 정렬 + RViz 시각화.

진화 단계:
    A : 우선순위 큐 (heapq)
    D : map 좌표 기반 큐 (TF 변환)
    F : LOS 검사 + TargetType 카테고리
    G : 거리 정렬 + RViz 마커 시각화  ← 이번 단계

좌표 종류 (TargetType):
    TARGET   (priority=0)  - 확정 표적 (이동 + 조준/격발)
    BOUNDARY (priority=5)  - 경계 (이동 중 사주 경계)
    PATROL   (priority=10) - 탐색 목표 (이동 + 정찰)

큐 정렬 (단계 G):
    sort_key = (priority, distance, count)
    - 같은 priority 안: 가까운 좌표 먼저
    - 다른 priority 간: priority 가 결정
    - 정렬 시점: pop 직전 (옵션 B)

LOS 정책 (단계 F):
                CLEAR    BLOCKED   UNKNOWN
    TARGET      처리     시도      처리
    BOUNDARY    처리     폐기      폐기
    PATROL      처리     시도      처리

상태 머신:
    IDLE/AIMING/SCANNING/TRACKING/CONFIRMING/FIRING/COOLDOWN

Subscribe:
    /omx/target_in_map       PointStamped  TARGET
    /omx/boundary_in_map     PointStamped  BOUNDARY
    /omx/patrol_in_map       PointStamped  PATROL
    /omx/control_mode        String        idle
    /omx/arm_enable          Bool
    /omx/abort               Empty
    /global_costmap/costmap  OccupancyGrid

Publish:
    /omx/status, /omx/state, /omx/target_detected, /omx/error_norm
    /omx/joint_state, /omx/fire, /omx/aim_progress
    /omx/target_processed   PointStamped
    /omx/target_lost        PointStamped
    /omx/target_blocked     PointStamped
    /omx/queue_size, /omx/patrol_complete
    /omx/queue_markers      MarkerArray   ← 단계 G
"""

from __future__ import annotations

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

import heapq
import itertools
import math
import time
from dataclasses import dataclass, field
from enum import Enum, IntEnum
from typing import Optional

import cv2
import rclpy
from rclpy.node import Node
from rclpy.duration import Duration

from std_msgs.msg import String, Bool, Float32, Empty, Int32
from geometry_msgs.msg import Point, PointStamped
from sensor_msgs.msg import JointState
from nav_msgs.msg import OccupancyGrid
from visualization_msgs.msg import Marker, MarkerArray

from tf2_ros import Buffer, TransformListener, TransformException

try:
    from tf2_geometry_msgs import do_transform_point
except ImportError:
    print()
    print("ERROR: tf2_geometry_msgs 패키지가 없습니다.")
    print("  sudo apt install ros-jazzy-tf2-geometry-msgs")
    sys.exit(1)

from ultralytics import YOLO

from omx.hardware import build_bus, get_dxl_symbols, ARM_MOTORS, MOTOR_ORDER
from omx.config import load_config, Config


TICKS_PER_REV = 4096
RAD2TICK = TICKS_PER_REV / (2.0 * math.pi)


class State(Enum):
    IDLE = "idle"
    AIMING = "aiming"
    SCANNING = "scanning"
    TRACKING = "tracking"
    CONFIRMING = "confirming"
    FIRING = "firing"
    COOLDOWN = "cooldown"


class TargetType(IntEnum):
    TARGET = 0
    BOUNDARY = 5
    PATROL = 10


class LOSResult(Enum):
    CLEAR = "clear"
    BLOCKED = "blocked"
    UNKNOWN = "unknown"


_entry_counter = itertools.count()


# ===========================================================
# TargetEntry (단계 G: sort_key 기반)
# ===========================================================

@dataclass(order=True)
class TargetEntry:
    """heapq 정렬은 sort_key 만 사용."""
    sort_key: tuple = field(init=False, default=(0, 0.0, 0))
    
    priority: int = field(compare=False, default=10)
    count: int = field(compare=False,
                       default_factory=lambda: next(_entry_counter))
    coord_map: tuple = field(compare=False, default=(0.0, 0.0, 0.0))
    target_type: TargetType = field(compare=False,
                                     default=TargetType.PATROL)
    arrival_time: float = field(compare=False, default=0.0)
    distance: float = field(compare=False, default=0.0)
    
    def __post_init__(self):
        self._update_sort_key()
    
    def update_distance(self, waffle_xy):
        """와플 위치 기준 유클리드 거리 갱신."""
        if waffle_xy is None:
            self.distance = 0.0
        else:
            dx = self.coord_map[0] - waffle_xy[0]
            dy = self.coord_map[1] - waffle_xy[1]
            self.distance = math.sqrt(dx*dx + dy*dy)
        self._update_sort_key()
    
    def _update_sort_key(self):
        self.sort_key = (self.priority, self.distance, self.count)
    
    @property
    def type_name(self):
        return self.target_type.name


def bresenham_line(x0: int, y0: int, x1: int, y1: int):
    cells = []
    dx = abs(x1 - x0)
    dy = abs(y1 - y0)
    sx = 1 if x0 < x1 else -1
    sy = 1 if y0 < y1 else -1
    err = dx - dy
    
    while True:
        cells.append((x0, y0))
        if x0 == x1 and y0 == y1:
            break
        e2 = 2 * err
        if e2 > -dy:
            err -= dy
            x0 += sx
        if e2 < dx:
            err += dx
            y0 += sy
    return cells


# ===========================================================
# YoloDetector
# ===========================================================

class YoloDetector:
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
        h, w = frame.shape[:2]
        cx, cy = w / 2.0, h / 2.0

        results = self.model.predict(
            frame, imgsz=self.cfg.yolo.imgsz,
            conf=self.cfg.yolo.conf_threshold,
            classes=[self.target_class], verbose=False)
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
            self.bus.write("Operating_Mode", "gripper",
                           OperatingMode.CURRENT_POSITION.value,
                           normalize=False)
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

    def aim_at_coord(self, x, y, z):
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

    def step_ibvs(self, error_x, error_y):
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
        if self.dry_run:
            self._log("[dry-run] 격발 시뮬레이션")
            time.sleep(self.cfg.fire.gripper_close_duration)
            time.sleep(self.cfg.fire.gripper_open_duration)
            return

        self.bus.write("Goal_Position", "gripper",
                       self.cfg.fire.gripper_close_pos, normalize=False)
        time.sleep(self.cfg.fire.gripper_close_duration)

        self.bus.write("Goal_Position", "gripper",
                       self.cfg.fire.gripper_open_pos, normalize=False)
        time.sleep(self.cfg.fire.gripper_open_duration)

        self._log("격발 완료")

    def read_joint_positions_rad(self):
        if self.dry_run or self.bus is None or not self.bus.is_connected:
            return {
                "shoulder_pan": self.yaw,
                "shoulder_lift": self.pitch,
                "elbow_flex": 0.0,
                "wrist_flex": 0.0,
                "wrist_roll": 0.0,
                "gripper": 0.0,
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
# StateMachine (큐 + LOS + 거리 정렬)
# ===========================================================

class StateMachine:
    def __init__(self, cfg: Config, logger=None):
        self.cfg = cfg
        self.logger = logger
        self.state = State.IDLE

        self.queue: list[TargetEntry] = []
        self.current_target: Optional[TargetEntry] = None

        self.scan_start_t: float = 0.0
        self.confirm_start_t: float = 0.0
        self.confirm_progress: float = 0.0
        self.cooldown_until: float = 0.0
        self.cooldown_home_sent: bool = False
        self.lost_start_t: float = 0.0

        self.armed = cfg.autotrack.default_armed if cfg.autotrack else False
        self.last_processed: Optional[tuple] = None
        self.patrol_complete_sent = True

        # 콜백 (OmxYoloNode 가 주입)
        self.los_check_fn = None
        self.waffle_pos_fn = None    # 단계 G

    def _log(self, msg):
        if self.logger:
            self.logger.info(msg)
        else:
            print(msg)

    def transition(self, new_state: State):
        if self.state != new_state:
            self._log(f"State: {self.state.value} -> {new_state.value}")
            self.state = new_state

    def add_target(self, coord, target_type: TargetType) -> bool:
        if self.state in (State.CONFIRMING, State.FIRING):
            self._log(f"좌표 무시 (state={self.state.value}, 격발 우선): "
                      f"type={target_type.name}")
            return False
        
        if self._is_duplicate(coord):
            self._log(f"좌표 중복 무시: {coord}")
            return False
        
        max_size = self.cfg.patrol.max_queue_size if self.cfg.patrol else 20
        if len(self.queue) >= max_size:
            removed = self._remove_oldest_low_priority()
            if not removed:
                self._log(f"큐 가득 ({max_size}), 추가 거부")
                return False
        
        entry = TargetEntry(
            priority=int(target_type),
            coord_map=coord,
            target_type=target_type,
            arrival_time=time.time(),
        )
        # 초기 거리 계산 (와플 위치 알면)
        if self.waffle_pos_fn:
            waffle_xy = self.waffle_pos_fn()
            entry.update_distance(waffle_xy)
        
        heapq.heappush(self.queue, entry)
        self.patrol_complete_sent = False
        
        self._log(f"큐 추가: type={target_type.name} "
                  f"coord={coord} dist={entry.distance:.2f}m, "
                  f"큐크기={len(self.queue)}")
        return True

    def _is_duplicate(self, coord) -> bool:
        if not self.cfg.patrol:
            return False
        threshold = self.cfg.patrol.duplicate_threshold_m
        
        if self.last_processed:
            if self._distance(coord, self.last_processed) < threshold:
                return True
        
        for entry in self.queue:
            if self._distance(coord, entry.coord_map) < threshold:
                return True
        
        if self.current_target:
            if self._distance(coord, self.current_target.coord_map) < threshold:
                return True
        
        return False

    def _distance(self, a, b):
        return math.sqrt(sum((ai - bi)**2 for ai, bi in zip(a, b)))

    def _remove_oldest_low_priority(self) -> bool:
        if not self.queue:
            return False
        max_idx = 0
        for i, entry in enumerate(self.queue):
            if (entry.priority > self.queue[max_idx].priority or
                (entry.priority == self.queue[max_idx].priority
                 and entry.count < self.queue[max_idx].count)):
                max_idx = i
        
        removed = self.queue.pop(max_idx)
        heapq.heapify(self.queue)
        self._log(f"큐 가득, 오래된 {removed.type_name} 제거")
        return True

    def resort_queue(self, waffle_xy):
        """큐 전체 거리 갱신 후 heap 재정렬 (단계 G, 옵션 B)."""
        if not self.queue:
            return
        
        for entry in self.queue:
            entry.update_distance(waffle_xy)
        
        heapq.heapify(self.queue)

    def pop_next_with_los(self, waffle_xy=None):
        """LOS 검사 통과한 entry pop.
        
        waffle_xy 주어지면 pop 전에 거리 기반 재정렬 (단계 G).
        """
        blocked_entries = []
        
        if waffle_xy is not None:
            self.resort_queue(waffle_xy)
        
        while self.queue:
            entry = heapq.heappop(self.queue)
            
            if (not self.cfg.patrol or
                not self.cfg.patrol.los_check_enabled or
                self.los_check_fn is None):
                return entry, blocked_entries
            
            result = self.los_check_fn(entry.coord_map)
            
            if result == LOSResult.CLEAR:
                self._log(f"LOS CLEAR: {entry.type_name} "
                          f"{entry.coord_map} dist={entry.distance:.2f}m")
                return entry, blocked_entries
            
            elif result == LOSResult.UNKNOWN:
                if entry.target_type == TargetType.BOUNDARY:
                    self._log(
                        f"LOS UNKNOWN, BOUNDARY 폐기: {entry.coord_map}")
                    blocked_entries.append(entry)
                    continue
                else:
                    self._log(
                        f"LOS UNKNOWN, {entry.type_name} 시도: "
                        f"{entry.coord_map}")
                    return entry, blocked_entries
            
            elif result == LOSResult.BLOCKED:
                if entry.target_type == TargetType.BOUNDARY:
                    self._log(
                        f"LOS BLOCKED, BOUNDARY 폐기: {entry.coord_map}")
                    blocked_entries.append(entry)
                    continue
                else:
                    self._log(
                        f"LOS BLOCKED, {entry.type_name} 시도: "
                        f"{entry.coord_map}")
                    return entry, blocked_entries
        
        return None, blocked_entries

    def clear_queue(self):
        self.queue.clear()
        self.current_target = None
        self._log("큐 비움")

    def queue_size(self) -> int:
        return len(self.queue)

    def on_target(self, coord) -> bool:
        return self.add_target(coord, TargetType.TARGET)

    def on_boundary(self, coord) -> bool:
        return self.add_target(coord, TargetType.BOUNDARY)

    def on_patrol(self, coord) -> bool:
        return self.add_target(coord, TargetType.PATROL)

    def on_abort(self):
        self._log("ABORT - IDLE + 큐 비움")
        self.transition(State.IDLE)
        self.clear_queue()
        self.confirm_progress = 0.0
        self.cooldown_home_sent = False
        self.patrol_complete_sent = True
        self.lost_start_t = 0.0

    def on_arm_enable(self, armed: bool):
        self.armed = armed
        self._log(f"Armed: {armed}")

    def update(self, detected: bool, error_norm, now: float) -> dict:
        action = {
            'action': 'wait',
            'state': self.state,
            'coord_map': None,
            'error': None,
            'confirm_progress': 0.0,
            'patrol_complete': False,
            'lost_coord_map': None,
            'blocked_entries': [],
        }

        if self.state == State.IDLE:
            if self.queue:
                # 와플 위치 가져와서 재정렬 후 pop
                waffle_xy = None
                if self.waffle_pos_fn:
                    waffle_xy = self.waffle_pos_fn()
                
                entry, blocked = self.pop_next_with_los(waffle_xy)
                action['blocked_entries'] = blocked
                
                if entry:
                    self.current_target = entry
                    self._log(f"큐에서 pop: type={entry.type_name} "
                              f"coord={entry.coord_map} "
                              f"dist={entry.distance:.2f}m")
                    self.transition(State.AIMING)
            
            elif self.armed and detected:
                self._log("Autonomous detection -> TRACKING")
                self.transition(State.TRACKING)
            else:
                if not self.patrol_complete_sent:
                    action['patrol_complete'] = True
                    self.patrol_complete_sent = True
                    self._log("정찰 완료 - 큐 비었음")

        elif self.state == State.AIMING:
            if self.current_target:
                action['action'] = 'aim'
                action['coord_map'] = self.current_target.coord_map
                self.last_processed = self.current_target.coord_map
                self.transition(State.SCANNING)
                self.scan_start_t = now

        elif self.state == State.SCANNING:
            if detected:
                self._log("SCANNING 중 표적 발견 -> TRACKING")
                self.lost_start_t = 0.0
                self.transition(State.TRACKING)
            else:
                scan_timeout = (self.cfg.patrol.scan_timeout_sec
                                if self.cfg.patrol else 2.0)
                if now - self.scan_start_t >= scan_timeout:
                    self._log(
                        f"SCANNING {scan_timeout}s 끝, 표적 없음 -> IDLE")
                    self.current_target = None
                    self.transition(State.IDLE)

        elif self.state == State.TRACKING:
            if detected:
                self.lost_start_t = 0.0
                
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
            else:
                if self.lost_start_t == 0.0:
                    self.lost_start_t = now
                    self._log("TRACKING 중 표적 사라짐 (타임아웃 대기)")
                
                elapsed = now - self.lost_start_t
                timeout = self.cfg.fire.lost_timeout_sec
                
                if elapsed >= timeout:
                    self._log(f"TRACKING 중 표적 {timeout:.1f}s 잃음 -> IDLE")
                    
                    if self.current_target:
                        self.last_processed = self.current_target.coord_map
                        action['lost_coord_map'] = self.current_target.coord_map
                    
                    action['action'] = 'target_lost'
                    self.current_target = None
                    self.lost_start_t = 0.0
                    self.transition(State.IDLE)

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
                self.current_target = None
                self.confirm_progress = 0.0
                self.cooldown_home_sent = False
                self.transition(State.IDLE)
            else:
                if not self.cooldown_home_sent:
                    action['action'] = 'home'
                    self.cooldown_home_sent = True

        action['state'] = self.state
        action['confirm_progress'] = self.confirm_progress
        return action


# ===========================================================
# OmxYoloNode
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
        if self.cfg.patrol is None:
            raise RuntimeError("config.yaml 에 patrol 섹션 필요")
        
        self.get_logger().info(
            f"Patrol: scan_timeout={self.cfg.patrol.scan_timeout_sec}s, "
            f"max_queue={self.cfg.patrol.max_queue_size}")
        self.get_logger().info(
            f"Fire: hold={self.cfg.fire.hold_time_sec}s, "
            f"cooldown={self.cfg.fire.cooldown_sec}s, "
            f"lost_timeout={self.cfg.fire.lost_timeout_sec}s")
        self.get_logger().info(
            f"LOS check: enabled={self.cfg.patrol.los_check_enabled}, "
            f"threshold={self.cfg.patrol.los_cost_threshold}")
        self.get_logger().info(
            f"Queue markers: enabled={self.cfg.patrol.publish_queue_markers}")

        # TF
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        # Arm base offset
        self.declare_parameter('arm_base_x', 0.10)
        self.declare_parameter('arm_base_y', 0.00)
        self.declare_parameter('arm_base_z', 0.18)
        self.arm_offset = (
            self.get_parameter('arm_base_x').value,
            self.get_parameter('arm_base_y').value,
            self.get_parameter('arm_base_z').value,
        )
        self.get_logger().info(
            f"Arm base offset: x={self.arm_offset[0]}, "
            f"y={self.arm_offset[1]}, z={self.arm_offset[2]} m")

        # Costmap
        self.costmap: Optional[OccupancyGrid] = None
        self._costmap_logged = False

        # 내부 모듈
        self.detector = YoloDetector(self.cfg, logger=self.get_logger())
        self.ctrl = OmxController(self.cfg, dry_run=dry_run,
                                    logger=self.get_logger())
        self.sm = StateMachine(self.cfg, logger=self.get_logger())
        
        # 콜백 주입
        self.sm.los_check_fn = self.check_line_of_sight
        self.sm.waffle_pos_fn = self.get_waffle_xy

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
        self.pub_processed = self.create_publisher(PointStamped, '/omx/target_processed', 10)
        self.pub_target_lost = self.create_publisher(PointStamped, '/omx/target_lost', 10)
        self.pub_target_blocked = self.create_publisher(PointStamped, '/omx/target_blocked', 10)
        self.pub_progress = self.create_publisher(Float32, '/omx/aim_progress', 10)
        self.pub_queue_size = self.create_publisher(Int32, '/omx/queue_size', 10)
        self.pub_patrol_complete = self.create_publisher(Empty, '/omx/patrol_complete', 10)
        self.pub_queue_markers = self.create_publisher(
            MarkerArray, '/omx/queue_markers', 10)

        # Subscribers
        self.create_subscription(String, '/omx/control_mode',
                                 self.on_control_mode, 10)
        self.create_subscription(PointStamped, '/omx/target_in_map',
                                 self.on_target_in_map, 10)
        self.create_subscription(PointStamped, '/omx/boundary_in_map',
                                 self.on_boundary_in_map, 10)
        self.create_subscription(PointStamped, '/omx/patrol_in_map',
                                 self.on_patrol_in_map, 10)
        self.create_subscription(Bool, '/omx/arm_enable',
                                 self.on_arm_enable, 10)
        self.create_subscription(Empty, '/omx/abort',
                                 self.on_abort, 10)
        
        self.create_subscription(
            OccupancyGrid, self.cfg.patrol.costmap_topic,
            self.on_costmap, 1)

        self.timer = self.create_timer(self.control_period, self.loop)
        self.status_timer = self.create_timer(1.0, self.publish_periodic)

        self._last_state = self.sm.state

        self.get_logger().info(
            f"Timer: 메인 {self.cfg.ibvs.control_hz} Hz, 상태 1 Hz")
        self.get_logger().info(f"Initial armed: {self.sm.armed}")
        self.get_logger().info("=== Node ready ===")

    # ----- Costmap -----
    
    def on_costmap(self, msg: OccupancyGrid):
        self.costmap = msg
        if not self._costmap_logged:
            self.get_logger().info(
                f"Costmap 수신: {msg.info.width}x{msg.info.height} "
                f"cells @ {msg.info.resolution}m/cell")
            self._costmap_logged = True

    # ----- TF + LOS -----
    
    def get_waffle_xy(self):
        try:
            tr = self.tf_buffer.lookup_transform(
                'map', 'base_link', rclpy.time.Time(),
                timeout=Duration(seconds=0.1))
            return tr.transform.translation.x, tr.transform.translation.y
        except TransformException:
            return None

    def check_line_of_sight(self, target_map) -> LOSResult:
        if self.costmap is None:
            return LOSResult.UNKNOWN
        
        waffle = self.get_waffle_xy()
        if waffle is None:
            return LOSResult.UNKNOWN
        
        info = self.costmap.info
        res = info.resolution
        ox = info.origin.position.x
        oy = info.origin.position.y
        
        wgx = int((waffle[0] - ox) / res)
        wgy = int((waffle[1] - oy) / res)
        tgx = int((target_map[0] - ox) / res)
        tgy = int((target_map[1] - oy) / res)
        
        cells = bresenham_line(wgx, wgy, tgx, tgy)
        
        threshold = self.cfg.patrol.los_cost_threshold
        width = info.width
        height = info.height
        data = self.costmap.data
        
        has_unknown = False
        
        for cx, cy in cells:
            if cx < 0 or cx >= width or cy < 0 or cy >= height:
                has_unknown = True
                continue
            
            idx = cy * width + cx
            cost = data[idx]
            
            if cost == -1:
                has_unknown = True
            elif cost >= threshold:
                return LOSResult.BLOCKED
        
        return LOSResult.UNKNOWN if has_unknown else LOSResult.CLEAR

    def transform_map_to_arm_base(self, coord_map):
        ps = PointStamped()
        ps.header.frame_id = 'map'
        ps.header.stamp = self.get_clock().now().to_msg()
        ps.point.x, ps.point.y, ps.point.z = coord_map
        
        try:
            transform = self.tf_buffer.lookup_transform(
                target_frame='base_link',
                source_frame='map',
                time=rclpy.time.Time(),
                timeout=Duration(seconds=0.1),
            )
        except TransformException as e:
            self.get_logger().warn(f"TF lookup 실패: {e}")
            return None
        
        try:
            ps_base = do_transform_point(ps, transform)
        except Exception as e:
            self.get_logger().warn(f"do_transform_point 실패: {e}")
            return None
        
        return (
            ps_base.point.x - self.arm_offset[0],
            ps_base.point.y - self.arm_offset[1],
            ps_base.point.z - self.arm_offset[2],
        )

    # ----- Subscribers -----

    def on_control_mode(self, msg):
        if msg.data == "idle":
            self.sm.on_abort()
            self.ctrl.go_home()

    def on_target_in_map(self, msg: PointStamped):
        coord = (msg.point.x, msg.point.y, msg.point.z)
        self.sm.on_target(coord)

    def on_boundary_in_map(self, msg: PointStamped):
        coord = (msg.point.x, msg.point.y, msg.point.z)
        self.sm.on_boundary(coord)

    def on_patrol_in_map(self, msg: PointStamped):
        coord = (msg.point.x, msg.point.y, msg.point.z)
        self.sm.on_patrol(coord)

    def on_arm_enable(self, msg):
        self.sm.on_arm_enable(msg.data)

    def on_abort(self, msg):
        self.sm.on_abort()
        self.ctrl.go_home()

    # ----- Publishers -----

    def publish_periodic(self):
        msg = String()
        prefix = ""
        if self.dry_run:
            prefix = "dry_run_"
        if self.paused:
            prefix = "paused_"
        msg.data = f"{prefix}{self.sm.state.value}"
        self.pub_status.publish(msg)
        
        qmsg = Int32()
        qmsg.data = self.sm.queue_size()
        self.pub_queue_size.publish(qmsg)
        
        # 단계 G: 큐 마커
        self.publish_queue_markers()

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

    def _make_point_stamped(self, coord_map):
        msg = PointStamped()
        msg.header.frame_id = 'map'
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.point.x, msg.point.y, msg.point.z = coord_map
        return msg

    def publish_processed(self, coord_map):
        if coord_map is None:
            return
        self.pub_processed.publish(self._make_point_stamped(coord_map))

    def publish_target_lost(self, coord_map):
        if coord_map is None:
            return
        self.pub_target_lost.publish(self._make_point_stamped(coord_map))
        self.get_logger().info(f"[target_lost] 발행 (map): {coord_map}")

    def publish_target_blocked(self, coord_map, type_name=""):
        if coord_map is None:
            return
        self.pub_target_blocked.publish(self._make_point_stamped(coord_map))
        self.get_logger().info(
            f"[target_blocked] 발행 (map, {type_name}): {coord_map}")

    def publish_patrol_complete(self):
        self.pub_patrol_complete.publish(Empty())
        self.get_logger().info("[patrol_complete] 발행")

    def publish_queue_markers(self):
        """단계 G: 큐 좌표 + 처리 중 좌표 RViz 마커."""
        if not self.cfg.patrol.publish_queue_markers:
            return
        
        marker_array = MarkerArray()
        now_stamp = self.get_clock().now().to_msg()
        
        type_colors = {
            TargetType.TARGET:   (1.0, 0.2, 0.2),
            TargetType.BOUNDARY: (1.0, 0.6, 0.0),
            TargetType.PATROL:   (1.0, 1.0, 0.2),
        }
        type_sizes = {
            TargetType.TARGET:   0.25,
            TargetType.BOUNDARY: 0.18,
            TargetType.PATROL:   0.12,
        }
        
        # 이전 마커 정리
        delete_marker = Marker()
        delete_marker.header.frame_id = 'map'
        delete_marker.action = Marker.DELETEALL
        marker_array.markers.append(delete_marker)
        
        # 현재 처리 중 (초록, 큼)
        if self.sm.current_target:
            m = Marker()
            m.header.frame_id = 'map'
            m.header.stamp = now_stamp
            m.ns = 'queue_current'
            m.id = 0
            m.type = Marker.SPHERE
            m.action = Marker.ADD
            m.pose.position.x = self.sm.current_target.coord_map[0]
            m.pose.position.y = self.sm.current_target.coord_map[1]
            m.pose.position.z = self.sm.current_target.coord_map[2]
            m.pose.orientation.w = 1.0
            m.scale.x = m.scale.y = m.scale.z = 0.35
            m.color.r = 0.2
            m.color.g = 1.0
            m.color.b = 0.2
            m.color.a = 0.9
            marker_array.markers.append(m)
            
            t = Marker()
            t.header.frame_id = 'map'
            t.header.stamp = now_stamp
            t.ns = 'queue_current_label'
            t.id = 0
            t.type = Marker.TEXT_VIEW_FACING
            t.action = Marker.ADD
            t.pose.position.x = self.sm.current_target.coord_map[0]
            t.pose.position.y = self.sm.current_target.coord_map[1]
            t.pose.position.z = self.sm.current_target.coord_map[2] + 0.4
            t.pose.orientation.w = 1.0
            t.scale.z = 0.18
            t.color.r = t.color.g = t.color.b = 1.0
            t.color.a = 1.0
            t.text = f"[{self.sm.current_target.type_name}] CURRENT"
            marker_array.markers.append(t)
        
        # 큐 좌표들
        for i, entry in enumerate(self.sm.queue):
            m = Marker()
            m.header.frame_id = 'map'
            m.header.stamp = now_stamp
            m.ns = 'queue'
            m.id = i
            m.type = Marker.SPHERE
            m.action = Marker.ADD
            m.pose.position.x = entry.coord_map[0]
            m.pose.position.y = entry.coord_map[1]
            m.pose.position.z = entry.coord_map[2]
            m.pose.orientation.w = 1.0
            
            size = type_sizes.get(entry.target_type, 0.15)
            m.scale.x = m.scale.y = m.scale.z = size
            
            r, g, b = type_colors.get(entry.target_type, (0.5, 0.5, 0.5))
            m.color.r = r
            m.color.g = g
            m.color.b = b
            m.color.a = 0.8
            marker_array.markers.append(m)
            
            t = Marker()
            t.header.frame_id = 'map'
            t.header.stamp = now_stamp
            t.ns = 'queue_label'
            t.id = i
            t.type = Marker.TEXT_VIEW_FACING
            t.action = Marker.ADD
            t.pose.position.x = entry.coord_map[0]
            t.pose.position.y = entry.coord_map[1]
            t.pose.position.z = entry.coord_map[2] + 0.25
            t.pose.orientation.w = 1.0
            t.scale.z = 0.15
            t.color.r = t.color.g = t.color.b = 1.0
            t.color.a = 0.9
            t.text = f"{entry.type_name} {entry.distance:.1f}m"
            marker_array.markers.append(t)
        
        self.pub_queue_markers.publish(marker_array)

    # ----- Main loop -----

    def loop(self):
        frame = self.detector.read_frame()
        if frame is None:
            self.get_logger().warn("프레임 읽기 실패")
            return

        detected, error_norm, bbox, conf = self.detector.detect(frame)

        now = time.time()
        action = self.sm.update(detected, error_norm, now)

        blocked_entries = action.get('blocked_entries', [])
        for entry in blocked_entries:
            self.publish_target_blocked(entry.coord_map, entry.type_name)

        if not self.paused:
            if action['action'] == 'aim':
                coord_map = action['coord_map']
                coord_arm = self.transform_map_to_arm_base(coord_map)
                if coord_arm is None:
                    self.get_logger().warn(
                        f"TF 변환 실패, IDLE 강제 전이: {coord_map}")
                    self.sm.transition(State.IDLE)
                    self.sm.current_target = None
                else:
                    self.ctrl.aim_at_coord(*coord_arm)
                    self.get_logger().info(
                        f"AIM: map{coord_map} -> arm{coord_arm}")
            
            elif action['action'] == 'track':
                self.ctrl.step_ibvs(*action['error'])
            
            elif action['action'] == 'fire':
                processed_map = (self.sm.current_target.coord_map
                                 if self.sm.current_target else None)
                self.publish_fire()
                self.ctrl.fire()
                self.publish_processed(processed_map)
            
            elif action['action'] == 'target_lost':
                self.publish_target_lost(action.get('lost_coord_map'))
            
            elif action['action'] == 'home':
                self.ctrl.go_home()
        
        if action.get('patrol_complete', False):
            self.publish_patrol_complete()

        self.publish_detected(detected)
        if error_norm is not None:
            self.publish_error(error_norm[0], error_norm[1])
        self.publish_joint_state()
        self.publish_progress(action.get('confirm_progress', 0.0))
        self.publish_state_change()

        self.visualize(frame, detected, error_norm, bbox, conf, action)

        key = cv2.waitKey(1) & 0xFF
        self._handle_key(key)

        self.fps_n += 1
        if now - self.fps_t >= 1.0:
            self.fps_disp = self.fps_n / (now - self.fps_t)
            self.fps_t = now
            self.fps_n = 0

    def visualize(self, frame, detected, error_norm, bbox, conf, action):
        h, w = frame.shape[:2]
        cx, cy = w / 2.0, h / 2.0
        deadband = self.cfg.ibvs.deadband

        cv2.drawMarker(frame, (int(cx), int(cy)),
                       (0, 255, 255), cv2.MARKER_CROSS, 20, 1)
        dz_x = int(deadband * cx)
        dz_y = int(deadband * cy)
        cv2.rectangle(frame,
                      (int(cx) - dz_x, int(cy) - dz_y),
                      (int(cx) + dz_x, int(cy) + dz_y),
                      (80, 80, 80), 1)

        if detected and bbox:
            x1, y1, x2, y2 = bbox
            state_color = {
                State.IDLE: (180, 180, 180),
                State.AIMING: (255, 200, 0),
                State.SCANNING: (200, 255, 200),
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

        state_txt = f"[{self.sm.state.value.upper()}]"
        if self.paused:
            state_txt = f"[PAUSED|{self.sm.state.value}]"
        if self.dry_run:
            state_txt = f"[DRY|{self.sm.state.value}]"

        armed_txt = "ARMED" if self.sm.armed else "DISARMED"
        queue_txt = f"Q:{self.sm.queue_size()}"
        costmap_txt = "MAP:OK" if self.costmap else "MAP:--"
        
        type_txt = ""
        if self.sm.current_target:
            type_txt = (f" [{self.sm.current_target.type_name} "
                        f"{self.sm.current_target.distance:.1f}m]")

        cv2.putText(frame,
                    f"{state_txt}{type_txt} {armed_txt} {queue_txt} {costmap_txt}",
                    (10, 25), cv2.FONT_HERSHEY_SIMPLEX,
                    0.5, (255, 255, 255), 1)
        cv2.putText(frame,
                    f"yaw={math.degrees(self.ctrl.yaw):+.1f} "
                    f"pitch={math.degrees(self.ctrl.pitch):+.1f} "
                    f"fps={self.fps_disp:.1f}",
                    (10, 45), cv2.FONT_HERSHEY_SIMPLEX,
                    0.5, (255, 255, 255), 1)

        if (self.sm.state == State.TRACKING
                and self.sm.lost_start_t > 0.0):
            elapsed = time.time() - self.sm.lost_start_t
            timeout = self.cfg.fire.lost_timeout_sec
            lost_progress = min(1.0, elapsed / timeout)
            
            bar_x, bar_y, bar_w, bar_h = 10, h - 100, 200, 12
            cv2.rectangle(frame, (bar_x, bar_y),
                         (bar_x + bar_w, bar_y + bar_h),
                         (100, 100, 100), 1)
            cv2.rectangle(frame, (bar_x, bar_y),
                         (bar_x + int(bar_w * lost_progress), bar_y + bar_h),
                         (0, 100, 255), -1)
            cv2.putText(frame, f"LOST {elapsed:.1f}/{timeout:.1f}s",
                        (bar_x + bar_w + 10, bar_y + 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 100, 100), 1)

        if self.sm.state == State.SCANNING:
            scan_timeout = (self.cfg.patrol.scan_timeout_sec
                            if self.cfg.patrol else 2.0)
            elapsed = time.time() - self.sm.scan_start_t
            scan_progress = min(1.0, elapsed / scan_timeout)
            
            bar_x, bar_y, bar_w, bar_h = 10, h - 80, 200, 12
            cv2.rectangle(frame, (bar_x, bar_y),
                         (bar_x + bar_w, bar_y + bar_h),
                         (100, 100, 100), 1)
            cv2.rectangle(frame, (bar_x, bar_y),
                         (bar_x + int(bar_w * scan_progress), bar_y + bar_h),
                         (100, 255, 100), -1)
            cv2.putText(frame, f"SCAN {elapsed:.1f}/{scan_timeout:.1f}s",
                        (bar_x + bar_w + 10, bar_y + 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 255, 255), 1)

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

        cv2.putText(frame, "p:pause a:arm h:home/clear ESC:quit",
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
            self.get_logger().info("Home + 큐 비움 (수동)")
            self.sm.on_abort()
            self.ctrl.go_home()

    def destroy_node(self):
        if hasattr(self, 'detector'):
            self.detector.release()
        cv2.destroyAllWindows()
        if hasattr(self, 'ctrl'):
            self.ctrl.disconnect()
        super().destroy_node()


def main(args=None):
    import argparse
    parser = argparse.ArgumentParser(
        description="OMX YOLO ROS 2 node - Stage G")
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