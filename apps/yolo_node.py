#!/usr/bin/env python3
"""OMX YOLO tracker - 단계 H2.

진화 단계:
    A : 우선순위 큐 (heapq)
    D : map 좌표 기반 큐 (TF 변환)
    F : LOS 검사 + TargetType 카테고리
    G : 거리 정렬 + RViz 마커
    H1: waffle_node.py (Nav2 클라이언트 분리)
    H2: CHECK_VIEW + VIEW_POSE v1 + WAITING_NAV + 큐 분리  ← 이번 단계

좌표 종류 (TargetType):
    TARGET   (priority=0)  - 외부 신뢰 좌표. main_queue.
    BOUNDARY (priority=5)  - 이동 중 사주 경계. boundary_queue. H4 에서 자동 생성.
    PATROL   (priority=10) - 탐색 대상 좌표. main_queue.

큐 분리 (H2):
    main_queue:     TARGET + PATROL. IDLE 에서 pop, CHECK_VIEW → AIMING or WAITING_NAV.
    boundary_queue: BOUNDARY. WAITING_NAV 중에만 pop, 와플 도착 시 일괄 폐기.

상태 머신:
    IDLE → [CHECK_VIEW]
           가능: AIMING → SCANNING → TRACKING → CONFIRMING → FIRING → COOLDOWN → IDLE
           불가: WAITING_NAV (와플 이동) → 도착 시 AIMING
    
    WAITING_NAV 중에는 boundary_queue 처리:
        boundary 가 있으면 잠시 AIMING → ... → COOLDOWN → WAITING_NAV 복귀
        nav_result=succeeded → boundary 폐기 + parent AIMING

CHECK_VIEW 정책 (H2):
    현 위치에서 PATROL/TARGET 좌표를 조준 가능?
        - LOS clear/unknown (BOUNDARY 정책과 동일)
        - OMX yaw 각도 한계 안
        - 거리 적정 범위
    가능 → AIMING / 불가 → VIEW_POSE 계산 → /omx/nav_goal publish → WAITING_NAV.

VIEW_POSE v1:
    target 으로부터 stand_off_distance 만큼, 와플→target 직선상 떨어진 점.
    yaw = target 방향.
    실패 시 fallback 없음 (v2 에서).

협력 (waffle_node.py):
    /omx/nav_goal       PoseStamped   yolo_node → waffle: 이동 목표 (VIEW_POSE)
    /waffle/nav_result  String        waffle → yolo_node: succeeded/aborted/canceled/rejected

Subscribe:
    /omx/target_in_map        PointStamped   TARGET (main)
    /omx/boundary_in_map      PointStamped   BOUNDARY 외부 입력 (H4 에서 내부 생성과 공존)
    /omx/patrol_in_map        PointStamped   PATROL (main)
    /omx/control_mode         String         idle
    /omx/arm_enable           Bool
    /omx/abort                Empty
    /global_costmap/costmap   OccupancyGrid
    /waffle/nav_result        String         ← H2 신규

Publish:
    /omx/status, /omx/state, /omx/target_detected, /omx/error_norm
    /omx/joint_state, /omx/fire, /omx/aim_progress
    /omx/target_processed   PointStamped
    /omx/target_lost        PointStamped
    /omx/target_blocked     PointStamped
    /omx/queue_size, /omx/patrol_complete
    /omx/queue_markers      MarkerArray
    /omx/nav_goal           PoseStamped    ← H2 신규
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
from geometry_msgs.msg import Point, PointStamped, PoseStamped, Quaternion
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


# ===========================================================
# Enums
# ===========================================================

class State(Enum):
    IDLE = "idle"
    AIMING = "aiming"
    SCANNING = "scanning"
    TRACKING = "tracking"
    CONFIRMING = "confirming"
    FIRING = "firing"
    COOLDOWN = "cooldown"
    WAITING_NAV = "waiting_nav"   # H2 신규


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
# TargetEntry
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
    # H2 신규 (H4 에서 사용). BOUNDARY 가 어느 parent 의 자식인지.
    parent_id: Optional[int] = field(compare=False, default=None)

    def __post_init__(self):
        self._update_sort_key()

    def update_distance(self, waffle_xy):
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


# ===========================================================
# Bresenham (LOS 셀 순회)
# ===========================================================

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
# YoloDetector (변경 없음)
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
# OmxController (wrist_roll + gripper 떼어낸 버전)
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
            # gripper 떼어냄 - 격발은 별도 MCU (Jetson GPIO)
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
            # wrist_roll 떼어냄
            for m in ("elbow_flex", "wrist_flex"):
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
        """격발 신호. 실제 격발은 별도 MCU 가 /omx/fire 토픽 받아 처리.
        
        OmxYoloNode 가 /omx/fire 토픽을 별도 publish 하므로,
        여기서는 cooldown UX 위한 짧은 대기만.
        """
        if self.dry_run:
            self._log("[dry-run] 격발 신호 시뮬레이션")
        else:
            self._log("격발 신호 발사 (외부 MCU 처리)")
        time.sleep(0.5)

    def read_joint_positions_rad(self):
        if self.dry_run or self.bus is None or not self.bus.is_connected:
            return {
                "shoulder_pan": self.yaw,
                "shoulder_lift": self.pitch,
                "elbow_flex": 0.0,
                "wrist_flex": 0.0,
            }
        ticks = self.bus.sync_read("Present_Position", normalize=False)
        home = self.cfg.calibration.home
        sign = self.cfg.calibration.sign
        result = {}
        for name in MOTOR_ORDER:
            result[name] = (ticks[name] - home[name]) / RAD2TICK / sign.get(name, 1)
        return result


# ===========================================================
# StateMachine (H2 큐 분리 + WAITING_NAV)
# ===========================================================

class StateMachine:
    def __init__(self, cfg: Config, logger=None):
        self.cfg = cfg
        self.logger = logger
        self.state = State.IDLE

        # H2: 큐 분리
        self.main_queue: list[TargetEntry] = []      # TARGET + PATROL
        self.boundary_queue: list[TargetEntry] = []  # BOUNDARY only

        # H2: 부모 + focus 분리
        self.current_parent: Optional[TargetEntry] = None  # 처리 중 TARGET/PATROL
        self.current_focus: Optional[TargetEntry] = None   # OMX 가 조준 중 (parent or boundary)

        # H2: nav_result 비동기 처리
        self.nav_pending_result: Optional[str] = None

        # 타이머/플래그
        self.aim_start_t: float = 0.0      # H2.1: AIMING 진입 시각
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
        self.los_check_fn = None              # (coord_map) -> LOSResult
        self.waffle_pos_fn = None             # () -> (x, y) or None
        self.check_view_fn = None             # H2: (coord_map) -> bool
        self.compute_view_pose_fn = None      # H2: (coord_map) -> (x,y,yaw) or None

    def _log(self, msg):
        if self.logger:
            self.logger.info(msg)
        else:
            print(msg)

    def transition(self, new_state: State):
        if self.state != new_state:
            self._log(f"State: {self.state.value} -> {new_state.value}")
            self.state = new_state

    # ----- Queue 조작 -----

    def add_target(self, coord, target_type: TargetType,
                   parent_id: Optional[int] = None) -> bool:
        # 큐 선택
        if target_type == TargetType.BOUNDARY:
            queue = self.boundary_queue
            max_size = (self.cfg.boundary.max_queue_size
                        if self.cfg.boundary else 10)
        else:
            queue = self.main_queue
            max_size = (self.cfg.patrol.max_queue_size
                        if self.cfg.patrol else 20)

        # 격발 중 main 큐 추가 금지 (BOUNDARY 는 OK - 큐만 쌓임)
        if (target_type != TargetType.BOUNDARY
                and self.state in (State.CONFIRMING, State.FIRING)):
            self._log(f"좌표 무시 (state={self.state.value}, 격발 우선): "
                      f"type={target_type.name}")
            return False

        if self._is_duplicate(coord, target_type):
            self._log(f"좌표 중복 무시: {coord} ({target_type.name})")
            return False

        if len(queue) >= max_size:
            removed = self._remove_oldest(queue)
            if not removed:
                self._log(f"큐 가득 ({max_size}), 추가 거부")
                return False

        entry = TargetEntry(
            priority=int(target_type),
            coord_map=coord,
            target_type=target_type,
            arrival_time=time.time(),
            parent_id=parent_id,
        )
        if self.waffle_pos_fn:
            entry.update_distance(self.waffle_pos_fn())

        heapq.heappush(queue, entry)
        if target_type != TargetType.BOUNDARY:
            self.patrol_complete_sent = False

        self._log(f"큐 추가: type={target_type.name} "
                  f"coord={coord} dist={entry.distance:.2f}m, "
                  f"main={len(self.main_queue)} bnd={len(self.boundary_queue)}")
        return True

    def _is_duplicate(self, coord, target_type) -> bool:
        if not self.cfg.patrol:
            return False
        threshold = self.cfg.patrol.duplicate_threshold_m

        # BOUNDARY 는 다른 큐와 비교 안 함 (단명적)
        check_queues = ([self.boundary_queue]
                        if target_type == TargetType.BOUNDARY
                        else [self.main_queue])

        if self.last_processed and target_type != TargetType.BOUNDARY:
            if self._distance(coord, self.last_processed) < threshold:
                return True

        for q in check_queues:
            for entry in q:
                if self._distance(coord, entry.coord_map) < threshold:
                    return True

        if (self.current_parent
                and target_type != TargetType.BOUNDARY):
            if self._distance(coord, self.current_parent.coord_map) < threshold:
                return True

        return False

    def _distance(self, a, b):
        return math.sqrt(sum((ai - bi)**2 for ai, bi in zip(a, b)))

    def _remove_oldest(self, queue) -> bool:
        if not queue:
            return False
        oldest_idx = 0
        for i, entry in enumerate(queue):
            if entry.count < queue[oldest_idx].count:
                oldest_idx = i
        removed = queue.pop(oldest_idx)
        heapq.heapify(queue)
        self._log(f"큐 가득, 오래된 {removed.type_name} 제거")
        return True

    def _pop_with_los(self, queue, waffle_xy=None):
        """LOS 검사 통과한 entry pop. 거리 기반 재정렬 후."""
        blocked_entries = []

        if waffle_xy is not None:
            for entry in queue:
                entry.update_distance(waffle_xy)
            heapq.heapify(queue)

        while queue:
            entry = heapq.heappop(queue)

            if (not self.cfg.patrol
                    or not self.cfg.patrol.los_check_enabled
                    or self.los_check_fn is None):
                return entry, blocked_entries

            result = self.los_check_fn(entry.coord_map)

            if result == LOSResult.CLEAR:
                return entry, blocked_entries
            elif result == LOSResult.UNKNOWN:
                if entry.target_type == TargetType.BOUNDARY:
                    self._log(f"LOS UNKNOWN, BOUNDARY 폐기: {entry.coord_map}")
                    blocked_entries.append(entry)
                    continue
                else:
                    return entry, blocked_entries
            elif result == LOSResult.BLOCKED:
                if entry.target_type == TargetType.BOUNDARY:
                    self._log(f"LOS BLOCKED, BOUNDARY 폐기: {entry.coord_map}")
                    blocked_entries.append(entry)
                    continue
                else:
                    return entry, blocked_entries

        return None, blocked_entries

    def clear_boundary_queue(self):
        cleared = len(self.boundary_queue)
        self.boundary_queue.clear()
        if cleared > 0:
            self._log(f"BOUNDARY 큐 {cleared}개 일괄 폐기")

    def queue_size(self) -> int:
        return len(self.main_queue) + len(self.boundary_queue)

    @property
    def queue(self):
        """RViz 마커용. main + boundary 통합 view."""
        return self.main_queue + self.boundary_queue

    # ----- 입력 핸들러 -----

    def on_target(self, coord) -> bool:
        return self.add_target(coord, TargetType.TARGET)

    def on_boundary(self, coord, parent_id=None) -> bool:
        return self.add_target(coord, TargetType.BOUNDARY, parent_id=parent_id)

    def on_patrol(self, coord) -> bool:
        return self.add_target(coord, TargetType.PATROL)

    def on_nav_result(self, result: str):
        """waffle_node 의 nav_result 비동기 수신. 다음 tick 에서 처리."""
        self.nav_pending_result = result
        self._log(f"nav_result 받음: {result}")

    def on_abort(self):
        self._log("ABORT - IDLE + 모든 큐 비움")
        self.transition(State.IDLE)
        self.main_queue.clear()
        self.boundary_queue.clear()
        self.current_parent = None
        self.current_focus = None
        self.confirm_progress = 0.0
        self.cooldown_home_sent = False
        self.patrol_complete_sent = True
        self.lost_start_t = 0.0
        self.nav_pending_result = None

    def on_arm_enable(self, armed: bool):
        self.armed = armed
        self._log(f"Armed: {armed}")

    # ----- update() 메인 -----

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
            'nav_goal_xyyaw': None,         # H2: (x, y, yaw) for /omx/nav_goal
            'focus_is_boundary': False,     # H2: 시각화용
        }

        # 1. nav_result 처리 (H2.1: WAITING_NAV state 일 때만 즉시 적용.
        #    boundary 처리 중 (state=AIMING/SCANNING/...) 이면 큐에 남겨두고,
        #    _on_focus_done 으로 WAITING_NAV 복귀 후 다음 tick 에서 처리.)
        if (self.nav_pending_result is not None
                and self.state == State.WAITING_NAV):
            result = self.nav_pending_result
            self.nav_pending_result = None
            self._handle_nav_result(result, action, now)

        # 2. State 분기
        if self.state == State.IDLE:
            self._on_idle(detected, action, now)

        elif self.state == State.WAITING_NAV:
            self._on_waiting_nav(action, now)

        elif self.state == State.AIMING:
            # H2.1: aim_settle_sec 동안 OMX 모터가 목표 각도로 이동.
            # 그동안 action='wait', 외부에 별다른 영향 없음.
            aim_settle = self.cfg.fire.aim_settle_sec
            if now - self.aim_start_t >= aim_settle:
                self.scan_start_t = now
                self.transition(State.SCANNING)

        elif self.state == State.SCANNING:
            self._on_scanning(detected, now, action)

        elif self.state == State.TRACKING:
            self._on_tracking(detected, error_norm, now, action)

        elif self.state == State.CONFIRMING:
            self._on_confirming(detected, error_norm, now, action)

        elif self.state == State.FIRING:
            action['action'] = 'fire'
            self.transition(State.COOLDOWN)
            self.cooldown_until = now + self.cfg.fire.cooldown_sec
            self.cooldown_home_sent = False

        elif self.state == State.COOLDOWN:
            self._on_cooldown(now, action)

        action['state'] = self.state
        action['confirm_progress'] = self.confirm_progress
        action['focus_is_boundary'] = (
            self.current_focus is not None
            and self.current_focus.target_type == TargetType.BOUNDARY)
        return action

    # ----- 핸들러: nav_result -----

    def _handle_nav_result(self, result: str, action: dict, now: float):
        """nav_result 적용. update() 시작에서 state == WAITING_NAV 가 보장됨.

        boundary 처리 중에 nav_result 가 들어오면 update() 의 조건문에서
        nav_pending_result 가 큐에 남고, _on_focus_done 으로 WAITING_NAV 복귀 후
        다음 tick 에서 다시 시도된다.
        """
        # 도착 정책: boundary 큐 일괄 폐기 (적용 시점에만)
        self.clear_boundary_queue()

        if result == "succeeded":
            if self.current_parent is not None:
                # parent 의 AIMING 진입
                self.current_focus = self.current_parent
                self.transition(State.AIMING)
                self.aim_start_t = now    # H2.1
                action['action'] = 'aim'
                action['coord_map'] = self.current_parent.coord_map
                self._log(f"와플 도착, parent AIMING: "
                          f"{self.current_parent.coord_map}")
            else:
                self._log("nav_result succeeded 인데 parent 없음. IDLE 로.")
                self.transition(State.IDLE)
        else:
            # aborted / canceled / rejected
            self._log(f"Nav 실패 ({result}), parent 폐기")
            self.current_parent = None
            self.current_focus = None
            self.transition(State.IDLE)

    # ----- 핸들러: IDLE -----

    def _on_idle(self, detected: bool, action: dict, now: float):
        # 처리 끝났으니 focus/parent 정리
        if self.current_focus is not None:
            self.current_focus = None

        # main_queue 에서 pop 시도
        if self.main_queue:
            waffle_xy = self.waffle_pos_fn() if self.waffle_pos_fn else None
            entry, blocked = self._pop_with_los(self.main_queue, waffle_xy)
            action['blocked_entries'] = blocked

            if entry is None:
                return

            self.current_parent = entry
            self._log(f"main_queue pop: {entry.type_name} "
                      f"{entry.coord_map} dist={entry.distance:.2f}m")

            # CHECK_VIEW
            can_view = (self.check_view_fn is None
                        or self.check_view_fn(entry.coord_map))

            if can_view:
                # 현 위치에서 조준 가능 → AIMING
                self.current_focus = entry
                self.last_processed = entry.coord_map
                self.transition(State.AIMING)
                self.aim_start_t = now    # H2.1
                action['action'] = 'aim'
                action['coord_map'] = entry.coord_map
                self._log("CHECK_VIEW: 현 위치 조준 가능 -> AIMING")
            else:
                # VIEW_POSE 계산 → 와플 이동.
                # H2.1: 도착 시 yaw 는 main_queue 의 다음 entry 방향
                # (다음 작업으로 빨리 출발하기 위해. OMX 가 ±180° 회전해서 조준).
                next_target_map = (self.main_queue[0].coord_map
                                   if self.main_queue else None)
                view_pose = (self.compute_view_pose_fn(
                                 entry.coord_map, next_target_map)
                             if self.compute_view_pose_fn else None)
                if view_pose is None:
                    self._log(f"VIEW_POSE 계산 실패, parent 폐기")
                    self.last_processed = entry.coord_map
                    self.current_parent = None
                    return
                self.last_processed = entry.coord_map
                self.transition(State.WAITING_NAV)
                action['action'] = 'nav_goal'
                action['nav_goal_xyyaw'] = view_pose
                self._log(f"CHECK_VIEW: 불가, VIEW_POSE={view_pose} "
                          f"-> WAITING_NAV")

        elif self.armed and detected:
            self._log("Autonomous detection -> TRACKING")
            self.current_focus = None  # autotrack 은 focus 없음
            self.transition(State.TRACKING)

        else:
            if not self.patrol_complete_sent:
                action['patrol_complete'] = True
                self.patrol_complete_sent = True
                self._log("정찰 완료 - main_queue 비었음")

    # ----- 핸들러: WAITING_NAV -----

    def _on_waiting_nav(self, action: dict, now: float):
        """와플 이동 대기. boundary_queue 에 있으면 처리."""
        # 이미 boundary 처리 중이면 (current_focus 있으면) 그대로 진행
        # 실제로는 AIMING/SCANNING 등 다른 state 에 있어야 하는데
        # WAITING_NAV state 라는 건 boundary 처리 안 하고 그냥 대기 중.

        if self.current_focus is not None:
            return  # 이미 처리 중

        # boundary_queue 에서 pop 시도
        if self.boundary_queue:
            waffle_xy = self.waffle_pos_fn() if self.waffle_pos_fn else None
            entry, blocked = self._pop_with_los(self.boundary_queue, waffle_xy)
            action['blocked_entries'] = blocked

            if entry is not None:
                self.current_focus = entry
                self.transition(State.AIMING)
                self.aim_start_t = now    # H2.1
                action['action'] = 'aim'
                action['coord_map'] = entry.coord_map
                self._log(f"WAITING_NAV 중 boundary AIMING: "
                          f"{entry.coord_map}")
        # else: 그냥 대기. nav_result 콜백을 기다림.

    # ----- 핸들러: SCANNING / TRACKING / CONFIRMING / COOLDOWN -----

    def _on_scanning(self, detected, now, action):
        if detected:
            self.lost_start_t = 0.0
            self.transition(State.TRACKING)
        else:
            scan_timeout = (self.cfg.patrol.scan_timeout_sec
                            if self.cfg.patrol else 2.0)
            if now - self.scan_start_t >= scan_timeout:
                self._log(f"SCANNING {scan_timeout}s 끝, 표적 없음")
                self._on_focus_done()

    def _on_tracking(self, detected, error_norm, now, action):
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
                self._log(f"TRACKING 표적 {timeout:.1f}s 잃음")
                if self.current_focus is not None:
                    self.last_processed = self.current_focus.coord_map
                    action['lost_coord_map'] = self.current_focus.coord_map
                action['action'] = 'target_lost'
                self.lost_start_t = 0.0
                self._on_focus_done()

    def _on_confirming(self, detected, error_norm, now, action):
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

    def _on_cooldown(self, now, action):
        if now >= self.cooldown_until:
            self._log("Cooldown 끝")
            self.confirm_progress = 0.0
            self.cooldown_home_sent = False
            self._on_focus_done()
        else:
            if not self.cooldown_home_sent:
                action['action'] = 'home'
                self.cooldown_home_sent = True

    # ----- focus 완료 처리 -----

    def _on_focus_done(self):
        """현재 focus 종료. focus 가 parent 면 IDLE, boundary 면 WAITING_NAV 복귀."""
        if self.current_focus is None:
            self.transition(State.IDLE)
            return

        is_boundary = (self.current_focus.target_type == TargetType.BOUNDARY)
        self.current_focus = None
        self.confirm_progress = 0.0

        if is_boundary:
            # boundary 처리 끝. parent 아직 이동 중이면 WAITING_NAV 복귀.
            if self.current_parent is not None:
                self.transition(State.WAITING_NAV)
                self._log("Boundary 처리 끝 -> WAITING_NAV 복귀")
            else:
                # parent 없으면 IDLE (autotrack 케이스 등)
                self.transition(State.IDLE)
        else:
            # main parent 처리 끝
            self.current_parent = None
            self.transition(State.IDLE)


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
        if self.cfg.view_pose is None:
            raise RuntimeError("config.yaml 에 view_pose 섹션 필요")

        self.get_logger().info(
            f"VIEW_POSE: yaw_limit={self.cfg.view_pose.omx_yaw_limit_deg}°, "
            f"dist=[{self.cfg.view_pose.min_distance_m}, "
            f"{self.cfg.view_pose.max_distance_m}]m, "
            f"stand_off={self.cfg.view_pose.stand_off_distance}m")

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
        self.sm.check_view_fn = self.check_view              # H2
        self.sm.compute_view_pose_fn = self.compute_view_pose  # H2

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
        # H2 신규
        self.pub_nav_goal = self.create_publisher(
            PoseStamped, '/omx/nav_goal', 10)

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
        # H2 신규
        self.create_subscription(String, '/waffle/nav_result',
                                 self.on_nav_result, 10)

        self.timer = self.create_timer(self.control_period, self.loop)
        self.status_timer = self.create_timer(1.0, self.publish_periodic)

        self._last_state = self.sm.state

        self.get_logger().info(
            f"Timer: 메인 {self.cfg.ibvs.control_hz} Hz, 상태 1 Hz")
        self.get_logger().info(f"Initial armed: {self.sm.armed}")
        self.get_logger().info("=== Node ready (H2) ===")

    # ----- Costmap -----

    def on_costmap(self, msg: OccupancyGrid):
        self.costmap = msg
        if not self._costmap_logged:
            self.get_logger().info(
                f"Costmap 수신: {msg.info.width}x{msg.info.height} "
                f"cells @ {msg.info.resolution}m/cell")
            self._costmap_logged = True

    # ----- TF helpers -----

    def get_waffle_xy(self):
        try:
            tr = self.tf_buffer.lookup_transform(
                'map', 'base_link', rclpy.time.Time(),
                timeout=Duration(seconds=0.1))
            return tr.transform.translation.x, tr.transform.translation.y
        except TransformException:
            return None

    def get_waffle_xy_yaw(self):
        """H2: 와플 (x, y, yaw) in map frame. H4 BOUNDARY 자동 생성에 사용."""
        try:
            tr = self.tf_buffer.lookup_transform(
                'map', 'base_link', rclpy.time.Time(),
                timeout=Duration(seconds=0.1))
            q = tr.transform.rotation
            # quaternion -> yaw
            yaw = math.atan2(
                2.0 * (q.w * q.z + q.x * q.y),
                1.0 - 2.0 * (q.y * q.y + q.z * q.z))
            return (tr.transform.translation.x,
                    tr.transform.translation.y,
                    yaw)
        except TransformException:
            return None

    # ----- LOS -----

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

    # ----- H2: CHECK_VIEW + VIEW_POSE -----

    def check_view(self, target_map) -> bool:
        """현재 와플 위치에서 target_map 을 OMX 가 조준 가능한가?
        
        판정 기준:
            1. LOS clear 또는 unknown (blocked 는 불가)
            2. arm_base 좌표 기준 OMX yaw 한계 안
            3. 거리 적정 범위
        """
        # 1. LOS
        los = self.check_line_of_sight(target_map)
        if los == LOSResult.BLOCKED:
            self.get_logger().info(f"CHECK_VIEW NG: LOS BLOCKED")
            return False

        # 2, 3. arm_base 변환 후 yaw/거리
        arm = self.transform_map_to_arm_base(target_map)
        if arm is None:
            self.get_logger().info(f"CHECK_VIEW NG: TF 변환 실패")
            return False

        ax, ay, az = arm
        yaw_deg = math.degrees(math.atan2(ay, ax))
        distance = math.sqrt(ax*ax + ay*ay + az*az)

        vp = self.cfg.view_pose
        if abs(yaw_deg) > vp.omx_yaw_limit_deg:
            self.get_logger().info(
                f"CHECK_VIEW NG: yaw={yaw_deg:+.1f}° > {vp.omx_yaw_limit_deg}°")
            return False
        if distance < vp.min_distance_m or distance > vp.max_distance_m:
            self.get_logger().info(
                f"CHECK_VIEW NG: dist={distance:.2f}m "
                f"out of [{vp.min_distance_m}, {vp.max_distance_m}]")
            return False

        self.get_logger().info(
            f"CHECK_VIEW OK: yaw={yaw_deg:+.1f}° dist={distance:.2f}m")
        return True

    def compute_view_pose(self, target_map, next_target_map=None):
        """target 으로부터 stand_off_distance 만큼 떨어진 와플 위치 + yaw.

        Args:
            target_map: VIEW_POSE 의 기준 (와플이 도착할 위치 계산용).
            next_target_map: 도착 후 와플이 향할 다음 target.
                None 이면 target_map 방향 (기존 v1 fallback).

        Returns: (x, y, yaw) in map frame, 또는 None.
        """
        waffle = self.get_waffle_xy()
        if waffle is None:
            self.get_logger().warn("compute_view_pose: 와플 위치 모름")
            return None

        tx, ty, _ = target_map
        wx, wy = waffle

        dx = tx - wx
        dy = ty - wy
        d = math.hypot(dx, dy)
        if d < 1e-3:
            # 와플이 이미 target 위치. 임의 방향으로 stand off
            self.get_logger().warn(
                "compute_view_pose: 와플이 target 위에 있음")
            return None

        # 단위 벡터: 와플 → target
        ux = dx / d
        uy = dy / d
        stand_off = self.cfg.view_pose.stand_off_distance

        # VIEW_POSE = target 에서 stand_off 만큼 와플 쪽으로 떨어진 점
        vp_x = tx - stand_off * ux
        vp_y = ty - stand_off * uy

        # H2.1: yaw 결정
        if next_target_map is not None:
            nx, ny, _ = next_target_map
            ndx = nx - vp_x
            ndy = ny - vp_y
            if math.hypot(ndx, ndy) > 1e-3:
                vp_yaw = math.atan2(ndy, ndx)
                self.get_logger().info(
                    f"VIEW_POSE yaw: 다음 main_queue 방향 "
                    f"{math.degrees(vp_yaw):+.1f}° "
                    f"(OMX 가 ±180° 회전해서 현재 target 조준)")
                return (vp_x, vp_y, vp_yaw)

        # fallback: 현재 target 방향
        vp_yaw = math.atan2(ty - vp_y, tx - vp_x)
        return (vp_x, vp_y, vp_yaw)

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
        # H2: 외부 토픽 입력 (디버그/수동). H4 에서 내부 자동 생성과 공존.
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

    def on_nav_result(self, msg: String):
        """H2: waffle_node 가 발행한 Nav2 액션 결과."""
        self.sm.on_nav_result(msg.data)

    # ----- Publishers -----

    def publish_periodic(self):
        msg = String()
        prefix = "dry_run_" if self.dry_run else ""
        if self.paused:
            prefix = "paused_"
        msg.data = f"{prefix}{self.sm.state.value}"
        self.pub_status.publish(msg)

        qmsg = Int32()
        qmsg.data = self.sm.queue_size()
        self.pub_queue_size.publish(qmsg)

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
        self.get_logger().info(f"[target_lost] 발행: {coord_map}")

    def publish_target_blocked(self, coord_map, type_name=""):
        if coord_map is None:
            return
        self.pub_target_blocked.publish(self._make_point_stamped(coord_map))
        self.get_logger().info(
            f"[target_blocked] 발행 ({type_name}): {coord_map}")

    def publish_patrol_complete(self):
        self.pub_patrol_complete.publish(Empty())
        self.get_logger().info("[patrol_complete] 발행")

    def publish_nav_goal(self, view_pose):
        """H2: VIEW_POSE 를 PoseStamped 로 발행."""
        x, y, yaw = view_pose
        msg = PoseStamped()
        msg.header.frame_id = 'map'
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose.position.x = x
        msg.pose.position.y = y
        msg.pose.position.z = 0.0
        # yaw -> quaternion
        msg.pose.orientation.z = math.sin(yaw / 2.0)
        msg.pose.orientation.w = math.cos(yaw / 2.0)
        self.pub_nav_goal.publish(msg)
        self.get_logger().info(
            f"[nav_goal] 발행: ({x:+.2f}, {y:+.2f}) "
            f"yaw={math.degrees(yaw):+.1f}°")

    def publish_queue_markers(self):
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

        delete_marker = Marker()
        delete_marker.header.frame_id = 'map'
        delete_marker.action = Marker.DELETEALL
        marker_array.markers.append(delete_marker)

        # 현재 focus (parent 또는 boundary)
        if self.sm.current_focus is not None:
            m = Marker()
            m.header.frame_id = 'map'
            m.header.stamp = now_stamp
            m.ns = 'queue_current'
            m.id = 0
            m.type = Marker.SPHERE
            m.action = Marker.ADD
            m.pose.position.x = self.sm.current_focus.coord_map[0]
            m.pose.position.y = self.sm.current_focus.coord_map[1]
            m.pose.position.z = self.sm.current_focus.coord_map[2]
            m.pose.orientation.w = 1.0
            m.scale.x = m.scale.y = m.scale.z = 0.35
            m.color.r = 0.2
            m.color.g = 1.0
            m.color.b = 0.2
            m.color.a = 0.9
            marker_array.markers.append(m)

        # 모든 큐 entry
        all_entries = list(self.sm.main_queue) + list(self.sm.boundary_queue)
        for i, entry in enumerate(all_entries):
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

        # blocked entries 알림
        for entry in action.get('blocked_entries', []):
            self.publish_target_blocked(entry.coord_map, entry.type_name)

        if not self.paused:
            if action['action'] == 'aim':
                coord_map = action['coord_map']
                coord_arm = self.transform_map_to_arm_base(coord_map)
                if coord_arm is None:
                    self.get_logger().warn(
                        f"TF 변환 실패, focus 종료: {coord_map}")
                    self.sm._on_focus_done()
                else:
                    self.ctrl.aim_at_coord(*coord_arm)
                    self.get_logger().info(
                        f"AIM: map{coord_map} -> arm{coord_arm}")

            elif action['action'] == 'track':
                self.ctrl.step_ibvs(*action['error'])

            elif action['action'] == 'fire':
                processed_map = (self.sm.current_focus.coord_map
                                 if self.sm.current_focus else None)
                self.publish_fire()
                self.ctrl.fire()
                self.publish_processed(processed_map)

            elif action['action'] == 'target_lost':
                self.publish_target_lost(action.get('lost_coord_map'))

            elif action['action'] == 'home':
                self.ctrl.go_home()

            elif action['action'] == 'nav_goal':
                # H2 신규
                vp = action['nav_goal_xyyaw']
                if vp is not None:
                    self.publish_nav_goal(vp)

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
                State.WAITING_NAV: (100, 200, 255),   # H2: 하늘색
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
        queue_txt = (f"Q:m{len(self.sm.main_queue)}"
                     f"/b{len(self.sm.boundary_queue)}")
        costmap_txt = "MAP:OK" if self.costmap else "MAP:--"

        focus_txt = ""
        if self.sm.current_focus is not None:
            is_b = action.get('focus_is_boundary', False)
            tag = "B" if is_b else self.sm.current_focus.type_name[0]
            focus_txt = (f" [{tag}:{self.sm.current_focus.distance:.1f}m]")

        cv2.putText(frame,
                    f"{state_txt}{focus_txt} {armed_txt} {queue_txt} {costmap_txt}",
                    (10, 25), cv2.FONT_HERSHEY_SIMPLEX,
                    0.5, (255, 255, 255), 1)
        cv2.putText(frame,
                    f"yaw={math.degrees(self.ctrl.yaw):+.1f} "
                    f"pitch={math.degrees(self.ctrl.pitch):+.1f} "
                    f"fps={self.fps_disp:.1f}",
                    (10, 45), cv2.FONT_HERSHEY_SIMPLEX,
                    0.5, (255, 255, 255), 1)

        # TRACKING lost progress
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

        # SCANNING progress
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

        # CONFIRMING progress
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
            self.get_logger().info("Home + 모든 큐 비움 (수동)")
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
        description="OMX YOLO ROS 2 node - Stage H2")
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