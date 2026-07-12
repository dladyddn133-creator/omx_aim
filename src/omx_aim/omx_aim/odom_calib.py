#!/usr/bin/env python3
"""
odom_calib.py - 궤도(트랙) 드라이브트레인 오도메트리 실측 보정 CLI

유효 바퀴 반지름(wheel radius)과 유효 좌우간격(wheel separation)을
실측으로 구하기 위한 도구.

사용 순서 (반드시 이 순서):
  1) linear  모드로 반지름 보정  -> r_new
  2) angular 모드로 좌우간격 보정 -> b_new  (r 보정 반영 후 또는 --radius-scale 사용)

주의:
  - Nav2 / teleop 을 반드시 끄고 실행할 것 (cmd_vel 충돌).
  - robot_localization(EKF) 을 켜뒀다면 끄고, 반드시 turtlebot3_node 가
    발행하는 raw /odom 을 봐야 함. (EKF 출력은 /odometry/filtered)

실행 예:
  ros2 run omx_aim odom_calib linear  --distance 3.0
  ros2 run omx_aim odom_calib angular --turns 10
  ros2 run omx_aim odom_calib monitor
"""

import argparse
import math
import sys
import threading
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

from geometry_msgs.msg import Twist, TwistStamped
from nav_msgs.msg import Odometry


# ---------------------------------------------------------------------------
# 운영 기본값 (TurtleBot3 Waffle Pi 순정값 기준)
# ---------------------------------------------------------------------------
DEFAULTS = {
    # 현재 시스템에 들어가 있는 값 (OpenCR + turtlebot3_node yaml)
    "wheel_radius": 0.033,       # [m]
    "wheel_separation": 0.287,   # [m]

    # 토픽
    "odom_topic": "/odom",
    "cmd_vel_topic": "/cmd_vel",

    # 직진 측정
    "linear_distance": 3.0,      # [m]  odom 기준 목표 거리
    "linear_speed": 0.10,        # [m/s]  느릴수록 슬립 적음
    "linear_accel": 0.05,        # [m/s^2]

    # 회전 측정
    "angular_turns": 10.0,       # [rev] odom 기준 목표 회전수
    "angular_speed": 0.5,        # [rad/s]
    "angular_accel": 0.30,       # [rad/s^2]

    # 공통
    "settle_sec": 2.0,           # 정지 후 odom 안정화 대기
    "control_hz": 50.0,
}


# ---------------------------------------------------------------------------
# 유틸
# ---------------------------------------------------------------------------
def quat_to_yaw(q) -> float:
    """쿼터니언 -> yaw [rad]"""
    siny = 2.0 * (q.w * q.z + q.x * q.y)
    cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny, cosy)


def wrap_pi(a: float) -> float:
    """[-pi, pi] 로 정규화"""
    return math.atan2(math.sin(a), math.cos(a))


# ---------------------------------------------------------------------------
# 노드
# ---------------------------------------------------------------------------
class OdomCalibNode(Node):
    """odom 누적 적산 + cmd_vel 발행."""

    def __init__(self, odom_topic: str, cmd_vel_topic: str, stamped: bool):
        super().__init__("odom_calib")

        self._lock = threading.Lock()
        self._prev = None          # (x, y, yaw)
        self._origin = None        # (x, y, yaw)

        self.path_len = 0.0        # odom 경로 길이 적산 [m]
        self.yaw_acc = 0.0         # odom yaw 누적 (unwrap, 부호 유지) [rad]
        self.last_vx = 0.0
        self.last_wz = 0.0
        self.n_odom = 0

        self.stamped = stamped
        self._cmd_type = TwistStamped if stamped else Twist

        # odom 은 보통 BEST_EFFORT / RELIABLE 둘 다 있어서 RELIABLE 로 시도
        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=20,
            durability=DurabilityPolicy.VOLATILE,
        )
        self.sub = self.create_subscription(Odometry, odom_topic, self._on_odom, qos)
        self.pub = self.create_publisher(self._cmd_type, cmd_vel_topic, 10)

        self.cmd_vel_topic = cmd_vel_topic
        self.odom_topic = odom_topic

    # -- odom 적산 --------------------------------------------------------
    def _on_odom(self, msg: Odometry):
        p = msg.pose.pose.position
        yaw = quat_to_yaw(msg.pose.pose.orientation)

        with self._lock:
            self.n_odom += 1
            self.last_vx = msg.twist.twist.linear.x
            self.last_wz = msg.twist.twist.angular.z

            if self._prev is None:
                self._prev = (p.x, p.y, yaw)
                self._origin = (p.x, p.y, yaw)
                return

            px, py, pyaw = self._prev
            self.path_len += math.hypot(p.x - px, p.y - py)
            self.yaw_acc += wrap_pi(yaw - pyaw)
            self._prev = (p.x, p.y, yaw)

    # -- 조회 -------------------------------------------------------------
    def snapshot(self):
        with self._lock:
            if self._origin is None or self._prev is None:
                return None
            ox, oy, _ = self._origin
            x, y, _ = self._prev
            return {
                "path_len": self.path_len,
                "net_disp": math.hypot(x - ox, y - oy),
                "yaw_deg": math.degrees(self.yaw_acc),
                "vx": self.last_vx,
                "wz": self.last_wz,
                "n": self.n_odom,
            }

    def reset(self):
        with self._lock:
            self._prev = None
            self._origin = None
            self.path_len = 0.0
            self.yaw_acc = 0.0

    def wait_for_odom(self, timeout: float = 10.0) -> bool:
        t0 = time.time()
        while time.time() - t0 < timeout:
            if self.snapshot() is not None:
                return True
            time.sleep(0.1)
        return False

    # -- cmd_vel ----------------------------------------------------------
    def send(self, vx: float = 0.0, wz: float = 0.0):
        if self.stamped:
            m = TwistStamped()
            m.header.stamp = self.get_clock().now().to_msg()
            m.header.frame_id = "base_link"
            m.twist.linear.x = float(vx)
            m.twist.angular.z = float(wz)
        else:
            m = Twist()
            m.linear.x = float(vx)
            m.angular.z = float(wz)
        self.pub.publish(m)

    def stop(self, repeat: int = 10):
        for _ in range(repeat):
            self.send(0.0, 0.0)
            time.sleep(0.02)

    def check_cmd_vel_conflict(self) -> int:
        """자기 자신 외에 cmd_vel 을 발행하는 노드 수."""
        return max(0, self.pub.get_subscription_count() and
                   self.count_publishers(self.cmd_vel_topic) - 1)


# ---------------------------------------------------------------------------
# 사다리꼴 프로파일 주행
# ---------------------------------------------------------------------------
def run_profile(node: OdomCalibNode, *, axis: str, target: float,
                v_max: float, accel: float, hz: float, settle: float):
    """
    axis: 'linear' | 'angular'
    target: 양수. linear=[m], angular=[rad]
    odom 적산값이 target 에 도달하도록 사다리꼴 속도 프로파일로 주행.
    """
    dt = 1.0 / hz
    v = 0.0
    t0 = time.time()
    timeout = (target / max(v_max, 1e-6)) * 3.0 + 20.0

    def measured():
        s = node.snapshot()
        if axis == "linear":
            return s["path_len"]
        return abs(math.radians(s["yaw_deg"]))

    print(f"\n[RUN] 주행 시작 — odom 목표 {target:.3f} "
          f"{'m' if axis == 'linear' else 'rad'}, v_max={v_max}, a={accel}")
    print("      (Ctrl-C 하면 즉시 정지)\n")

    try:
        while rclpy.ok():
            done = measured()
            remaining = target - done
            d_stop = (v * v) / (2.0 * accel) if accel > 0 else 0.0

            if remaining <= d_stop:
                v = max(0.0, v - accel * dt)   # 감속
            else:
                v = min(v_max, v + accel * dt)  # 가속 / 순항

            if axis == "linear":
                node.send(vx=v, wz=0.0)
            else:
                node.send(vx=0.0, wz=v)

            sys.stdout.write(
                f"\r  odom={done:8.4f}  남음={remaining:8.4f}  cmd={v:6.3f}   ")
            sys.stdout.flush()

            if v <= 1e-3 and remaining <= d_stop + 1e-3:
                break
            if time.time() - t0 > timeout:
                print("\n[WARN] 타임아웃 — 정지합니다.")
                break

            time.sleep(dt)
    except KeyboardInterrupt:
        print("\n[ABORT] 사용자 중단")
    finally:
        node.stop()

    print(f"\n[STOP] 정지 명령 — {settle:.1f}s 안정화 대기 (관성/잔여 이동 포함)")
    t1 = time.time()
    while time.time() - t1 < settle and rclpy.ok():
        node.send(0.0, 0.0)
        time.sleep(0.05)

    return node.snapshot()


# ---------------------------------------------------------------------------
# 입력 유틸
# ---------------------------------------------------------------------------
def ask_float(prompt: str, default=None):
    while True:
        suffix = f" [{default}]" if default is not None else ""
        s = input(f"{prompt}{suffix}: ").strip()
        if not s and default is not None:
            return float(default)
        try:
            return float(s)
        except ValueError:
            print("  숫자를 입력해줘.")


# ---------------------------------------------------------------------------
# 모드: linear (반지름 보정)
# ---------------------------------------------------------------------------
def mode_linear(node: OdomCalibNode, args):
    print("=" * 66)
    print(" LINEAR — 유효 바퀴 반지름(wheel radius) 보정")
    print("=" * 66)
    print(" 준비:")
    print("  1. 바닥에 로봇 기준점(예: 좌측 궤도 앞 모서리) 위치를 테이프로 표시")
    print("  2. 앞쪽으로 최소 4m 여유 공간 확보")
    print("  3. Nav2 / teleop 종료 확인")
    print()
    input(" 준비되면 Enter...")

    node.reset()
    if not node.wait_for_odom():
        print(f"[ERROR] {node.odom_topic} 수신 없음. bringup 확인.")
        return

    snap = run_profile(node, axis="linear", target=args.distance,
                       v_max=args.speed, accel=args.accel,
                       hz=DEFAULTS["control_hz"], settle=args.settle)

    print("\n" + "-" * 66)
    print(f"  odom 경로길이 : {snap['path_len']:.4f} m")
    print(f"  odom 직선변위 : {snap['net_disp']:.4f} m")
    print(f"  odom yaw 변화 : {snap['yaw_deg']:+.2f} deg  (직진이면 0 근처여야 함)")
    print("-" * 66)

    if abs(snap["yaw_deg"]) > 5.0:
        print("  [WARN] yaw 가 5도 이상 틀어졌어. 궤도 좌우 장력이 다르거나")
        print("         한쪽이 미끄러지는 중일 수 있음. 다시 재보는 걸 추천.")

    d_odom = snap["path_len"]
    print("\n>>> 이제 줄자로 '실제 이동 거리'를 재서 입력해줘.")
    d_real = ask_float("  실제 이동 거리 [m]")

    if d_odom < 1e-6:
        print("[ERROR] odom 이동량이 0이야.")
        return

    ratio = d_real / d_odom
    r_new = args.radius * ratio

    print("\n" + "=" * 66)
    print(" 결과")
    print("=" * 66)
    print(f"  실제 / odom          = {d_real:.4f} / {d_odom:.4f} = {ratio:.4f}")
    print(f"  현재 wheel_radius    = {args.radius:.5f} m")
    print(f"  >>> 새 wheel_radius  = {r_new:.5f} m   (지름 {r_new*2*100:.2f} cm)")
    print()
    print("  [적용할 곳 — 두 군데 다 바꿔야 함]")
    print(f"   1) OpenCR 펌웨어: turtlebot3_waffle.h  WHEEL_RADIUS = {r_new:.5f}")
    print(f"   2) turtlebot3_node: param/waffle_pi.yaml  wheels.radius: {r_new:.5f}")
    print()
    print(f"  다음 단계 (회전 보정)에서 --radius-scale {ratio:.4f} 를 쓰면")
    print("  펌웨어 반영 전에도 좌우간격을 바로 계산할 수 있어.")
    print("=" * 66)


# ---------------------------------------------------------------------------
# 모드: angular (좌우간격 보정)
# ---------------------------------------------------------------------------
def mode_angular(node: OdomCalibNode, args):
    print("=" * 66)
    print(" ANGULAR — 유효 좌우간격(wheel separation) 보정")
    print("=" * 66)
    print(" 준비:")
    print("  1. 로봇 정면 방향을 바닥 테이프 선에 정확히 맞춤")
    print("  2. 로봇 위에도 정면 방향 마커(테이프) 부착 -> 회전수 세기 편함")
    print("  3. 제자리 회전 가능한 평평한 바닥")
    print()
    if abs(args.radius_scale - 1.0) < 1e-9:
        print("  [주의] --radius-scale 이 1.0 이야.")
        print("         반지름 보정을 '이미 펌웨어+yaml 에 반영했다'는 뜻이야.")
        print("         아직 반영 안 했으면 Ctrl-C 하고 linear 결과의 비율을 넣어줘.")
    else:
        print(f"  [정보] radius_scale = {args.radius_scale:.4f} 로 보정식에 반영함")
        print("         (반지름을 아직 시스템에 안 넣은 상태로 측정 중)")
    print()
    input(" 준비되면 Enter...")

    node.reset()
    if not node.wait_for_odom():
        print(f"[ERROR] {node.odom_topic} 수신 없음.")
        return

    target_rad = args.turns * 2.0 * math.pi
    snap = run_profile(node, axis="angular", target=target_rad,
                       v_max=args.omega, accel=args.alpha,
                       hz=DEFAULTS["control_hz"], settle=args.settle)

    odom_deg = abs(snap["yaw_deg"])

    print("\n" + "-" * 66)
    print(f"  odom 누적 회전 : {odom_deg:.2f} deg  ({odom_deg/360.0:.4f} 바퀴)")
    print(f"  odom 위치 이동 : {snap['net_disp']:.4f} m  (제자리면 작아야 함)")
    print("-" * 66)

    if snap["net_disp"] > 0.15:
        print("  [WARN] 제자리 회전인데 15cm 이상 밀렸어. 회전 중심이 안 맞는 중.")

    print("\n>>> 로봇이 '실제로' 몇 도 돌았는지 입력해줘.")
    print("    (완전한 바퀴 수 + 남은 각도. 예: 7바퀴 + 130도)")
    turns_real = ask_float("  실제 완전 회전 수 [rev]", 0)
    resid_real = ask_float("  실제 남은 각도 [deg]", 0)
    actual_deg = abs(turns_real * 360.0 + resid_real)

    if actual_deg < 1e-6:
        print("[ERROR] 실제 회전량이 0이야.")
        return

    ratio = odom_deg / actual_deg
    b_new = args.separation * args.radius_scale * ratio

    print("\n" + "=" * 66)
    print(" 결과")
    print("=" * 66)
    print(f"  odom / 실제              = {odom_deg:.2f} / {actual_deg:.2f} = {ratio:.4f}")
    if abs(args.radius_scale - 1.0) > 1e-9:
        print(f"  radius_scale 보정        = x {args.radius_scale:.4f}")
    print(f"  현재 wheel_separation    = {args.separation:.5f} m")
    print(f"  >>> 새 wheel_separation  = {b_new:.5f} m")
    print()
    if ratio > 1.0:
        print("  해석: odom 이 실제보다 '더 돌았다'고 과대보고 중.")
        print("        -> 궤도 슬립으로 유효 좌우간격이 기하학적 값보다 넓게 동작.")
    else:
        print("  해석: odom 이 실제보다 '덜 돌았다'고 과소보고 중.")
    print()
    print("  [적용할 곳 — 두 군데 다]")
    print(f"   1) OpenCR: turtlebot3_waffle.h  WHEEL_SEPARATION = {b_new:.5f}")
    print(f"   2) turtlebot3_node: param/waffle_pi.yaml  wheels.separation: {b_new:.5f}")
    print()
    print("  다음: 이 값은 '이 회전 속도'에서만 정확해. 스키드스티어는 선회반경마다")
    print("        슬립량이 달라지니까, robot_localization EKF 로 IMU yaw 융합하는 게")
    print("        근본 해법이야. 이 보정은 EKF 의 출발점.")
    print("=" * 66)


# ---------------------------------------------------------------------------
# 모드: monitor (수동 주행 — 가장 정확)
# ---------------------------------------------------------------------------
def mode_monitor(node: OdomCalibNode, args):
    print("=" * 66)
    print(" MONITOR — 수동 주행 측정 (cmd_vel 발행 안 함)")
    print("=" * 66)
    print(" 이 모드가 제일 정확해. 실제값을 '자로 재는' 게 아니라 '고정'시키니까.")
    print()
    print(" [직진]  바닥에 정확히 3.000 m 떨어진 두 선을 긋고,")
    print("         teleop 으로 첫 선 -> 둘째 선까지 주행. 실제값 = 3.000 (오차 0)")
    print()
    print(" [회전]  로봇 정면을 테이프 선에 맞춘 뒤, teleop 으로 제자리 회전하며")
    print("         눈으로 세서 정확히 10바퀴 후 같은 선에 재정렬. 실제값 = 3600.0")
    print()
    print(" 다른 터미널에서:")
    print("   ros2 run turtlebot3_teleop teleop_keyboard")
    print()
    print(" Ctrl-C 로 측정 종료 -> 계산 프롬프트.")
    print("=" * 66)
    input("\n 준비되면 Enter (Enter 누른 시점이 0점)...")

    node.reset()
    if not node.wait_for_odom():
        print(f"[ERROR] {node.odom_topic} 수신 없음.")
        return

    print()
    try:
        while rclpy.ok():
            s = node.snapshot()
            sys.stdout.write(
                f"\r  경로={s['path_len']:8.4f} m | 변위={s['net_disp']:8.4f} m | "
                f"yaw={s['yaw_deg']:+9.2f} deg ({s['yaw_deg']/360.0:+6.3f} rev) | "
                f"vx={s['vx']:+5.2f} wz={s['wz']:+5.2f}  ")
            sys.stdout.flush()
            time.sleep(0.1)
    except KeyboardInterrupt:
        pass

    s = node.snapshot()
    print("\n\n" + "-" * 66)
    print(f"  odom 경로길이  : {s['path_len']:.4f} m")
    print(f"  odom 직선변위  : {s['net_disp']:.4f} m")
    print(f"  odom 누적 yaw  : {s['yaw_deg']:+.2f} deg  ({s['yaw_deg']/360.0:+.4f} rev)")
    print("-" * 66)

    print("\n 어떤 보정을 계산할까?")
    print("   1) 반지름 (직진 측정)")
    print("   2) 좌우간격 (회전 측정)")
    print("   0) 계산 안 함")
    choice = input(" 선택 [0]: ").strip() or "0"

    if choice == "1":
        d_real = ask_float("  실제 이동 거리 [m]", 3.0)
        d_odom = s["path_len"]
        ratio = d_real / d_odom
        r_new = args.radius * ratio
        print(f"\n  비율 = {ratio:.4f}")
        print(f"  >>> 새 wheel_radius = {r_new:.5f} m  (지름 {r_new*2*100:.2f} cm)")
        print(f"  >>> 회전 측정 시 --radius-scale {ratio:.4f}")

    elif choice == "2":
        turns_real = ask_float("  실제 회전 수 [rev]", 10.0)
        actual_deg = abs(turns_real * 360.0)
        odom_deg = abs(s["yaw_deg"])
        ratio = odom_deg / actual_deg
        b_new = args.separation * args.radius_scale * ratio
        print(f"\n  odom/실제 = {odom_deg:.2f}/{actual_deg:.2f} = {ratio:.4f}")
        if abs(args.radius_scale - 1.0) > 1e-9:
            print(f"  radius_scale 보정 = x {args.radius_scale:.4f}")
        print(f"  >>> 새 wheel_separation = {b_new:.5f} m")


# ---------------------------------------------------------------------------
# main
# ---------------------------------------------------------------------------
def build_parser():
    p = argparse.ArgumentParser(
        prog="odom_calib",
        description="궤도 드라이브트레인 오도메트리 실측 보정",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    p.add_argument("--radius", type=float, default=DEFAULTS["wheel_radius"],
                   help="현재 시스템에 들어있는 wheel_radius [m]")
    p.add_argument("--separation", type=float, default=DEFAULTS["wheel_separation"],
                   help="현재 시스템에 들어있는 wheel_separation [m]")
    p.add_argument("--radius-scale", type=float, default=1.0,
                   help="반지름 보정 배율(r_new/r_old). 아직 시스템에 반영 안 했을 때 사용")
    p.add_argument("--odom-topic", default=DEFAULTS["odom_topic"])
    p.add_argument("--cmd-vel-topic", default=DEFAULTS["cmd_vel_topic"])
    p.add_argument("--stamped", action="store_true",
                   help="cmd_vel 을 TwistStamped 로 발행")
    p.add_argument("--settle", type=float, default=DEFAULTS["settle_sec"])

    sub = p.add_subparsers(dest="mode", required=True)

    pl = sub.add_parser("linear", help="직진 -> 반지름 보정")
    pl.add_argument("--distance", type=float, default=DEFAULTS["linear_distance"])
    pl.add_argument("--speed", type=float, default=DEFAULTS["linear_speed"])
    pl.add_argument("--accel", type=float, default=DEFAULTS["linear_accel"])

    pa = sub.add_parser("angular", help="제자리 회전 -> 좌우간격 보정")
    pa.add_argument("--turns", type=float, default=DEFAULTS["angular_turns"])
    pa.add_argument("--omega", type=float, default=DEFAULTS["angular_speed"])
    pa.add_argument("--alpha", type=float, default=DEFAULTS["angular_accel"])

    sub.add_parser("monitor", help="수동 주행 관측 (cmd_vel 발행 안 함, 가장 정확)")

    return p


def main(argv=None):
    args = build_parser().parse_args(argv if argv is not None else sys.argv[1:])

    rclpy.init()
    node = OdomCalibNode(args.odom_topic, args.cmd_vel_topic, args.stamped)

    spin_thread = threading.Thread(
        target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()
    time.sleep(0.5)

    # cmd_vel 충돌 사전 점검
    if args.mode in ("linear", "angular"):
        n_pub = node.count_publishers(args.cmd_vel_topic)
        if n_pub > 1:
            print(f"\n[WARN] {args.cmd_vel_topic} 에 다른 발행자가 "
                  f"{n_pub - 1}개 있어 (Nav2? teleop?).")
            print("       충돌하면 측정이 전부 망가져. 먼저 끄고 오는 걸 강력 추천.")
            if input("       그래도 진행? [y/N]: ").strip().lower() != "y":
                node.stop()
                node.destroy_node()
                rclpy.shutdown()
                return

    try:
        if args.mode == "linear":
            mode_linear(node, args)
        elif args.mode == "angular":
            mode_angular(node, args)
        elif args.mode == "monitor":
            mode_monitor(node, args)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()