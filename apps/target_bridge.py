#!/usr/bin/env python3
"""Target Bridge - 표적 좌표를 와플 기준으로 변환.

역할:
    1. /target_in_map 또는 /clicked_point 으로 표적 좌표 수신
    2. TF 조회: map -> base_link (와플 위치/방향)
    3. arm_base 오프셋 적용
    4. /omx/target_coord 로 OMX 노드에 전송

입력 토픽:
    /target_in_map  (geometry_msgs/PointStamped) - 외부 코드/명령
    /clicked_point  (geometry_msgs/PointStamped) - RViz 'Publish Point' 도구

출력 토픽:
    /omx/target_coord  (geometry_msgs/Point)
    /bridge/status     (std_msgs/String)

RViz 사용법:
    1. 상단 툴바 'Publish Point' 선택 (또는 P 키)
    2. 맵 위 클릭
    3. /clicked_point 발행 -> bridge 수신
    
    참고: RViz Publish Point 의 기본 z = 0.0 (지면)
          표적 높이 자동 설정: default_target_z 파라미터

ROS 파라미터:
    arm_base_x, arm_base_y, arm_base_z (m): OMX 베이스 오프셋
    map_frame, robot_frame: TF frame 이름
    tf_timeout_sec: TF lookup timeout
    default_target_z (m): RViz 클릭 시 z 값 override (0이면 받은 값 유지)

실행:
    omxenv
    python3 apps/target_bridge.py
    
    # RViz 클릭 시 표적 높이를 자동으로 30cm 로
    python3 apps/target_bridge.py --ros-args -p default_target_z:=0.3
"""

from __future__ import annotations

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

import math
from typing import Optional

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration

from geometry_msgs.msg import Point, PointStamped
from std_msgs.msg import String

from tf2_ros import Buffer, TransformListener, TransformException

try:
    from tf2_geometry_msgs import do_transform_point
except ImportError as e:
    print()
    print("=" * 60)
    print("ERROR: tf2_geometry_msgs 패키지가 없습니다.")
    print("  sudo apt install ros-jazzy-tf2-geometry-msgs")
    print(f"세부: {e}")
    print("=" * 60)
    sys.exit(1)


def quat_to_yaw(qx, qy, qz, qw):
    siny_cosp = 2.0 * (qw * qz + qx * qy)
    cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
    return math.atan2(siny_cosp, cosy_cosp)


def compute_aim_angles(x, y, z):
    dist = math.sqrt(x*x + y*y + z*z)
    yaw = math.atan2(y, x)
    pitch = math.atan2(z, math.hypot(x, y))
    return yaw, pitch, dist


class TargetBridge(Node):

    def __init__(self):
        super().__init__('target_bridge')

        # ----- 파라미터 -----
        self.declare_parameter('arm_base_x', 0.10)
        self.declare_parameter('arm_base_y', 0.00)
        self.declare_parameter('arm_base_z', 0.18)
        self.declare_parameter('tf_timeout_sec', 0.1)
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('robot_frame', 'base_link')
        self.declare_parameter('status_publish_hz', 1.0)
        self.declare_parameter('default_target_z', 0.0)

        self.arm_offset = (
            self.get_parameter('arm_base_x').value,
            self.get_parameter('arm_base_y').value,
            self.get_parameter('arm_base_z').value,
        )
        self.tf_timeout = Duration(
            seconds=self.get_parameter('tf_timeout_sec').value)
        self.map_frame = self.get_parameter('map_frame').value
        self.robot_frame = self.get_parameter('robot_frame').value
        self.default_target_z = self.get_parameter('default_target_z').value
        status_hz = self.get_parameter('status_publish_hz').value

        self.get_logger().info("=" * 50)
        self.get_logger().info("Target Bridge")
        self.get_logger().info("=" * 50)
        self.get_logger().info(
            f"Arm base offset: x={self.arm_offset[0]:+.3f}, "
            f"y={self.arm_offset[1]:+.3f}, z={self.arm_offset[2]:+.3f} m")
        self.get_logger().info(
            f"TF frames: {self.map_frame} -> {self.robot_frame}")
        if self.default_target_z != 0.0:
            self.get_logger().info(
                f"RViz click z override: {self.default_target_z:+.3f} m")

        # ----- TF -----
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # ----- Subscribers -----
        self.create_subscription(
            PointStamped, '/target_in_map',
            self.on_target_in_map, 10)
        
        # RViz 'Publish Point' 도구
        self.create_subscription(
            PointStamped, '/clicked_point',
            self.on_clicked_point, 10)

        # ----- Publishers -----
        self.pub_target_coord = self.create_publisher(
            Point, '/omx/target_coord', 10)
        self.pub_status = self.create_publisher(
            String, '/bridge/status', 10)

        if status_hz > 0:
            self.create_timer(1.0 / status_hz, self.publish_waffle_pose_debug)

        self.tf_ready = False
        self.target_count = 0
        self.success_count = 0

        self.get_logger().info("입력 (Subscribe):")
        self.get_logger().info("  /target_in_map  geometry_msgs/PointStamped")
        self.get_logger().info("  /clicked_point  geometry_msgs/PointStamped  (RViz)")
        self.get_logger().info("출력 (Publish):")
        self.get_logger().info("  /omx/target_coord  geometry_msgs/Point")
        self.get_logger().info("  /bridge/status     std_msgs/String")
        self.get_logger().info("=" * 50)
        self.get_logger().info("=== Bridge ready ===")

    # -------------------------------------------------------
    # 좌표 변환
    # -------------------------------------------------------

    def transform_target_to_arm_base(self, target_in_map):
        try:
            transform = self.tf_buffer.lookup_transform(
                target_frame=self.robot_frame,
                source_frame=self.map_frame,
                time=rclpy.time.Time(),
                timeout=self.tf_timeout,
            )
        except TransformException as e:
            self.get_logger().warn(
                f"TF lookup 실패: {e}")
            return None

        try:
            target_in_base = do_transform_point(target_in_map, transform)
        except Exception as e:
            self.get_logger().warn(f"do_transform_point 실패: {e}")
            return None

        result = Point()
        result.x = target_in_base.point.x - self.arm_offset[0]
        result.y = target_in_base.point.y - self.arm_offset[1]
        result.z = target_in_base.point.z - self.arm_offset[2]
        return result

    # -------------------------------------------------------
    # 콜백
    # -------------------------------------------------------

    def on_target_in_map(self, msg, source="topic"):
        """표적 좌표 수신 (메인 콜백)."""
        self.target_count += 1
        
        recv_frame = msg.header.frame_id
        if recv_frame and recv_frame != self.map_frame:
            self.get_logger().warn(
                f"표적 frame='{recv_frame}', '{self.map_frame}' 으로 가정")
        
        if not msg.header.frame_id:
            msg.header.frame_id = self.map_frame

        self.get_logger().info(
            f"[#{self.target_count}] 표적 수신 ({source}, map): "
            f"({msg.point.x:+.3f}, {msg.point.y:+.3f}, {msg.point.z:+.3f}) m")

        target_arm = self.transform_target_to_arm_base(msg)
        if target_arm is None:
            self._publish_status(f"[#{self.target_count}] 변환 실패")
            return

        self.success_count += 1
        yaw, pitch, dist = compute_aim_angles(
            target_arm.x, target_arm.y, target_arm.z)

        self.get_logger().info(
            f"[#{self.target_count}] 표적 변환 (arm_base): "
            f"({target_arm.x:+.3f}, {target_arm.y:+.3f}, {target_arm.z:+.3f}) m")
        self.get_logger().info(
            f"           거리 {dist:.3f}m, "
            f"yaw {math.degrees(yaw):+.1f}°, pitch {math.degrees(pitch):+.1f}°")

        self.pub_target_coord.publish(target_arm)
        self._publish_status(
            f"[#{self.target_count}] 전송 ({source}): arm_base "
            f"({target_arm.x:+.2f}, {target_arm.y:+.2f}, {target_arm.z:+.2f})")

    def on_clicked_point(self, msg):
        """RViz 'Publish Point' 도구 출력 처리.
        
        RViz 는 기본 z=0. default_target_z 파라미터로 override 가능.
        """
        if self.default_target_z != 0.0:
            self.get_logger().info(
                f"RViz click z={msg.point.z:.3f} -> override "
                f"{self.default_target_z:.3f}")
            msg.point.z = self.default_target_z
        
        self.on_target_in_map(msg, source="rviz")

    def publish_waffle_pose_debug(self):
        try:
            transform = self.tf_buffer.lookup_transform(
                target_frame=self.map_frame,
                source_frame=self.robot_frame,
                time=rclpy.time.Time(),
                timeout=Duration(seconds=0.05),
            )
        except TransformException:
            if not self.tf_ready:
                self._publish_status(
                    f"TF 대기 중 ({self.map_frame}->{self.robot_frame})")
            return

        if not self.tf_ready:
            self.tf_ready = True
            self.get_logger().info(
                f"TF 연결 성공: {self.map_frame} -> {self.robot_frame}")

        wx = transform.transform.translation.x
        wy = transform.transform.translation.y
        wz = transform.transform.translation.z
        qx = transform.transform.rotation.x
        qy = transform.transform.rotation.y
        qz = transform.transform.rotation.z
        qw = transform.transform.rotation.w
        yaw_deg = math.degrees(quat_to_yaw(qx, qy, qz, qw))

        self._publish_status(
            f"와플 (map): x={wx:+.2f}, y={wy:+.2f}, z={wz:+.2f}, "
            f"yaw={yaw_deg:+.1f}°  "
            f"[targets: {self.success_count}/{self.target_count}]")

    def _publish_status(self, text):
        msg = String()
        msg.data = text
        self.pub_status.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    
    node = None
    try:
        node = TargetBridge()
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("\n중단됨.")
    except Exception as e:
        print(f"노드 에러: {type(e).__name__}: {e}")
        import traceback
        traceback.print_exc()
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()