"""Jetson(Waffle) 측 노드 일괄 실행: waffle_node + yolo_node + fire_node + target_bridge + scan_processor.

turtlebot3_bringup 은 별도 실행 (README.md 참고).
yolo_node 의 --debug-stream 등 세부 플래그가 필요하면
`ros2 run omx_aim yolo_node --debug-stream` 로 개별 실행하세요.

사용:
    ros2 launch omx_aim jetson.launch.py
"""
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='omx_aim', executable='waffle_node', name='waffle_node',
            output='screen',
        ),
        Node(
            package='omx_aim', executable='yolo_node', name='omx_yolo_node',
            output='screen',
            arguments=['--no-display'],
        ),
        Node(
            package='omx_aim', executable='fire_node', name='fire_node',
            output='screen',
        ),
        Node(
            package='omx_aim', executable='target_bridge', name='target_bridge',
            output='screen',
        ),
        Node(
            package='omx_aim', executable='scan_processor', name='scan_processor',
            output='screen',
        ),
    ])
