"""
restricted_zone_detector.launch.py

통제 구역 감지 노드 실행. 사용:
  ros2 launch felony_stop_sim restricted_zone_detector.launch.py
"""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='felony_stop_sim',
            executable='restricted_zone_detector',
            name='restricted_zone_detector',
            output='screen',
            parameters=[{
                'use_sim_time': True,
            }],
        ),
    ])