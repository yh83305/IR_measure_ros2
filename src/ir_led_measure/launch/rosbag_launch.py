from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='ir_led_measure',
            executable='camera_rosbag.py',
            name='camera_rosbag',
            output='screen',
        ),
    ])
