from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='ir_led_measure',
            executable='camera_publisher.py',
            name='camera_publisher',
            output='screen',
        ),
        Node(
            package='ir_led_measure',
            executable='fusion.py',
            name='fusion',
            output='screen',
        ),
    ])
