from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz',
            arguments=['-d', '/path/to/your/ros2_ws/src/altosradar/altosradar.rviz'],  # Update the path if needed
            output='screen'
        )
    ])
