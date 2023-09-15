from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='aut_global_planner',
            executable='global_planner',
            name='global_planner'
        )
    ])