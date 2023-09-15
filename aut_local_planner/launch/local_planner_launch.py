from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    return LaunchDescription([
        Node(
            package='aut_local_planner',
            executable='local_planner',
            name='local_planner'
        )
    ])