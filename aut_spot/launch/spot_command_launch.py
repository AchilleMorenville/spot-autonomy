from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    return LaunchDescription([
        Node(
            package='aut_spot',
            executable='spot_command',
            name='spot_command'
        )
    ])