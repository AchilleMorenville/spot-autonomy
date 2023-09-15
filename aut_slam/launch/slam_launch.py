import os
import yaml

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    
    config_slam = os.path.join(
        get_package_share_directory('aut_slam'),
        'params',
        'slam_params.yaml'
    )

    return LaunchDescription([
        Node(
            package='aut_slam',
            executable='slam',
            name='slam',
            parameters=[
                config_slam
            ]
        )
    ])