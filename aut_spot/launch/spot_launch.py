import os
import yaml

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    
    config_spot = os.path.join(
        get_package_share_directory('aut_spot'),
        'params',
        'spot_params.yaml'
    )

    return LaunchDescription([
        Node(
            package='aut_spot',
            executable='spot_data',
            name='spot_data',
            parameters=[
                config_spot
            ]
        )
    ])