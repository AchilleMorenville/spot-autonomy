import os
import yaml

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    
    config_localization = os.path.join(
        get_package_share_directory('aut_localization'),
        'params',
        'localization_params.yaml'
    )

    return LaunchDescription([
        Node(
            package='aut_localization',
            executable='localization',
            name='localization',
            parameters=[
                config_localization
            ]
        )
    ])