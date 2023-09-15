import os
import yaml

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    
    config_lidar_odometry = os.path.join(
        get_package_share_directory('aut_lidar_odometry'),
        'params',
        'lidar_odometry_params.yaml'
    )

    return LaunchDescription([
        Node(
            package='aut_lidar_odometry',
            executable='lidar_odometry',
            name='lidar_odometry',
            parameters=[
                config_lidar_odometry
            ]
        ),
        Node(
            package='aut_lidar_odometry',
            executable='feature_extraction',
            name='feature_extraction',
            parameters=[
                config_lidar_odometry
            ]
        )
    ])