import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    velodyne_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('aut_launch'), 'launch'),
            '/velodyne_launch.py'])
        )
    spot_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('aut_spot'), 'launch'),
            '/spot_launch.py'])
        )

    return LaunchDescription([
        velodyne_launch,
        spot_launch
    ])