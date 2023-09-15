import os
import yaml

import ament_index_python.packages
import launch
import launch_ros.actions


def generate_launch_description():

    dir_path = os.path.dirname(os.path.realpath(__file__))

    with open(f"{dir_path}/../params/velodyne_params.yaml", 'r') as f:
        transform_params = yaml.safe_load(f)['velodyne_transform_node']['ros__parameters']
    with open(f"{dir_path}/../params/velodyne_params.yaml", 'r') as f:
        driver_params = yaml.safe_load(f)['velodyne_driver_node']['ros__parameters']

    driver_share_dir = ament_index_python.packages.get_package_share_directory('velodyne_driver')
    # driver_params_file = os.path.join(driver_share_dir, 'config', 'VLP16-velodyne_driver_node-params.yaml')
    velodyne_driver_node = launch_ros.actions.Node(package='velodyne_driver',
                                                   executable='velodyne_driver_node',
                                                   output='both',
                                                   parameters=[driver_params])

    transform_share_dir = ament_index_python.packages.get_package_share_directory('velodyne_pointcloud')
    # transform_params_file = os.path.join(transform_share_dir, 'config', 'VLP16-velodyne_transform_node-params.yaml')
    transform_params['calibration'] = os.path.join(transform_share_dir, 'params', 'VLP16db.yaml')
    velodyne_transform_node = launch_ros.actions.Node(package='velodyne_pointcloud',
                                                    executable='velodyne_transform_node',
                                                    output='both',
                                                    parameters=[transform_params])

    laserscan_share_dir = ament_index_python.packages.get_package_share_directory('velodyne_laserscan')
    laserscan_params_file = os.path.join(laserscan_share_dir, 'config', 'default-velodyne_laserscan_node-params.yaml')
    velodyne_laserscan_node = launch_ros.actions.Node(package='velodyne_laserscan',
                                                      executable='velodyne_laserscan_node',
                                                      output='both',
                                                      parameters=[laserscan_params_file])


    return launch.LaunchDescription([velodyne_driver_node,
                                     velodyne_transform_node,
                                     velodyne_laserscan_node,

                                     launch.actions.RegisterEventHandler(
                                         event_handler=launch.event_handlers.OnProcessExit(
                                             target_action=velodyne_driver_node,
                                             on_exit=[launch.actions.EmitEvent(
                                                 event=launch.events.Shutdown())],
                                         )),
                                     ])