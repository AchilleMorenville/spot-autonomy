cmake_minimum_required(VERSION 3.8)
project(aut_msgs)

find_package(ament_cmake REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

set(msg_files
  "msg/PointCloudWithPose.msg"
  "msg/LocalGrid.msg"
  "msg/Fiducial.msg"
  "msg/SpeedCommand.msg"
)

set(srv_files
  "srv/SaveMap.srv"
  "srv/LoadMap.srv"
  "srv/SetGlobalPath.srv"
  "srv/GraphModif.srv"
)

set(action_files
  "action/NavigateToGoal.action"
)

rosidl_generate_interfaces(${PROJECT_NAME} ${msg_files} ${srv_files} ${action_files} DEPENDENCIES std_msgs sensor_msgs geometry_msgs nav_msgs)

ament_export_dependencies(rosidl_default_runtime)

ament_package()
