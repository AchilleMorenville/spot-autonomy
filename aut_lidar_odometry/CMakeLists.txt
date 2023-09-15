cmake_minimum_required(VERSION 3.8)
project(aut_lidar_odometry)

# if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
#   add_compile_options(-Wall -Wextra -Wpedantic -O3)
# endif()

add_compile_options(-Wall -Wextra -Wpedantic -Ofast)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(PCL REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(Eigen REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(OpenCV REQUIRED)
find_package(aut_msgs REQUIRED)
find_package(aut_utils REQUIRED)

set(msg_files
  "msg/Features.msg"
)

rosidl_generate_interfaces(${PROJECT_NAME} ${msg_files} DEPENDENCIES std_msgs sensor_msgs geometry_msgs)

rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")

include_directories(
  include
  ${aut_utils_INCLUDE_DIRS}
)

add_executable(feature_extraction
  src/feature_extraction.cc
)
ament_target_dependencies(feature_extraction rclcpp sensor_msgs nav_msgs tf2_ros PCL pcl_conversions Eigen std_msgs)
target_link_libraries(feature_extraction ${PCL_LIBRARIES} "${cpp_typesupport_target}")

add_executable(lidar_odometry
  src/lidar_odometry.cc src/optimization.cc
)
ament_target_dependencies(lidar_odometry rclcpp PCL pcl_conversions Eigen std_msgs geometry_msgs OpenCV aut_msgs tf2_ros aut_utils)
target_link_libraries(lidar_odometry ${PCL_LIBRARIES} ${OpenCV_LIBRARIES} "${cpp_typesupport_target}" ${aut_utils_LIBRARIES})


install(TARGETS lidar_odometry feature_extraction
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include/
)

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

install(DIRECTORY
  params
  DESTINATION share/${PROJECT_NAME}/
)

ament_export_include_directories(include)
ament_export_dependencies(rclcpp sensor_msgs nav_msgs tf2_ros PCL pcl_conversions Eigen std_msgs OpenCV aut_msgs aut_utils)

ament_package()
