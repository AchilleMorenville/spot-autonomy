cmake_minimum_required(VERSION 3.8)
project(aut_slam)

add_compile_options(-Wall -Wextra -Wpedantic -Ofast)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(aut_msgs REQUIRED)
find_package(PCL REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(Eigen REQUIRED)
find_package(std_msgs REQUIRED)
find_package(aut_utils REQUIRED)
find_package(aut_common REQUIRED)
find_package(GTSAM REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(dynamicEDT3D REQUIRED)
find_package(octomap REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(std_srvs REQUIRED)

include_directories(
  include
)

include_directories(${OCTOMAP_INCLUDE_DIRS})
include_directories(${DYNAMICEDT3D_INCLUDE_DIRS})

add_library(graph_optimization SHARED
  src/graph_optimization.cc
)
ament_target_dependencies(graph_optimization rclcpp aut_msgs PCL pcl_conversions Eigen std_msgs aut_utils GTSAM tf2_ros octomap dynamicEDT3D aut_common visualization_msgs std_srvs)
target_link_libraries(graph_optimization ${PCL_LIBRARIES} gtsam ${OCTOMAP_LIBRARIES} ${DYNAMICEDT3D_LIBRARIES})

add_executable(slam
  src/main.cc
)
ament_target_dependencies(slam rclcpp aut_msgs PCL pcl_conversions Eigen std_msgs aut_utils GTSAM tf2_ros octomap dynamicEDT3D aut_common visualization_msgs std_srvs)
target_link_libraries(slam graph_optimization ${PCL_LIBRARIES} gtsam ${OCTOMAP_LIBRARIES} ${DYNAMICEDT3D_LIBRARIES})

install(TARGETS graph_optimization
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(TARGETS slam
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
ament_export_libraries(graph_optimization)
ament_export_dependencies(rclcpp aut_msgs PCL pcl_conversions Eigen std_msgs aut_utils GTSAM tf2_ros octomap dynamicEDT3D aut_common visualization_msgs std_srvs)

ament_package()
