cmake_minimum_required(VERSION 3.8)
project(aut_global_planner)

add_compile_options(-Wall -Wextra -Wpedantic -Ofast)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(aut_common REQUIRED)
find_package(aut_utils REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(aut_msgs REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(std_srvs REQUIRED)


include_directories(
  include
  ${EIGEN3_INCLUDE_DIR}
)

add_library(global_planner_impl SHARED
  src/global_planner.cc
)
ament_target_dependencies(global_planner_impl rclcpp aut_common aut_utils Eigen3 visualization_msgs tf2_ros aut_msgs rclcpp_action std_srvs)
target_link_libraries(global_planner_impl ${Eigen3_LIBRARIES})
 
add_executable(global_planner
  src/main.cc
)
ament_target_dependencies(global_planner rclcpp aut_common aut_utils Eigen3 visualization_msgs tf2_ros aut_msgs rclcpp_action std_srvs)
target_link_libraries(global_planner global_planner_impl ${Eigen3_LIBRARIES})

install(TARGETS global_planner_impl
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(TARGETS global_planner
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include/
)

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

ament_export_include_directories(include)
ament_export_libraries(global_planner_impl)
ament_export_dependencies(rclcpp aut_common aut_utils Eigen3 visualization_msgs tf2_ros aut_msgs rclcpp_action std_srvs)

ament_package()
