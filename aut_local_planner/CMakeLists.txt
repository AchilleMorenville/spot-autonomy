cmake_minimum_required(VERSION 3.8)
project(aut_local_planner)

add_compile_options(-Wall -Wextra -Wpedantic -Ofast)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(aut_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(aut_utils REQUIRED)
find_package(aut_common REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(std_srvs REQUIRED)


include_directories(
  include
  ${EIGEN3_INCLUDE_DIR}
)

add_library(local_planner_impl SHARED
  src/local_planner.cc src/a_star.cc
)
ament_target_dependencies(local_planner_impl rclcpp tf2_ros aut_msgs Eigen3 aut_utils nav_msgs geometry_msgs visualization_msgs aut_common std_srvs)
target_link_libraries(local_planner_impl ${Eigen3_LIBRARIES})
 
add_executable(local_planner
  src/main.cc
)
ament_target_dependencies(local_planner rclcpp tf2_ros aut_msgs Eigen3 aut_utils nav_msgs geometry_msgs visualization_msgs aut_common std_srvs)
target_link_libraries(local_planner local_planner_impl ${Eigen3_LIBRARIES})

add_executable(a_star
  src/a_star.cc
)
ament_target_dependencies(a_star Eigen3 aut_utils)
target_link_libraries(a_star local_planner_impl ${Eigen3_LIBRARIES})


install(TARGETS local_planner_impl
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(TARGETS local_planner a_star
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include/
)

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

ament_export_include_directories(include)
ament_export_libraries(local_planner_impl)
ament_export_dependencies(rclcpp tf2_ros aut_msgs aut_utils nav_msgs geometry_msgs visualization_msgs aut_common std_srvs)

ament_package()
