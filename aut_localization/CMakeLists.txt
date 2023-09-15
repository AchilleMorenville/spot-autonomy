cmake_minimum_required(VERSION 3.8)
project(aut_localization)

add_compile_options(-Wall -Wextra -Wpedantic -Ofast)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(aut_msgs REQUIRED)
find_package(PCL REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(Eigen REQUIRED)
find_package(aut_utils REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(dynamicEDT3D REQUIRED)
find_package(octomap REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_srvs REQUIRED)

include_directories(
  include
)

include_directories(${OCTOMAP_INCLUDE_DIRS})
include_directories(${DYNAMICEDT3D_INCLUDE_DIRS})

add_library(monte_carlo_localization SHARED
  src/monte_carlo_localization.cc
)
ament_target_dependencies(monte_carlo_localization rclcpp aut_msgs PCL pcl_conversions Eigen aut_utils tf2_ros octomap dynamicEDT3D geometry_msgs std_srvs)
target_link_libraries(monte_carlo_localization ${PCL_LIBRARIES} ${OCTOMAP_LIBRARIES} ${DYNAMICEDT3D_LIBRARIES})

add_executable(localization
  src/main.cc
)
ament_target_dependencies(localization rclcpp aut_msgs PCL pcl_conversions Eigen aut_utils tf2_ros octomap dynamicEDT3D geometry_msgs std_srvs)
target_link_libraries(localization monte_carlo_localization ${PCL_LIBRARIES} ${OCTOMAP_LIBRARIES} ${DYNAMICEDT3D_LIBRARIES})

install(TARGETS monte_carlo_localization
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(TARGETS localization
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
ament_export_libraries(monte_carlo_localization)
ament_export_dependencies(rclcpp aut_msgs PCL pcl_conversions Eigen aut_utils tf2_ros octomap dynamicEDT3D geometry_msgs std_srvs)

ament_package()
