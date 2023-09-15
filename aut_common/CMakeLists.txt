cmake_minimum_required(VERSION 3.8)
project(aut_common)

add_compile_options(-Wall -Wextra -Wpedantic -Ofast)

find_package(ament_cmake REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED)
find_package(aut_utils REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(nav_msgs REQUIRED)


set(library_name common)

include_directories(
  include
  ${EIGEN3_INCLUDE_DIR}
)

add_library(${library_name} SHARED 
  src/graph.cc src/transformer.cc src/transform_smoother.cc
)
ament_target_dependencies(${library_name} Eigen3 visualization_msgs geometry_msgs PCL aut_utils tf2_ros builtin_interfaces nav_msgs)
target_link_libraries(${library_name} ${Eigen3_LIBRARIES} ${PCL_LIBRARIES})

install(TARGETS
  ${library_name}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include/
)

ament_export_include_directories(include)
ament_export_libraries(${library_name})
ament_export_dependencies(Eigen3 visualization_msgs geometry_msgs aut_utils tf2_ros builtin_interfaces nav_msgs)

ament_package()
