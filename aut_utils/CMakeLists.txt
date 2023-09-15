cmake_minimum_required(VERSION 3.8)
project(aut_utils)

add_compile_options(-Wall -Wextra -Wpedantic -Ofast)

find_package(ament_cmake REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(PCL REQUIRED)

set(library_name utils)

include_directories(
  include
  ${EIGEN3_INCLUDE_DIR}
)

add_library(${library_name} SHARED 
  src/utils.cc
)
ament_target_dependencies(${library_name} Eigen3 geometry_msgs PCL)
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
ament_export_dependencies(Eigen3 geometry_msgs PCL)

ament_package()
