cmake_minimum_required(VERSION 3.8)
project(fastlio_localization)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(pcl_conversions REQUIRED)

# PCL 只用到 common/io/filters/registration/kdtree/search，避免把 VTK 等全部链进来。
find_package(PCL REQUIRED COMPONENTS common io filters registration kdtree search)

add_executable(relocalization_node src/relocalization_node.cpp)
target_include_directories(relocalization_node PUBLIC ${PCL_INCLUDE_DIRS})
target_link_libraries(relocalization_node
  ${PCL_COMMON_LIBRARIES}
  ${PCL_IO_LIBRARIES}
  ${PCL_FILTERS_LIBRARIES}
  ${PCL_REGISTRATION_LIBRARIES}
  ${PCL_KDTREE_LIBRARIES}
  ${PCL_SEARCH_LIBRARIES}
)
ament_target_dependencies(relocalization_node
  rclcpp
  sensor_msgs
  geometry_msgs
  nav_msgs
  std_srvs
  tf2
  tf2_ros
  pcl_conversions
)

install(TARGETS relocalization_node
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME})

install(PROGRAMS tools/auto_drive_map.py
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
