cmake_minimum_required(VERSION 3.8)
project(universe_dbw2_bridge)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(message_filters REQUIRED)
find_package(autoware_vehicle_msgs REQUIRED)
find_package(autoware_control_msgs REQUIRED)
find_package(tier4_vehicle_msgs REQUIRED)
find_package(tier4_api_msgs REQUIRED)
find_package(tier4_api_utils REQUIRED)
find_package(tier4_external_api_msgs REQUIRED)
find_package(ds_dbw_msgs REQUIRED)

add_executable(Lincoln_MKZ_Bridge src/Lincoln_MKZ_Bridge.cpp)
ament_target_dependencies(Lincoln_MKZ_Bridge rclcpp std_msgs sensor_msgs message_filters autoware_vehicle_msgs autoware_control_msgs tier4_vehicle_msgs tier4_api_msgs tier4_api_utils tier4_external_api_msgs ds_dbw_msgs)

install(TARGETS
  Lincoln_MKZ_Bridge
  DESTINATION lib/${PROJECT_NAME})

# Install config and launch folders
install(DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
