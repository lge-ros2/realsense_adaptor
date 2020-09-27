###############################################################################
# Set minimum required version of cmake, project name and compile options
################################################################################
cmake_minimum_required(VERSION 3.5)
project(realsense_adaptor)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -O2)
endif()

################################################################################
# Find colcon packages and libraries for colcon and system dependencies
################################################################################
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(message_filters REQUIRED)
find_package(realsense_adaptor_msgs REQUIRED)

set(dependencies
  rclcpp
  sensor_msgs
  realsense_adaptor_msgs
  message_filters
)

################################################################################
# Build
################################################################################
include_directories(include)

add_executable(${PROJECT_NAME}
  src/realsense_adaptor.cpp
  src/main.cpp)

ament_target_dependencies(${PROJECT_NAME}
  ${dependencies}
)

################################################################################
# Install
################################################################################
install(TARGETS ${PROJECT_NAME}
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME})

ament_package()