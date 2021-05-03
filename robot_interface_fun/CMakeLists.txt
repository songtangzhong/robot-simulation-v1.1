cmake_minimum_required(VERSION 3.5)
project(robot_interface_fun)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(robot_info REQUIRED)
find_package(shared_memory REQUIRED)

add_library(${PROJECT_NAME} SHARED
  src/robot_interface_fun.cpp
)
target_include_directories(${PROJECT_NAME} 
  PRIVATE 
  include
  ${robot_info_INCLUDE_DIRS}
  ${shared_memory_INCLUDE_DIRS}
)
target_link_libraries(${PROJECT_NAME} 
  ${robot_info_LIBRARIES}
  ${shared_memory_LIBRARIES}
)
ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  sensor_msgs
  robot_info
  shared_memory
)

add_executable(test_robot_interface_fun
  test/test_robot_interface_fun.cpp
)
target_include_directories(test_robot_interface_fun
  PRIVATE
  include
)
target_link_libraries(test_robot_interface_fun
  ${PROJECT_NAME}
)
ament_target_dependencies(test_robot_interface_fun
  rclcpp
  sensor_msgs
  robot_info
  shared_memory
)

install(
  TARGETS 
  ${PROJECT_NAME}
  RUNTIME DESTINATION lib
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
)

install(TARGETS
  test_robot_interface_fun
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

ament_export_libraries(
  ${PROJECT_NAME}
)

ament_export_include_directories(
  include
)

ament_export_dependencies(
  rclcpp
  sensor_msgs
  robot_info
  shared_memory
)

ament_package()
