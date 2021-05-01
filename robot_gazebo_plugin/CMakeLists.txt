cmake_minimum_required(VERSION 3.5)
project(robot_gazebo_plugin)

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
find_package(gazebo_dev REQUIRED)
find_package(rclcpp REQUIRED)
find_package(robot_info REQUIRED)
find_package(shared_memory REQUIRED)

add_library(${PROJECT_NAME} SHARED
  src/robot_gazebo_plugin.cpp
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
  gazebo_dev
  rclcpp
  robot_info
  shared_memory
)

add_executable(test_gazebo_plugin_write
  test/test_gazebo_plugin_write.cpp
)
target_include_directories(test_gazebo_plugin_write 
  PRIVATE 
  ${robot_info_INCLUDE_DIRS}
  ${shared_memory_INCLUDE_DIRS}
)
target_link_libraries(test_gazebo_plugin_write
  ${robot_info_LIBRARIES}
  ${shared_memory_LIBRARIES}
)
ament_target_dependencies(test_gazebo_plugin_write 
  rclcpp
  robot_info
  shared_memory
)

add_executable(test_gazebo_plugin_read
  test/test_gazebo_plugin_read.cpp
)
target_include_directories(test_gazebo_plugin_read 
  PRIVATE 
  ${robot_info_INCLUDE_DIRS}
  ${shared_memory_INCLUDE_DIRS}
)
target_link_libraries(test_gazebo_plugin_read
  ${robot_info_LIBRARIES}
  ${shared_memory_LIBRARIES}
)
ament_target_dependencies(test_gazebo_plugin_read 
  rclcpp
  robot_info
  shared_memory
)

install(DIRECTORY include/
  DESTINATION include
)

install(TARGETS ${PROJECT_NAME}
  RUNTIME DESTINATION lib
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
)

install(TARGETS 
  test_gazebo_plugin_write
  test_gazebo_plugin_read
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

ament_export_libraries(
  ${PROJECT_NAME}
)

ament_export_include_directories(
  include
)

ament_export_dependencies(
  gazebo_dev
  rclcpp
  robot_info
  shared_memory
)

ament_package()
