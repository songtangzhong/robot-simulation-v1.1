cmake_minimum_required(VERSION 3.5)
project(shared_memory)

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
find_package(robot_info REQUIRED)

add_library(${PROJECT_NAME} SHARED
  src/shared_memory.cpp
  src/semaphore.cpp
)
target_include_directories(${PROJECT_NAME} 
  PRIVATE 
  include
  ${robot_info_INCLUDE_DIRS}
)
target_link_libraries(${PROJECT_NAME} ${robot_info_LIBRARIES})
ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  robot_info
)

add_executable(test_shm 
  test/test_shm.cpp
)
target_include_directories(test_shm 
  PRIVATE 
  include
  ${robot_info_INCLUDE_DIRS}
)
target_link_libraries(test_shm 
  ${PROJECT_NAME}
  ${robot_info_LIBRARIES}
)
ament_target_dependencies(test_shm 
  rclcpp
  robot_info
)

add_executable(test_sem
  test/test_sem.cpp
)
target_include_directories(test_sem 
  PRIVATE 
  include
  ${robot_info_INCLUDE_DIRS}
)
target_link_libraries(test_sem 
  ${PROJECT_NAME}
  ${robot_info_LIBRARIES}
)
ament_target_dependencies(test_sem 
  rclcpp
  robot_info
)

add_executable(test_shm_with_sem
  test/test_shm_with_sem.cpp
)
target_include_directories(test_shm_with_sem 
  PRIVATE 
  include
  ${robot_info_INCLUDE_DIRS}
)
target_link_libraries(test_shm_with_sem 
  ${PROJECT_NAME}
  ${robot_info_LIBRARIES}
)
ament_target_dependencies(test_shm_with_sem 
  rclcpp
  robot_info
)

add_executable(test_pv_sem
  test/test_pv_sem.cpp
)
target_include_directories(test_pv_sem 
  PRIVATE 
  include
  ${robot_info_INCLUDE_DIRS}
)
target_link_libraries(test_pv_sem 
  ${PROJECT_NAME}
  ${robot_info_LIBRARIES}
)
ament_target_dependencies(test_pv_sem 
  rclcpp
  robot_info
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
  test_shm
  test_sem
  test_shm_with_sem
  test_pv_sem
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
  robot_info
)

ament_package()
