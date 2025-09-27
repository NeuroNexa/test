cmake_minimum_required(VERSION 3.8)
project(titati_motor_tools)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tita_robot REQUIRED)

add_executable(motor_test_node src/motor_test_node.cpp)
target_link_libraries(motor_test_node tita_robot)
ament_target_dependencies(motor_test_node rclcpp tita_robot)

install(TARGETS motor_test_node DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

ament_package()
