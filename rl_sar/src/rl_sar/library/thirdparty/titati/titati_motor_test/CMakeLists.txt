cmake_minimum_required(VERSION 3.8)
project(titati_motor_test)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(titati_can_driver REQUIRED)

add_executable(motor_test_node src/motor_test_node.cpp)
target_link_libraries(motor_test_node
  titati_can_driver::titati_can_driver
)
ament_target_dependencies(motor_test_node rclcpp titati_can_driver)

install(TARGETS motor_test_node DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

ament_package()
