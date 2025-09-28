cmake_minimum_required(VERSION 3.8)
project(titati_power_services)

option(RL_SAR_HARDWARE_ONLY "Build without simulator extras for RL SAR" OFF)
mark_as_advanced(RL_SAR_HARDWARE_ONLY)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Werror -Wno-unused-function)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(titati_topics REQUIRED)
find_package(titati_system_interfaces REQUIRED)

add_library(${PROJECT_NAME} SHARED
  src/battery.cpp
  src/battery_device_node.cpp
  src/update_battery_info.cpp
  src/update_power_info.cpp
  src/power_management.cpp
)
target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_17)
target_include_directories(${PROJECT_NAME}
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
)
ament_target_dependencies(${PROJECT_NAME}
  PUBLIC
    rclcpp
    sensor_msgs
    diagnostic_msgs
    titati_topics
    titati_system_interfaces
)

add_executable(${PROJECT_NAME}_node
  src/main.cpp
)
target_link_libraries(${PROJECT_NAME}_node ${PROJECT_NAME})
ament_target_dependencies(${PROJECT_NAME}_node
  rclcpp
  sensor_msgs
  diagnostic_msgs
  titati_topics
  titati_system_interfaces
)

install(TARGETS
  ${PROJECT_NAME}
  ${PROJECT_NAME}_node
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION include/${PROJECT_NAME}
)

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_include_directories(include/${PROJECT_NAME})
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(
  rclcpp
  sensor_msgs
  diagnostic_msgs
  titati_topics
  titati_system_interfaces
)
ament_package()
