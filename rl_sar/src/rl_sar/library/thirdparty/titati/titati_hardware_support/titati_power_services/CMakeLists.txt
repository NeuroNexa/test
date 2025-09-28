cmake_minimum_required(VERSION 3.8)
project(titati_power_services)

option(RL_SAR_HARDWARE_ONLY "Build without simulator extras for RL SAR" OFF)
mark_as_advanced(RL_SAR_HARDWARE_ONLY)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Werror -Wno-unused-function)

  # add_compile_options(-Wall -Wextra -Wpedantic -w)
endif()

set(CMAKE_CXX_FLAGS "-std=c++17 -Wall -O3")

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

include_directories(include)
include_directories(utils)

ament_auto_add_library(${PROJECT_NAME}_lib SHARED
  src/battery.cpp
  src/battery_device_node.cpp
  src/update_battery_info.cpp
  src/update_power_info.cpp
  src/power_management.cpp
)

ament_auto_add_executable(${PROJECT_NAME}_node
  src/main.cpp
)

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_auto_package()
