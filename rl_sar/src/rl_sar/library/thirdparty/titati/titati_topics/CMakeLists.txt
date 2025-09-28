cmake_minimum_required(VERSION 3.8)
project(titati_topics)

option(RL_SAR_HARDWARE_ONLY "Build without simulator extras for RL SAR" OFF)
mark_as_advanced(RL_SAR_HARDWARE_ONLY)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)

ament_export_include_directories(include)

install(DIRECTORY include/
  DESTINATION include
)

ament_package()
