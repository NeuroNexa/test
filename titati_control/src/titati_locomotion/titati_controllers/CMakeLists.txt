cmake_minimum_required(VERSION 3.8)
project(titati_controller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()
set(CMAKE_CXX_FLAGS "-std=c++17 -Wall -O3")

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()
find_package(Eigen3 REQUIRED)
find_package(CUDA REQUIRED)

if(DEFINED ENV{SIM_WEBOTS})
  add_definitions(-DSIM_WEBOTS)
  message("SIM_WEBOTS defined")
endif()

if(CMAKE_BUILD_TYPE STREQUAL "Debug")
message("Building control in debug mode")
  add_compile_options(
    -Wall -Wextra -Wpedantic
    -g
    -O0
    -Werror
  )
else()
  message("Building control in release mode")
  add_compile_options(
    -Wall -Wextra -Wpedantic
    -fomit-frame-pointer
    -finline-functions
    -floop-unroll-and-jam
    -O2
    # -Werror
  )
endif()

include_directories(include)
include_directories(madgwick)
include_directories(${EIGEN3_INCLUDE_DIR})
include_directories(/usr/local/cuda/include)
link_directories(/usr/local/cuda/lib64)

file(GLOB task_src "src/task/*.cpp")

set(ahrs_sources
madgwick/ahrs_madgwick.c
madgwick/FusionAhrs.c
madgwick/FusionCompass.c
madgwick/ahrs_madgwick.c
madgwick/FusionOffset.c
)

# generate parameter listener hpp file
generate_parameter_library(titati_controller_parameters
  config/titati_controller_parameters.yaml
)

pluginlib_export_plugin_description_file(controller_interface titati_controller_plugin.xml)

ament_auto_add_library(${PROJECT_NAME} SHARED
  ${task_src}
  ${ahrs_sources}
  src/titati_controller_node.cpp
)

install(DIRECTORY config DESTINATION share/${PROJECT_NAME})

target_link_libraries(titati_controller titati_controller_parameters cudart nvinfer)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_auto_package()
