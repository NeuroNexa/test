cmake_minimum_required(VERSION 3.10)
project(tita_system_interfaces LANGUAGES CXX)

find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(diagnostic_msgs REQUIRED)

set(SRV_FILES
  "srv/FanModeSetSrv.srv"
  "srv/HeadLightControlSrv.srv"
  "srv/LegLightControlSrv.srv"
  "srv/PowerHeartBeatSrv.srv"
  "srv/PowerSelfTestSrv.srv"
  "srv/PowerStateSetSrv.srv"
  "srv/TailLightControlSrv.srv"
)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${SRV_FILES}
  DEPENDENCIES std_srvs diagnostic_msgs
)

# Export include directories so parent projects can consume generated headers
set(${PROJECT_NAME}_INCLUDE_DIRS
  ${CMAKE_CURRENT_BINARY_DIR}/rosidl_generator_cpp/${PROJECT_NAME})
set(${PROJECT_NAME}_TARGETS
  ${PROJECT_NAME}__rosidl_generator_cpp
  ${PROJECT_NAME}__rosidl_typesupport_cpp
)

foreach(target IN LISTS ${PROJECT_NAME}_TARGETS)
  if(TARGET ${target})
    set_target_properties(${target} PROPERTIES
      FOLDER "tita_system_interfaces"
    )
  endif()
endforeach()

# Provide an interface target that consumers can link against to pick up include dirs
add_library(${PROJECT_NAME} INTERFACE)
target_include_directories(${PROJECT_NAME} INTERFACE
  $<BUILD_INTERFACE:${${PROJECT_NAME}_INCLUDE_DIRS}>
)
foreach(target IN LISTS ${PROJECT_NAME}_TARGETS)
  if(TARGET ${target})
    add_dependencies(${PROJECT_NAME} ${target})
  endif()
endforeach()

