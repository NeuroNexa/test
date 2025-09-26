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

# Pass the generated header directory to the parent scope so consumers can
# include the service definitions without relying on ament installation.
set(TITA_SYSTEM_INTERFACES_INCLUDE_DIR
  ${CMAKE_CURRENT_BINARY_DIR}/rosidl_generator_cpp
  PARENT_SCOPE)

