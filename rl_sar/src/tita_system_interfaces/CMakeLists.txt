cmake_minimum_required(VERSION 3.5)
project(tita_system_interfaces VERSION 3.0.0)

if($ENV{ROS_DISTRO} MATCHES "noetic")
  find_package(catkin REQUIRED COMPONENTS
    diagnostic_msgs
    message_generation
  )

  add_service_files(
    FILES
      PowerHeartBeatSrv.srv
      PowerSelfTestSrv.srv
      PowerStateSetSrv.srv
  )

  generate_messages(
    DEPENDENCIES
      diagnostic_msgs
  )

  catkin_package(
    CATKIN_DEPENDS
      diagnostic_msgs
      message_runtime
  )
elseif($ENV{ROS_DISTRO} MATCHES "foxy|humble")
  find_package(ament_cmake REQUIRED)
  find_package(rosidl_default_generators REQUIRED)
  find_package(diagnostic_msgs REQUIRED)

  rosidl_generate_interfaces(${PROJECT_NAME}
    "srv/PowerHeartBeatSrv.srv"
    "srv/PowerSelfTestSrv.srv"
    "srv/PowerStateSetSrv.srv"
    DEPENDENCIES diagnostic_msgs
  )

  ament_export_dependencies(rosidl_default_runtime)
  ament_package()
else()
  message(FATAL_ERROR "Unsupported ROS distribution: $ENV{ROS_DISTRO}")
endif()
