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

  rosidl_get_typesupport_target(tita_system_interfaces_cpp_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
  rosidl_get_typesupport_target(tita_system_interfaces_introspection_target ${PROJECT_NAME} "rosidl_typesupport_introspection_cpp")
  set(TITA_SYSTEM_INTERFACES_INCLUDE_DIRS
    "${CMAKE_CURRENT_BINARY_DIR}/rosidl_generator_cpp"
    "${CMAKE_CURRENT_BINARY_DIR}/rosidl_typesupport_cpp"
    "${CMAKE_CURRENT_BINARY_DIR}/rosidl_typesupport_introspection_cpp"
    PARENT_SCOPE)
  set(TITA_SYSTEM_INTERFACES_TYPESUPPORT_CPP_TARGET "${tita_system_interfaces_cpp_target}" PARENT_SCOPE)
  set(TITA_SYSTEM_INTERFACES_TYPESUPPORT_INTROSPECTION_CPP_TARGET "${tita_system_interfaces_introspection_target}" PARENT_SCOPE)

  ament_export_dependencies(rosidl_default_runtime)
  ament_package()
else()
  message(FATAL_ERROR "Unsupported ROS distribution: $ENV{ROS_DISTRO}")
endif()
