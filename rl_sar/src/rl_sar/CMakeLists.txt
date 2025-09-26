cmake_minimum_required(VERSION 3.5)
project(rl_sar VERSION 3.0.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)


set(USE_CMAKE OFF CACHE BOOL "Use Cmake build system")
message(STATUS "USE_CMAKE: ${USE_CMAKE}")

if(DEFINED ENV{ROS_DISTRO})
    set(DEFAULT_ENABLE_ROS ON)
else()
    set(DEFAULT_ENABLE_ROS OFF)
endif()

option(ENABLE_ROS "Enable ROS integration when building with CMake" ${DEFAULT_ENABLE_ROS})

if(USE_CMAKE)
    set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)
    set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
    set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
    add_compile_definitions(USE_CMAKE)
endif()

if(NOT USE_CMAKE AND NOT ENABLE_ROS)
    message(WARNING "ROS build requested without ENABLE_ROS; forcing ROS integration on.")
    set(ENABLE_ROS ON CACHE BOOL "Enable ROS integration when building with CMake" FORCE)
endif()

set(ROS_LIBRARIES "")

if(NOT USE_CMAKE)
    if($ENV{ROS_DISTRO} MATCHES "noetic")
        add_compile_definitions(USE_ROS USE_ROS1)
    elseif($ENV{ROS_DISTRO} MATCHES "foxy|humble|galactic|iron|rolling")
        add_compile_definitions(USE_ROS USE_ROS2)
    endif()
elseif(ENABLE_ROS)
    if(NOT DEFINED ENV{ROS_DISTRO})
        message(WARNING "ENABLE_ROS requested but ROS_DISTRO is not set. Disabling ROS integration.")
        set(ENABLE_ROS OFF)
    elseif($ENV{ROS_DISTRO} MATCHES "noetic")
        add_compile_definitions(USE_ROS USE_ROS1)
        find_package(catkin REQUIRED COMPONENTS
            roscpp
            geometry_msgs
            std_msgs
        )
        include_directories(${catkin_INCLUDE_DIRS})
        list(APPEND ROS_LIBRARIES ${catkin_LIBRARIES})
    elseif($ENV{ROS_DISTRO} MATCHES "foxy|humble|galactic|iron|rolling")
        add_compile_definitions(USE_ROS USE_ROS2)
        find_package(rclcpp REQUIRED)
        find_package(geometry_msgs REQUIRED)
        find_package(sensor_msgs REQUIRED)
        find_package(diagnostic_msgs REQUIRED)
        find_package(std_srvs REQUIRED)
        find_package(rosidl_default_generators REQUIRED)
        list(APPEND ROS_LIBRARIES rclcpp::rclcpp)
        if(TARGET geometry_msgs::geometry_msgs)
            list(APPEND ROS_LIBRARIES geometry_msgs::geometry_msgs)
        elseif(TARGET geometry_msgs::geometry_msgs__rosidl_typesupport_cpp)
            list(APPEND ROS_LIBRARIES geometry_msgs::geometry_msgs__rosidl_typesupport_cpp)
        else()
            message(WARNING "geometry_msgs imported target not available; falling back to include directories only.")
            if(DEFINED geometry_msgs_INCLUDE_DIRS)
                include_directories(${geometry_msgs_INCLUDE_DIRS})
            endif()
        endif()
    else()
        message(WARNING "Unsupported ROS_DISTRO '${ENV{ROS_DISTRO}}'. Disabling ROS integration.")
        set(ENABLE_ROS OFF)
    endif()
endif()

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Debug)
endif()
string(TOUPPER "${CMAKE_BUILD_TYPE}" UPPER_BUILD_TYPE)
if(UPPER_BUILD_TYPE STREQUAL "DEBUG")
    add_compile_options(-g)
endif()

add_definitions(-DCMAKE_CURRENT_SOURCE_DIR="${CMAKE_CURRENT_SOURCE_DIR}")
add_definitions(-DBOOST_BIND_GLOBAL_PLACEHOLDERS)

if(NOT USE_CMAKE)
    if($ENV{ROS_DISTRO} MATCHES "noetic")
        find_package(catkin REQUIRED COMPONENTS
            controller_manager
            genmsg
            joint_state_controller
            robot_state_publisher
            roscpp
            gazebo_ros
            std_msgs
            tf
            geometry_msgs
            robot_msgs
            robot_joint_controller
            rospy
        )
        catkin_package(
            CATKIN_DEPENDS
            robot_joint_controller
            rospy
        )
    elseif($ENV{ROS_DISTRO} MATCHES "foxy|humble")
        find_package(ament_cmake REQUIRED)
        find_package(joint_state_broadcaster REQUIRED)
        find_package(robot_state_publisher REQUIRED)
        find_package(rclcpp REQUIRED)
        find_package(gazebo_ros REQUIRED)
        find_package(std_msgs REQUIRED)
        find_package(robot_msgs REQUIRED)
        find_package(robot_joint_controller REQUIRED)
        find_package(rclpy REQUIRED)
        find_package(gazebo_msgs REQUIRED)
        find_package(std_srvs REQUIRED)
        find_package(geometry_msgs REQUIRED)
        ament_package()
    endif()
    add_compile_options(${GAZEBO_CXX_FLAGS})
    find_package(gazebo REQUIRED)
endif()

find_package(Torch REQUIRED)
find_package(TBB REQUIRED)
find_package(Threads REQUIRED)
find_package(Python3 COMPONENTS Interpreter Development REQUIRED)

link_directories(/usr/local/lib)
include_directories(${YAML_CPP_INCLUDE_DIR})

if(NOT USE_CMAKE)
    if($ENV{ROS_DISTRO} MATCHES "noetic")
        include_directories(${catkin_INCLUDE_DIRS})
    endif()
endif()

include_directories(
    include
    library/core/matplotlibcpp
    library/core/observation_buffer
    library/core/rl_sdk
    library/core/loop
    library/core/fsm
    policy
)

set(TITATI_HARDWARE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/library/thirdparty/titati_hardware)
set(TITATI_ROBOT_DIR ${TITATI_HARDWARE_DIR}/tita_robot)
set(TITATI_UTILS_DIR ${TITATI_HARDWARE_DIR}/tita_utils)

if(ENABLE_ROS)
    add_subdirectory(${TITATI_HARDWARE_DIR}/tita_interfaces/tita_system_interfaces)
endif()
add_library(titati_hardware
    ${TITATI_ROBOT_DIR}/src/tita_robot.cpp
    ${TITATI_ROBOT_DIR}/src/can_receiver.cpp
    ${TITATI_ROBOT_DIR}/src/can_sender.cpp
)
target_include_directories(titati_hardware PUBLIC
    ${TITATI_ROBOT_DIR}/include
)
set_target_properties(titati_hardware PROPERTIES
    CXX_STANDARD 17
    CXX_STANDARD_REQUIRED ON
)
target_link_libraries(titati_hardware PUBLIC Threads::Threads)

add_library(rl_sdk library/core/rl_sdk/rl_sdk.cpp)
set_target_properties(rl_sdk PROPERTIES
    CXX_STANDARD 17
    CXX_STANDARD_REQUIRED ON
)
target_link_libraries(rl_sdk PUBLIC
    "${TORCH_LIBRARIES}"
    Python3::Python
    Python3::Module
    TBB::tbb
)
find_package(Python3 COMPONENTS NumPy)
if(Python3_NumPy_FOUND)
    target_link_libraries(rl_sdk PUBLIC Python3::NumPy)
else()
    target_compile_definitions(rl_sdk PUBLIC WITHOUT_NUMPY)
endif()
if(NOT USE_CMAKE)
    if($ENV{ROS_DISTRO} MATCHES "foxy|humble")
        install(TARGETS rl_sdk DESTINATION lib/${PROJECT_NAME})
    endif()
endif()

add_library(observation_buffer library/core/observation_buffer/observation_buffer.cpp)
target_link_libraries(observation_buffer PUBLIC "${TORCH_LIBRARIES}")
set_target_properties(observation_buffer PROPERTIES
    CXX_STANDARD 17
    CXX_STANDARD_REQUIRED ON
)
if(NOT USE_CMAKE)
    if($ENV{ROS_DISTRO} MATCHES "foxy|humble")
        install(TARGETS observation_buffer DESTINATION lib/${PROJECT_NAME})
    endif()
endif()

if(NOT USE_CMAKE)
    add_executable(rl_sim src/rl_sim.cpp)
    target_link_libraries(rl_sim
        rl_sdk
        observation_buffer
        yaml-cpp
        Threads::Threads
    )
    if($ENV{ROS_DISTRO} MATCHES "noetic")
        target_link_libraries(rl_sim ${catkin_LIBRARIES})
    elseif($ENV{ROS_DISTRO} MATCHES "foxy|humble")
        ament_target_dependencies(rl_sim
            joint_state_broadcaster
            robot_state_publisher
            rclcpp
            gazebo_ros
            std_msgs
            robot_msgs
            robot_joint_controller
            rclpy
            gazebo_msgs
            std_srvs
            geometry_msgs
        )
        install(TARGETS rl_sim DESTINATION lib/${PROJECT_NAME})
    endif()
endif()

add_executable(rl_real_titati src/rl_real_titati.cpp)
target_link_libraries(rl_real_titati
    titati_hardware
    rl_sdk
    observation_buffer
    yaml-cpp
    ${ROS_LIBRARIES}
)
if(NOT USE_CMAKE)
    if($ENV{ROS_DISTRO} MATCHES "noetic")
        target_link_libraries(rl_real_titati ${catkin_LIBRARIES})
    elseif($ENV{ROS_DISTRO} MATCHES "foxy|humble")
        ament_target_dependencies(rl_real_titati
            rclcpp
            geometry_msgs
        )
        install(TARGETS rl_real_titati DESTINATION lib/${PROJECT_NAME})
    endif()
endif()

add_executable(titati_motor_test src/titati_motor_test.cpp)
target_link_libraries(titati_motor_test
    titati_hardware
)
if(NOT USE_CMAKE)
    if($ENV{ROS_DISTRO} MATCHES "noetic")
        install(TARGETS titati_motor_test DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
    elseif($ENV{ROS_DISTRO} MATCHES "foxy|humble")
        install(TARGETS titati_motor_test DESTINATION lib/${PROJECT_NAME})
    endif()
endif()

if(NOT USE_CMAKE)
    if($ENV{ROS_DISTRO} MATCHES "noetic")
        catkin_install_python(PROGRAMS
            scripts/actuator_net.py
            DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
        )
        install(PROGRAMS
            scripts/titati_can_setup_master.sh
            scripts/titati_can_setup_slave.sh
            DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
        )
    elseif($ENV{ROS_DISTRO} MATCHES "foxy|humble")
        install(PROGRAMS
            scripts/actuator_net.py
            scripts/titati_can_setup_master.sh
            scripts/titati_can_setup_slave.sh
            DESTINATION lib/${PROJECT_NAME}
        )
    endif()
endif()

if(NOT USE_CMAKE)
    if($ENV{ROS_DISTRO} MATCHES "foxy|humble")
        install(
            DIRECTORY launch worlds policy
            DESTINATION share/${PROJECT_NAME}/${dir}
        )
    endif()
endif()

# ROS 2 hardware support utilities for Titati
if(ENABLE_ROS)
    set(TITATI_BATTERY_DIR ${TITATI_HARDWARE_DIR}/battery_device)
    add_library(titati_battery_device
        ${TITATI_BATTERY_DIR}/src/battery.cpp
        ${TITATI_BATTERY_DIR}/src/battery_device_node.cpp
        ${TITATI_BATTERY_DIR}/src/update_battery_info.cpp
        ${TITATI_BATTERY_DIR}/src/update_power_info.cpp
        ${TITATI_BATTERY_DIR}/src/power_management.cpp
    )
    target_include_directories(titati_battery_device PUBLIC
        ${TITATI_BATTERY_DIR}/include
        ${TITATI_BATTERY_DIR}/utils
        ${TITATI_UTILS_DIR}
    )
    target_link_libraries(titati_battery_device
        Threads::Threads
        ${ROS_LIBRARIES}
        sensor_msgs::sensor_msgs
        diagnostic_msgs::diagnostic_msgs
        tita_system_interfaces
    )
    rosidl_target_interfaces(titati_battery_device
        tita_system_interfaces "rosidl_typesupport_cpp")
    add_dependencies(titati_battery_device tita_system_interfaces__rosidl_generator_cpp)

    add_executable(battery_device_node ${TITATI_BATTERY_DIR}/src/main.cpp)
    target_link_libraries(battery_device_node
        titati_battery_device
    )
    add_dependencies(battery_device_node tita_system_interfaces__rosidl_generator_cpp)

    set(TITATI_CANFD_ROUTER_DIR ${TITATI_HARDWARE_DIR}/titati_canfd_router)
    add_library(titati_canfd_router
        ${TITATI_CANFD_ROUTER_DIR}/src/canfd_router_can_receive_api.cpp
    )
    target_include_directories(titati_canfd_router PUBLIC
        ${TITATI_CANFD_ROUTER_DIR}/include
        ${TITATI_ROBOT_DIR}/include
        ${TITATI_UTILS_DIR}
    )
    target_link_libraries(titati_canfd_router
        titati_hardware
        Threads::Threads
    )

    add_executable(titati_canfd_router_node ${TITATI_CANFD_ROUTER_DIR}/src/main.cpp)
    target_link_libraries(titati_canfd_router_node
        titati_canfd_router
        ${ROS_LIBRARIES}
        tita_system_interfaces
    )
    rosidl_target_interfaces(titati_canfd_router_node
        tita_system_interfaces "rosidl_typesupport_cpp")
    add_dependencies(titati_canfd_router_node tita_system_interfaces__rosidl_generator_cpp)
endif()

# only for test
# add_executable(test_observation_buffer test/test_observation_buffer.cpp)
# target_link_libraries(test_observation_buffer
#     observation_buffer
# )
# install(TARGETS test_observation_buffer DESTINATION lib/${PROJECT_NAME})
