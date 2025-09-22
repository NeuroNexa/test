cmake_minimum_required(VERSION 3.5)
project(rl_sar VERSION 3.0.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

set(USE_CMAKE OFF CACHE BOOL "Build without ROS (standalone mode)")
set(HARDWARE_ONLY OFF CACHE BOOL "Disable simulation-only components such as Gazebo")
message(STATUS "USE_CMAKE (requested): ${USE_CMAKE}")
message(STATUS "HARDWARE_ONLY: ${HARDWARE_ONLY}")

set(BUILD_STANDALONE ${USE_CMAKE})
if(NOT BUILD_STANDALONE AND NOT DEFINED ENV{ROS_DISTRO})
    message(WARNING "ROS_DISTRO not set, enabling standalone mode (USE_CMAKE)")
    set(BUILD_STANDALONE ON)
endif()

if(NOT BUILD_STANDALONE)
    if($ENV{ROS_DISTRO} MATCHES "noetic")
        add_compile_definitions(USE_ROS USE_ROS1)
    elseif($ENV{ROS_DISTRO} MATCHES "foxy|humble")
        add_compile_definitions(USE_ROS USE_ROS2)
    else()
        message(WARNING "Unsupported ROS distribution '$ENV{ROS_DISTRO}'. Building without ROS integration.")
        set(BUILD_STANDALONE ON)
    endif()
endif()

message(STATUS "BUILD_STANDALONE (effective): ${BUILD_STANDALONE}")

if(BUILD_STANDALONE)
    add_compile_definitions(USE_CMAKE)
    set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)
    set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
    set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
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

if(NOT BUILD_STANDALONE)
    if($ENV{ROS_DISTRO} MATCHES "noetic")
        set(catkin_components
            roscpp
            geometry_msgs
        )
        if(NOT HARDWARE_ONLY)
            list(APPEND catkin_components
                controller_manager
                joint_state_controller
                robot_state_publisher
                std_msgs
                tf
                robot_msgs
                robot_joint_controller
                rospy
                gazebo_ros
            )
        endif()
        find_package(catkin REQUIRED COMPONENTS ${catkin_components})
        if(NOT HARDWARE_ONLY)
            catkin_package(
                CATKIN_DEPENDS
                    robot_joint_controller
                    rospy
            )
        else()
            catkin_package(
                CATKIN_DEPENDS
                    roscpp
                    geometry_msgs
            )
        endif()
    elseif($ENV{ROS_DISTRO} MATCHES "foxy|humble")
        find_package(ament_cmake REQUIRED)
        find_package(rclcpp REQUIRED)
        find_package(geometry_msgs REQUIRED)
        if(NOT HARDWARE_ONLY)
            find_package(joint_state_broadcaster REQUIRED)
            find_package(robot_state_publisher REQUIRED)
            find_package(std_msgs REQUIRED)
            find_package(robot_msgs REQUIRED)
            find_package(robot_joint_controller REQUIRED)
            find_package(rclpy REQUIRED)
            find_package(std_srvs REQUIRED)
            find_package(gazebo_ros REQUIRED)
            find_package(gazebo_msgs REQUIRED)
        endif()
        ament_package()
    endif()
    if(NOT HARDWARE_ONLY)
        find_package(gazebo REQUIRED)
        add_compile_options(${GAZEBO_CXX_FLAGS})
    endif()
endif()

find_package(Torch REQUIRED)
find_package(TBB REQUIRED)
find_package(Threads REQUIRED)
find_package(Python3 COMPONENTS Interpreter Development REQUIRED)

link_directories(/usr/local/lib)
include_directories(${YAML_CPP_INCLUDE_DIR})

# Titati hardware interface (vendored into rl_sar)
add_library(titati_robot_sdk
    library/hardware/titati/src/tita_robot.cpp
    library/hardware/titati/src/can_receiver.cpp
    library/hardware/titati/src/can_sender.cpp
)
target_include_directories(titati_robot_sdk PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/library/hardware/titati/include
)
target_link_libraries(titati_robot_sdk PUBLIC Threads::Threads)

add_library(titati_canfd_router_sdk
    library/hardware/titati/src/canfd_router_can_receive_api.cpp
)
target_include_directories(titati_canfd_router_sdk PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/library/hardware/titati/include
)
target_link_libraries(titati_canfd_router_sdk PUBLIC Threads::Threads)

if(NOT BUILD_STANDALONE)
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
    library/hardware/titati/include
    policy
)

add_library(rl_sdk library/core/rl_sdk/rl_sdk.cpp)
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
if(NOT BUILD_STANDALONE)
    if($ENV{ROS_DISTRO} MATCHES "foxy|humble")
        install(TARGETS rl_sdk DESTINATION lib/${PROJECT_NAME})
    endif()
endif()

add_library(observation_buffer library/core/observation_buffer/observation_buffer.cpp)
target_link_libraries(observation_buffer PUBLIC "${TORCH_LIBRARIES}")
if(NOT BUILD_STANDALONE)
    if($ENV{ROS_DISTRO} MATCHES "foxy|humble")
        install(TARGETS observation_buffer DESTINATION lib/${PROJECT_NAME})
    endif()
endif()

if(NOT BUILD_STANDALONE AND NOT HARDWARE_ONLY)
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
    titati_robot_sdk
    rl_sdk
    observation_buffer
    yaml-cpp
)
if(NOT BUILD_STANDALONE)
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

add_executable(titati_canfd_router_node src/titati_canfd_router_node.cpp)
target_link_libraries(titati_canfd_router_node
    titati_canfd_router_sdk
)
if(NOT BUILD_STANDALONE)
    if($ENV{ROS_DISTRO} MATCHES "noetic")
        target_link_libraries(titati_canfd_router_node ${catkin_LIBRARIES})
    elseif($ENV{ROS_DISTRO} MATCHES "foxy|humble")
        ament_target_dependencies(titati_canfd_router_node
            rclcpp
        )
        install(TARGETS titati_canfd_router_node DESTINATION lib/${PROJECT_NAME})
    endif()
endif()

add_executable(test_titati_motors src/test_titati_motors.cpp)
target_link_libraries(test_titati_motors
    titati_robot_sdk
    yaml-cpp
)
if(NOT BUILD_STANDALONE)
    if($ENV{ROS_DISTRO} MATCHES "noetic")
        target_link_libraries(test_titati_motors ${catkin_LIBRARIES})
    elseif($ENV{ROS_DISTRO} MATCHES "foxy|humble")
        install(TARGETS test_titati_motors DESTINATION lib/${PROJECT_NAME})
    endif()
endif()

if(NOT BUILD_STANDALONE)
    if($ENV{ROS_DISTRO} MATCHES "noetic")
        catkin_install_python(PROGRAMS
            scripts/actuator_net.py
            DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
        )
        install(PROGRAMS
            scripts/setup_titati_canfd.sh
            scripts/run_titati_canfd_router.sh
            DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
        )
    elseif($ENV{ROS_DISTRO} MATCHES "foxy|humble")
        install(PROGRAMS
            scripts/actuator_net.py
            scripts/setup_titati_canfd.sh
            scripts/run_titati_canfd_router.sh
            DESTINATION lib/${PROJECT_NAME}
        )
    endif()
endif()

if(NOT BUILD_STANDALONE)
    if($ENV{ROS_DISTRO} MATCHES "foxy|humble")
        install(
            DIRECTORY launch worlds policy
            DESTINATION share/${PROJECT_NAME}/${dir}
        )
    endif()
endif()

# only for test
# add_executable(test_observation_buffer test/test_observation_buffer.cpp)
# target_link_libraries(test_observation_buffer
#     observation_buffer
# )
# install(TARGETS test_observation_buffer DESTINATION lib/${PROJECT_NAME})
