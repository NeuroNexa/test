cmake_minimum_required(VERSION 3.16)
project(rl_sar VERSION 3.0.0 LANGUAGES CXX)

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
add_compile_definitions(USE_CMAKE)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE RelWithDebInfo)
endif()
string(TOUPPER "${CMAKE_BUILD_TYPE}" UPPER_BUILD_TYPE)
if(UPPER_BUILD_TYPE STREQUAL "DEBUG")
  add_compile_options(-g)
endif()

add_definitions(-DCMAKE_CURRENT_SOURCE_DIR="${CMAKE_CURRENT_SOURCE_DIR}")
add_definitions(-DBOOST_BIND_GLOBAL_PLACEHOLDERS)

find_package(Torch REQUIRED)
find_package(TBB REQUIRED)
find_package(Threads REQUIRED)
find_package(Python3 COMPONENTS Interpreter Development REQUIRED)
find_package(Python3 COMPONENTS NumPy)
find_package(yaml-cpp REQUIRED)

link_directories(/usr/local/lib)

include_directories(
  include
  library/core/matplotlibcpp
  library/core/observation_buffer
  library/core/rl_sdk
  library/core/loop
  library/core/fsm
  library/titati_sdk/include
  policy
)

if(CMAKE_SYSTEM_PROCESSOR MATCHES "aarch64|arm64|ARM64")
  message(STATUS "Current system architecture: ${CMAKE_SYSTEM_PROCESSOR}")
  set(ARCH_DIR "aarch64")
  set(UNITREE_LIB "libunitree_legged_sdk_arm64")
elseif(CMAKE_SYSTEM_PROCESSOR MATCHES "x86_64|amd64|AMD64")
  message(STATUS "Current system architecture: ${CMAKE_SYSTEM_PROCESSOR}")
  set(ARCH_DIR "x86_64")
  set(UNITREE_LIB "libunitree_legged_sdk_amd64")
else()
  message(FATAL_ERROR "Unsupported architecture: ${CMAKE_SYSTEM_PROCESSOR}")
endif()

# unitree_legged_sdk-3.2
add_library(unitree_legged_sdk SHARED IMPORTED)
set_target_properties(unitree_legged_sdk PROPERTIES
  INTERFACE_INCLUDE_DIRECTORIES "${CMAKE_CURRENT_SOURCE_DIR}/library/thirdparty/unitree_legged_sdk-3.2/include"
  IMPORTED_LOCATION "${CMAKE_CURRENT_SOURCE_DIR}/library/thirdparty/unitree_legged_sdk-3.2/lib/${UNITREE_LIB}.so"
)
set(UNITREE_A1_LIBS Threads::Threads unitree_legged_sdk lcm)

# unitree_sdk2-2.0.0
set(BUILD_EXAMPLES OFF CACHE BOOL "Disable examples build for unitree_sdk2" FORCE)
add_subdirectory(library/thirdparty/unitree_sdk2-2.0.0)

# l4w4_sdk
add_library(l4w4_sdk INTERFACE)
target_include_directories(l4w4_sdk INTERFACE
  library/thirdparty/l4w4_sdk
)
target_link_libraries(l4w4_sdk INTERFACE Threads::Threads)

# Lite3_MotionSDK
add_library(lite3_motionsdk SHARED IMPORTED)
set_target_properties(lite3_motionsdk PROPERTIES
  INTERFACE_INCLUDE_DIRECTORIES "${CMAKE_CURRENT_SOURCE_DIR}/library/thirdparty/Lite3_MotionSDK/include"
  IMPORTED_LOCATION "${CMAKE_CURRENT_SOURCE_DIR}/library/thirdparty/Lite3_MotionSDK/lib/libdeeprobotics_legged_sdk_${ARCH_DIR}.so"
)
set(LITE3_REAL_LIBS Threads::Threads lite3_motionsdk)

# Retroid Gamepad
set(GAMEPAD_SRC
  ${CMAKE_CURRENT_SOURCE_DIR}/library/thirdparty/gamepad/src/gamepad.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/library/thirdparty/gamepad/src/retroid_gamepad.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/library/thirdparty/gamepad/src/udp_receiver.cpp
)

add_library(rl_sdk library/core/rl_sdk/rl_sdk.cpp)
target_link_libraries(rl_sdk PUBLIC
  "${TORCH_LIBRARIES}"
  Python3::Python
  Python3::Module
  TBB::tbb
)
if(Python3_NumPy_FOUND)
  target_link_libraries(rl_sdk PUBLIC Python3::NumPy)
else()
  target_compile_definitions(rl_sdk PUBLIC WITHOUT_NUMPY)
endif()

add_library(observation_buffer library/core/observation_buffer/observation_buffer.cpp)
target_link_libraries(observation_buffer PUBLIC "${TORCH_LIBRARIES}")

add_library(titati_sdk
  library/titati_sdk/src/can_receiver.cpp
  library/titati_sdk/src/can_sender.cpp
  library/titati_sdk/src/titati_robot.cpp
)
target_link_libraries(titati_sdk PUBLIC Threads::Threads)
target_include_directories(titati_sdk PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/library/titati_sdk/include)

add_library(titati_can_router_lib STATIC
  ../titati_canfd_router/src/canfd_router_can_receive_api.cpp
)
target_include_directories(titati_can_router_lib PUBLIC
  ${CMAKE_CURRENT_SOURCE_DIR}/../titati_canfd_router/include
)
target_link_libraries(titati_can_router_lib PUBLIC Threads::Threads)

add_executable(rl_real_titati src/rl_real_titati.cpp)
target_link_libraries(rl_real_titati
  titati_sdk
  rl_sdk
  observation_buffer
  yaml-cpp
  Threads::Threads
  titati_can_router_lib
)

add_executable(titati_motor_test src/titati_motor_test.cpp)
target_link_libraries(titati_motor_test
  titati_sdk
  yaml-cpp
  Threads::Threads
  titati_can_router_lib
)

add_executable(titati_can_router
  ../titati_canfd_router/src/main_cli.cpp
)
target_include_directories(titati_can_router PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/../titati_canfd_router/include
)
target_link_libraries(titati_can_router
  Threads::Threads
  titati_can_router_lib
)

# only for test
# add_executable(test_observation_buffer test/test_observation_buffer.cpp)
# target_link_libraries(test_observation_buffer
#     observation_buffer
# )
# install(TARGETS test_observation_buffer DESTINATION lib/${PROJECT_NAME})
