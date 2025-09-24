cmake_minimum_required(VERSION 3.10)
project(titati_canfd_router LANGUAGES CXX)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tita_system_interfaces REQUIRED)
find_package(Threads REQUIRED)

add_library(${PROJECT_NAME}_lib SHARED
  src/canfd_router_can_receive_api.cpp
)
target_include_directories(${PROJECT_NAME}_lib
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
)
ament_target_dependencies(${PROJECT_NAME}_lib
  PUBLIC
    rclcpp
    tita_system_interfaces
)
target_link_libraries(${PROJECT_NAME}_lib PUBLIC Threads::Threads)

add_executable(${PROJECT_NAME}_node src/main.cpp)
target_link_libraries(${PROJECT_NAME}_node
  PRIVATE
    ${PROJECT_NAME}_lib
)
ament_target_dependencies(${PROJECT_NAME}_node
  rclcpp
  tita_system_interfaces
)

install(TARGETS
  ${PROJECT_NAME}_lib
  ${PROJECT_NAME}_node
  EXPORT ${PROJECT_NAME}-targets
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/${PROJECT_NAME}
  DESTINATION include/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_include_directories(include/${PROJECT_NAME})
ament_export_targets(${PROJECT_NAME}-targets HAS_LIBRARY_TARGET)
ament_package()
