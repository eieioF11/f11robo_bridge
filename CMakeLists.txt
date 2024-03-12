cmake_minimum_required(VERSION 3.8)
project(f11robo_bridge)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

ament_auto_add_library(${PROJECT_NAME}_component SHARED
  src/${PROJECT_NAME}_component.cpp)

rclcpp_components_register_nodes(${PROJECT_NAME}_component "F11RoboBridge")
target_compile_options(${PROJECT_NAME}_component PUBLIC -Wall)

ament_auto_add_executable(${PROJECT_NAME} src/main.cpp)

install(DIRECTORY
  config
  launch
  xacro
  meshes
  DESTINATION share/${PROJECT_NAME}/
)

ament_auto_package()