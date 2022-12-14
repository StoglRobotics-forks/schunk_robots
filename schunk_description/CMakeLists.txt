cmake_minimum_required(VERSION 3.8)
project(schunk_description)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 20)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED COMPONENTS)

if(BUILD_TESTING)
endif()

################################################################################
# Install
################################################################################
install(DIRECTORY Media meshes urdf launch ros2_control
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
