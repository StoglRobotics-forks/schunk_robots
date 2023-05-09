cmake_minimum_required(VERSION 3.8)
project(schunk_lwa4p)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 20)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED COMPONENTS roslaunch rostest)
find_package(lely_core_libraries REQUIRED)

generate_dcf(schunk_lwa4p)


### TESTING ###

if(BUILD_TESTING)
endif()

### INSTALL ###
install(DIRECTORY config launch urdf
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
