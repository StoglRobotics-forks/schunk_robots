cmake_minimum_required(VERSION 3.8)
project(schunk_lwa4p)

find_package(ament_cmake REQUIRED COMPONENTS roslaunch rostest )

ament_package()

### TESTING ###
if(BUILD_TESTING)
    #roslaunch_add_file_check(launch)
    #add_rostest(urdf/test/urdf.test)
    find_package(launch_testing_ament_cmake)
    add_launch_test(urdf/test/urdf.test)
endif()

### INSTALL ###
install(DIRECTORY config launch urdf
  DESTINATION share/${PROJECT_NAME}/
)
