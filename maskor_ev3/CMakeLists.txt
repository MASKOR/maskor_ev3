cmake_minimum_required(VERSION 2.8.3)
project(maskor_ev3)

#comment this in to cross compile for ARM
set(CMAKE_CXX_COMPILER /usr/bin/arm-linux-gnueabi-g++)
set(CMAKE_C_COMPILER /usr/bin/arm-linux-gnueabi-gcc)

add_definitions(-D_GLIBCXX_USE_NANOSLEEP)
add_compile_options(-std=c++11)

#comment this in if using raspberry pi
#add_definitions(-DEV3DEV_PLATFORM_BRICKPI)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rosserial_client
#  std_msgs
#  message_generation
)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)


## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

################################################
## Declare ROS messages, services and actions ##
################################################

## To declare and build messages, services or actions from within this
## package, follow these steps:
## * Let MSG_DEP_SET be the set of packages whose message types you use in
##   your messages/services/actions (e.g. std_msgs, actionlib_msgs, ...).
## * In the file package.xml:
##   * add a build_depend tag for "message_generation"
##   * add a build_depend and a run_depend tag for each package in MSG_DEP_SET
##   * If MSG_DEP_SET isn't empty the following dependency has been pulled in
##     but can be declared for certainty nonetheless:
##     * add a run_depend tag for "message_runtime"
## * In this file (CMakeLists.txt):
##   * add "message_generation" and every package in MSG_DEP_SET to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * add "message_runtime" and every package in MSG_DEP_SET to
##     catkin_package(CATKIN_DEPENDS ...)
##   * uncomment the add_*_files sections below as needed
##     and list every .msg/.srv/.action file to be processed
##   * uncomment the generate_messages entry below
##   * add every package in MSG_DEP_SET to generate_messages(DEPENDENCIES ...)

## Generate messages in the 'msg' folder
# add_message_files(
#   FILES
#   ColorSensorMsg.msg
#)

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   lift_up.srv
#   )

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
# generate_messages(
#   DEPENDENCIES
#   std_msgs  # Or other packages containing msgs
# )

# rosserial_generate_ros_lib(
#   PACKAGE rosserial_client
#   SCRIPT make_libraries.py
# )

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

## To declare and build dynamic reconfigure parameters within this
## package, follow these steps:
## * In the file package.xml:
##   * add a build_depend and a run_depend tag for "dynamic_reconfigure"
## * In this file (CMakeLists.txt):
##   * add "dynamic_reconfigure" to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * uncomment the "generate_dynamic_reconfigure_options" section below
##     and list every .cfg file to be processed

## Generate dynamic reconfigure parameters in the 'cfg' folder
# generate_dynamic_reconfigure_options(
#   cfg/DynReconf1.cfg
#   cfg/DynReconf2.cfg
# )

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES maskor_ev3
  CATKIN_DEPENDS roscpp
  #  DEPENDS system_lib
  )

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  include/ros_lib/ #rosserial
  ${catkin_INCLUDE_DIRS}
  )

## Declare a C++ library
add_library(maskor_ev3
  include/maskor_ev3/maskor_ev3.h
  include/maskor_ev3/common.h
  include/maskor_ev3/lru_cache.h
  include/maskor_ev3/port.h src/port.cpp
  include/maskor_ev3/device.h src/device.cpp
  include/maskor_ev3/sensor.h src/sensor.cpp
  include/maskor_ev3/i2c_sensor.h src/i2c_sensor.cpp
  include/maskor_ev3/touch_sensor.h src/touch_sensor.cpp
  include/maskor_ev3/color_sensor.h src/color_sensor.cpp
  include/maskor_ev3/infrared_sensor.h src/infrared_sensor.cpp
  include/maskor_ev3/gyro_sensor.h src/gyro_sensor.cpp
  include/maskor_ev3/ultrasonic_sensor.h src/ultrasonic_sensor.cpp
  include/maskor_ev3/sound_sensor.h src/sound_sensor.cpp
  include/maskor_ev3/light_sensor.h src/light_sensor.cpp 
  include/maskor_ev3/motor.h src/motor.cpp
  include/maskor_ev3/medium_motor.h src/medium_motor.cpp
  include/maskor_ev3/large_motor.h src/large_motor.cpp
  include/maskor_ev3/dc_motor.h src/dc_motor.cpp
  include/maskor_ev3/servo_motor.h src/servo_motor.cpp
  include/maskor_ev3/led.h src/led.cpp
  include/maskor_ev3/power_supply.h src/power_supply.cpp
  include/maskor_ev3/button.h src/button.cpp
  include/maskor_ev3/sound.h src/sound.cpp
  include/maskor_ev3/lcd.h src/lcd.cpp
  include/maskor_ev3/remote_control.h src/remote_control.cpp
  )

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
# add_dependencies(maskor_ev3 ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Declare a C++ executable
add_executable(maskor_ev3_bobb3e_node src/maskor_ev3_bobb3e_node.cpp)


## Add cmake target dependencies of the executable
## same as for the library above
# add_dependencies(maskor_ev3_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(maskor_ev3_bobb3e_node maskor_ev3_msgs_generate_messages_cpp)
#add_dependencies(turn_left_for_x_secs maskor_ev3_msgs_generate_messages_cpp)

## Specify libraries to link a library or executable target against
target_link_libraries(maskor_ev3
  #${catkin_LIBRARIES}
)

target_link_libraries(maskor_ev3_bobb3e_node
  #${catkin_LIBRARIES}
  maskor_ev3
)

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## automatic deployment of the nodes and libs to the EV3
#set(NFS_MOUNT /home/mstuettgen/ev3)
#set(CATKIN_PACKAGE_BIN_DESTINATION ${NFS_MOUNT})
#set(CATKIN_PACKAGE_LIB_DESTINATION ${NFS_MOUNT})

## Mark executables and/or libraries for installation
#install(TARGETS maskor_ev3 maskor_ev3_bobb3e_node
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
#)

## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_maskor_ev3.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
