cmake_minimum_required(VERSION 2.8.3)
project(epos4_control)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
    hardware_interface
    transmission_interface
    controller_manager
    roscpp
    diagnostic_updater
    control_msgs
    actionlib
    std_msgs
)

# find the 32 or 64 bit libraries
if(CMAKE_SIZEOF_VOID_P EQUAL 8)
  set(ARCH "x86_64")
else()
  set(ARCH "x86")
endif()
message(STATUS "Detected architecture: ${ARCH}")

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
#   Message1.msg
#   Message2.msg
# )

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

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
  LIBRARIES epos4_control ftd2xx EposCmd epos_library_utils epos_manager
  CATKIN_DEPENDS hardware_interface roscpp rospy controller_manager diagnostic_updater control_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(include "${CMAKE_SOURCE_DIR}/include/${PROJECT_NAME}")
include_directories(SYSTEM ${Boost_INCLUDE_DIR})
include_directories(${catkin_INCLUDE_DIRS})
link_directories(${catkin_LIBRARY_DIRS})
link_directories("~/catkin_ws/devel/lib")   #so it works in QT for some reason

set(ftd2xx_LIBRARY ${PROJECT_SOURCE_DIR}/lib/${ARCH}/libftd2xx.so.1.3.6)
set(EposCmd_LIBRARY ${PROJECT_SOURCE_DIR}/lib/${ARCH}/libEposCmd.so.6.1.1.0)

add_library(ftd2xx SHARED ${ftd2xx_LIBRARY})
add_custom_command(TARGET ftd2xx POST_BUILD COMMAND cp ${ftd2xx_LIBRARY} ${CMAKE_LIBRARY_OUTPUT_DIRECTORY}/libftd2xx.so)
set_target_properties(ftd2xx PROPERTIES LINKER_LANGUAGE CXX)

add_library(EposCmd SHARED ${EposCmd_LIBRARY})
add_custom_command(TARGET EposCmd POST_BUILD COMMAND cp ${EposCmd_LIBRARY} ${CMAKE_LIBRARY_OUTPUT_DIRECTORY}/libEposCmd.so)
set_target_properties(EposCmd PROPERTIES LINKER_LANGUAGE CXX)

# A collection of utilities for using the EPOS Command Libary
add_library(epos_library_utils
  src/util/utils.cpp
)
target_link_libraries(epos_library_utils
  ${catkin_LIBRARIES}
  ftd2xx
  EposCmd
)


# Build tool to list available devices
add_executable(list_devices src/tools/list_devices.cpp)
target_link_libraries(list_devices
  ${catkin_LIBRARIES}
  epos_library_utils
)

add_executable(read_io src/tools/read_io.cpp)
target_link_libraries(read_io
  ${catkin_LIBRARIES}
  epos_library_utils
)

# Build tool to list a devices state
add_executable(get_state src/tools/get_state.cpp)
target_link_libraries(get_state
  ${catkin_LIBRARIES}
  epos_library_utils
)

add_library(epos_manager
  src/util/epos.cpp src/util/epos_manager.cpp
)
target_link_libraries(epos_manager
  ${catkin_LIBRARIES}
  epos_library_utils
  ftd2xx
  EposCmd
)

add_library(epos4_control
  src/util/epos4_control.cpp
)
target_link_libraries(epos4_control
  ${catkin_LIBRARIES}
  epos_manager
  epos_library_utils
)


add_executable(epos4_control_node src/nodes/epos4_control_node.cpp)
target_link_libraries(epos4_control_node
  ${catkin_LIBRARIES}
  epos4_control
)

#############################################################################################################################################

#add_library(epos4_trajectory_commander
#  src/util/epos4_trajectory_commander.cpp
#)
#target_link_libraries(epos4_trajectory_commander
#  ${catkin_LIBRARIES}
#  ClassicTrajectory
#)

#add_executable(epos4_trajectory_commander_node src/nodes/epos4_trajectory_commander_node.cpp)
#target_link_libraries(epos4_trajectory_commander_node
#  ${catkin_LIBRARIES}
#  epos4_trajectory_commander
#  epos4_control
#)

#############################################################################################################################################

add_executable(trajectory_test_node src/nodes/trajectory_test_node.cpp)
target_link_libraries(trajectory_test_node
  ${catkin_LIBRARIES}
#  epos4_control
)


#############
## Install ##
#############

install(TARGETS ftd2xx EposCmd
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE
)

install(PROGRAMS
  install_udev_rules
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(FILES 99-ftdi.rules 99-epos4.rules
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

# Mark libraries and nodes for installation
install(TARGETS epos_library_utils epos_manager epos4_control list_devices get_state epos4_control_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE
)

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_epos4_control.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
