cmake_minimum_required(VERSION 2.8.3)
project(image_stitch)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  image_transport
  pcl_conversions
  pcl_ros
  roscpp
  rospy
  std_msgs
  sensor_msgs
  geometry_msgs
  std_srvs
  message_generation
  tf
  serial
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
#   Message1.msg
#   Message2.msg
# )

## Generate services in the 'srv' folder
add_service_files(
  FILES
  CloudStitch.srv
#   Service2.srv
)

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs
)

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
#  LIBRARIES image_stitch
#  CATKIN_DEPENDS roscpp rospy std_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
)

## Declare a C++ library
# add_library(image_stitch
#   src/${PROJECT_NAME}/image_stitch.cpp
# )
#add_library(image_stitch_lib OBJECT
#  include/image_stitch/Vec3.h
#)

add_executable(move_base src/move_base.cpp)
target_link_libraries(move_base ${catkin_LIBRARIES})

add_executable(move_base3 src/move_base3.cpp)
target_link_libraries(move_base3 ${catkin_LIBRARIES})

add_executable(move_base4 src/move_base4.cpp)
target_link_libraries(move_base4 ${catkin_LIBRARIES})

add_executable(move_base5 src/move_base5.cpp)
target_link_libraries(move_base5 ${catkin_LIBRARIES})

#add_executable(stitcher src/stitcher.cpp)
#target_link_libraries(stitcher ${catkin_LIBRARIES})

#add_executable(stitcher2 src/stitcher2.cpp)
#target_link_libraries(stitcher2 ${catkin_LIBRARIES})

#add_executable(stitcher4 src/stitcher4.cpp)
#target_link_libraries(stitcher4 ${catkin_LIBRARIES} ${OpenCV_LIBS})

add_executable(stitcher5 src/stitcher5.cpp)
target_link_libraries(stitcher5 ${catkin_LIBRARIES} ${OpenCV_LIBS})

add_executable(stitcher6_simple src/stitcher6_simple.cpp)
target_link_libraries(stitcher6_simple ${catkin_LIBRARIES} ${OpenCV_LIBS})

add_executable(stitcher9 src/stitcher9.cpp)
target_link_libraries(stitcher9 ${catkin_LIBRARIES} ${OpenCV_LIBS})

add_executable(stitcher10 src/stitcher10.cpp)
target_link_libraries(stitcher10 ${catkin_LIBRARIES} ${OpenCV_LIBS})

add_executable(stitcher_why src/stitcher_why.cpp)
target_link_libraries(stitcher_why ${catkin_LIBRARIES} ${OpenCV_LIBS})

add_executable(slow_down src/slow_down.cpp)
target_link_libraries(slow_down ${catkin_LIBRARIES})

add_executable(basic_shapes src/basic_shapes.cpp)
target_link_libraries(basic_shapes ${catkin_LIBRARIES})

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
# add_dependencies(image_stitch ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Declare a C++ executable
# add_executable(image_stitch_node src/image_stitch_node.cpp)

## Add cmake target dependencies of the executable
## same as for the library above
# add_dependencies(image_stitch_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
# target_link_libraries(image_stitch_node
#   ${catkin_LIBRARIES}
# )

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

## Mark executables and/or libraries for installation
# install(TARGETS image_stitch image_stitch_node
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

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
# catkin_add_gtest(${PROJECT_NAME}-test test/test_image_stitch.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
