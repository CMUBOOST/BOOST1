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
  AlphaCentroid.srv
  ImageProcess.srv
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
  sensor_msgs
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

#add_executable(move_base5 src/move_base5.cpp)
#target_link_libraries(move_base5 ${catkin_LIBRARIES})

#add_executable(move_base6 src/move_base6.cpp)
#target_link_libraries(move_base6 ${catkin_LIBRARIES})

add_executable(move_base7 src/move_base7.cpp)
target_link_libraries(move_base7 ${catkin_LIBRARIES})

#add_executable(stitcher10 src/stitcher10.cpp)
#target_link_libraries(stitcher10 ${catkin_LIBRARIES} ${OpenCV_LIBS})

#add_executable(stitcher11 src/stitcher11.cpp)
#target_link_libraries(stitcher11 ${catkin_LIBRARIES} ${OpenCV_LIBS})

#add_executable(stitcher12 src/stitcher12.cpp)
#target_link_libraries(stitcher12 ${catkin_LIBRARIES} ${OpenCV_LIBS})

#add_executable(stitcher13 src/stitcher13.cpp)
#target_link_libraries(stitcher13 ${catkin_LIBRARIES} ${OpenCV_LIBS})

#add_executable(stitcher14 src/stitcher14.cpp)
#target_link_libraries(stitcher14 ${catkin_LIBRARIES} ${OpenCV_LIBS})
#add_dependencies(stitcher14 image_stitch_generate_messages_cpp)

#add_executable(stitcher15 src/stitcher15.cpp)
#target_link_libraries(stitcher15 ${catkin_LIBRARIES} ${OpenCV_LIBS})
#add_dependencies(stitcher15 image_stitch_generate_messages_cpp)

#add_executable(stitcher16 src/stitcher16.cpp)
#target_link_libraries(stitcher16 ${catkin_LIBRARIES} ${OpenCV_LIBS})
#add_dependencies(stitcher16 image_stitch_generate_messages_cpp)

#add_executable(stitcher17 src/stitcher17.cpp)
#target_link_libraries(stitcher17 ${catkin_LIBRARIES} ${OpenCV_LIBS})
#add_dependencies(stitcher17 image_stitch_generate_messages_cpp)

add_executable(stitcher18 src/stitcher18.cpp)
target_link_libraries(stitcher18 ${catkin_LIBRARIES} ${OpenCV_LIBS})
add_dependencies(stitcher18 image_stitch_generate_messages_cpp)

add_executable(stitcher19 src/stitcher19.cpp)
target_link_libraries(stitcher19 ${catkin_LIBRARIES} ${OpenCV_LIBS})
add_dependencies(stitcher19 image_stitch_generate_messages_cpp)

add_executable(stitcher20 src/stitcher20.cpp)
target_link_libraries(stitcher20 ${catkin_LIBRARIES} ${OpenCV_LIBS})
add_dependencies(stitcher20 image_stitch_generate_messages_cpp)

add_executable(stitcher21 src/stitcher21.cpp)
target_link_libraries(stitcher21 ${catkin_LIBRARIES} ${OpenCV_LIBS})
add_dependencies(stitcher21 image_stitch_generate_messages_cpp)

add_executable(stitcher22 src/stitcher22.cpp)
target_link_libraries(stitcher22 ${catkin_LIBRARIES} ${OpenCV_LIBS})
add_dependencies(stitcher22 image_stitch_generate_messages_cpp)

add_executable(stitcher_CNN src/stitcher_CNN.cpp)
target_link_libraries(stitcher_CNN ${catkin_LIBRARIES} ${OpenCV_LIBS})
add_dependencies(stitcher_CNN image_stitch_generate_messages_cpp)

add_executable(organized_test src/organized_test.cpp)
target_link_libraries(organized_test ${catkin_LIBRARIES} ${OpenCV_LIBS})
add_dependencies(organized_test organized_test_generate_messages_cpp)

#add_executable(stitcher_why src/stitcher_why.cpp)
#target_link_libraries(stitcher_why ${catkin_LIBRARIES} ${OpenCV_LIBS})

#add_executable(slow_down src/slow_down.cpp)
#target_link_libraries(slow_down ${catkin_LIBRARIES})

#add_executable(basic_shapes src/basic_shapes.cpp)
#target_link_libraries(basic_shapes ${catkin_LIBRARIES})

add_executable(tf_broadcaster src/tf_broadcaster.cpp)
target_link_libraries(tf_broadcaster ${catkin_LIBRARIES})
