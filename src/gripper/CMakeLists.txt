cmake_minimum_required(VERSION 2.8.3)
project(gripper)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  rosbag
  message_generation
)

 add_service_files(
   FILES
   GripperActuate.srv
   GripperClose.srv
   GripperOpen.srv
 )

 generate_messages(
   DEPENDENCIES
   std_msgs
 )

# find the 32 or 64 bit libraries
if(CMAKE_SIZEOF_VOID_P EQUAL 8)
  set(ARCH "x86_64")
else()
  set(ARCH "x86")
endif()
message(STATUS "Detected architecture: ${ARCH}")

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ftd2xx EposCmd
  CATKIN_DEPENDS rosbag
)

###########
## Build ##
###########

include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
)

set(ftd2xx_LIBRARY ${PROJECT_SOURCE_DIR}/lib/${ARCH}/libftd2xx.so.1.1.12)
set(EposCmd_LIBRARY ${PROJECT_SOURCE_DIR}/lib/${ARCH}/libEposCmd.so.5.0.1.0)

message(STATUS "ftd2xx library is: ${ftd2xx_LIBRARY}")

add_library(ftd2xx SHARED ${ftd2xx_LIBRARY})
add_custom_command(TARGET ftd2xx POST_BUILD COMMAND cp ${ftd2xx_LIBRARY} ${CMAKE_LIBRARY_OUTPUT_DIRECTORY}/libftd2xx.so)
set_target_properties(ftd2xx PROPERTIES LINKER_LANGUAGE CXX)

add_library(EposCmd SHARED ${EposCmd_LIBRARY})
add_custom_command(TARGET EposCmd POST_BUILD COMMAND cp ${EposCmd_LIBRARY} ${CMAKE_LIBRARY_OUTPUT_DIRECTORY}/libEposCmd.so)
set_target_properties(EposCmd PROPERTIES LINKER_LANGUAGE CXX)

target_link_libraries(ftd2xx EposCmd) 

###add_executable(duo_node src/duo.cpp src/main.cpp src/dense3d.cpp)
add_executable(gripper_node src/gripper.cpp)
add_executable(gripper_node2 src/gripper2.cpp)
add_executable(gripper_node3 src/gripper3.cpp)
add_executable(gripper_node4 src/gripper4.cpp)
add_executable(gripper_node5 src/gripper5.cpp)
add_executable(gripper_node6 src/gripper6.cpp)
add_executable(gripper_close src/gripperClose.cpp)
add_executable(gripper_open src/gripperOpen.cpp)
add_executable(gripper_debug src/gripper_debug.cpp)
add_executable(load_cell_ticksToPounds src/load_cell_ticksToPounds.cpp)
#add_executable(dense3d src/duo_dense3d.cpp)

target_link_libraries(gripper_node ftd2xx) 
target_link_libraries(gripper_node2 ftd2xx) 
target_link_libraries(gripper_node3 ftd2xx) 
target_link_libraries(gripper_node2 ${catkin_LIBRARIES})
target_link_libraries(gripper_node3 ${catkin_LIBRARIES})
target_link_libraries(gripper_debug ftd2xx) 
target_link_libraries(gripper_debug ${catkin_LIBRARIES})
target_link_libraries(gripper_node4 ftd2xx) 
target_link_libraries(gripper_node4 ${catkin_LIBRARIES})
target_link_libraries(gripper_node5 ftd2xx) 
target_link_libraries(gripper_node5 ${catkin_LIBRARIES})
target_link_libraries(gripper_node6 ftd2xx) 
target_link_libraries(gripper_node6 ${catkin_LIBRARIES})
target_link_libraries(gripper_close ftd2xx) 
target_link_libraries(gripper_close ${catkin_LIBRARIES})
target_link_libraries(gripper_open ftd2xx) 
target_link_libraries(gripper_open ${catkin_LIBRARIES})
target_link_libraries(load_cell_ticksToPounds ${catkin_LIBRARIES})


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

# Mark udev install program for installation
install(PROGRAMS
  install_udev_rules
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(FILES 90-ftd2xx.rules
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)




#add_dependencies(gripper_node ${PROJECT_NAME}_gencfg)

# for linking duo_node to dynamic_reconfigure cfg files

#set(DUO_LIB "${DUO_SDK_ARCH}/libDUO.so")
###set(DENSE_LIB "${DUO_SDK_ARCH}/libDense3D.so")

#target_link_libraries(gripper_node " ")
#set_target_properties(duo_node PROPERTIES COMPILE_FLAGS -g)
#target_link_libraries(dense3d ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${DUO_LIB} ${DENSE_LIB} )
#set_target_properties(dense3d PROPERTIES COMPILE_FLAGS -g)

#if(WIN32)
#        target_link_libraries(duo_node DUOlib)
#        target_link_libraries(dense3d DUOlib)
#elseif(UNIX OR APPLE)
#	target_link_libraries(duo_node duo)
#        target_link_libraries(dense3d duo)
#endif()

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_duo3d_ros.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
