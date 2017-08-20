cmake_minimum_required(VERSION 2.8.3)
project(automatic_exposure)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  rospy
  roscpp
  dynamic_reconfigure
  roslaunch
  message_generation
  std_msgs
  sensor_msgs
)



## Generate services in the 'srv' folder
 add_service_files(
   FILES
   AutomaticExposure.srv
)

## Generate added messages and services with any dependencies listed here
generate_messages(
   DEPENDENCIES
   automatic_exposure
   std_msgs
   sensor_msgs
)

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################


catkin_package(
  CATKIN_DEPENDS message_runtime 
)

###########
## Build ##
###########

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(automatic_exposure src/automatic_exposure_server.cpp)
add_dependencies(automatic_exposure ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(automatic_exposure
   ${catkin_LIBRARIES}
 )


add_executable(get_intensity src/get_intensity.cpp)
add_dependencies(get_intensity ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(get_intensity
   ${catkin_LIBRARIES}
 )
