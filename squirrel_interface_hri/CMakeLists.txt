cmake_minimum_required(VERSION 2.8.3)
project(squirrel_interface_hri)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  rosplan_knowledge_msgs
  rosplan_dispatch_msgs
  nav_msgs
  mongodb_store
  geometry_msgs
  diagnostic_msgs
  visualization_msgs
  tf
)

find_package(Boost REQUIRED COMPONENTS
  filesystem
)

###################################
## catkin specific configuration ##
###################################

## Declare things to be passed to dependent projects
catkin_package(
  LIBRARIES squirrel_knowledge_base
  CATKIN_DEPENDS roscpp rospy std_msgs rosplan_knowledge_msgs nav_msgs mongodb_store geometry_msgs diagnostic_msgs visualization_msgs tf
  DEPENDS
)

###########
## Build ##
###########

## include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
  include
)

## Pointing service
set(RPPointingService_SOURCES
	src/RPPointingServer.cpp)

set(FollowChildService_SOURCES
	src/FollowChildAction.cpp)

## Declare cpp executables
add_executable(rppointingServer ${RPPointingService_SOURCES})
add_dependencies(rppointingServer ${catkin_EXPORTED_TARGETS})

add_executable(FollowChildServer ${FollowChildService_SOURCES})
add_dependencies(FollowChildServer ${catkin_EXPORTED_TARGETS})

## Specify libraries against which to link a library or executable target
target_link_libraries(rppointingServer ${catkin_LIBRARIES})
target_link_libraries(FollowChildServer ${catkin_LIBRARIES})

##########
## Test ##
##########
# please do not use add_rosttest_gtest (seems to be interfering with qtcreator and cmake)
# see test documentation: http://wiki.ros.org/gtest
