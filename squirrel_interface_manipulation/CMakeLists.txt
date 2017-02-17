cmake_minimum_required(VERSION 2.8.3)
project(squirrel_interface_manipulation)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  rosplan_knowledge_msgs
  rosplan_dispatch_msgs
  nav_msgs
  move_base_msgs
  mongodb_store
  geometry_msgs
  diagnostic_msgs
  visualization_msgs
  squirrel_manipulation_msgs
  kclhand_control
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
  CATKIN_DEPENDS roscpp rospy std_msgs rosplan_knowledge_msgs rosplan_dispatch_msgs squirrel_manipulation_msgs nav_msgs mongodb_store geometry_msgs diagnostic_msgs move_base_msgs visualization_msgs kclhand_control tf
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

## Declare cpp executables
add_executable(rppushServer src/RPPushAction.cpp)
add_executable(rpgraspServer src/RPGraspAction.cpp)
add_executable(rphandoverServer src/RPHandoverAction.cpp)
add_dependencies(rppushServer ${catkin_EXPORTED_TARGETS})
add_dependencies(rpgraspServer ${catkin_EXPORTED_TARGETS})
add_dependencies(rphandoverServer ${catkin_EXPORTED_TARGETS})

## Specify libraries against which to link a library or executable target
target_link_libraries(rppushServer ${catkin_LIBRARIES})
target_link_libraries(rpgraspServer ${catkin_LIBRARIES})
target_link_libraries(rphandoverServer ${catkin_LIBRARIES})

##########
## Test ##
##########

# please do not use add_rosttest_gtest (seems to be interfering with qtcreator and cmake)
# see test documentation: http://wiki.ros.org/gtest
