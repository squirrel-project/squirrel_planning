cmake_minimum_required(VERSION 2.8.3)
project(squirrel_interface_speech)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  rosplan_knowledge_msgs
  rosplan_dispatch_msgs
  mongodb_store
  squirrel_speech_msgs
  squirrel_vad_msgs
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
  CATKIN_DEPENDS roscpp rospy std_msgs rosplan_knowledge_msgs rosplan_dispatch_msgs mongodb_store squirrel_speech_msgs squirrel_vad_msgs
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
set(SPEECH_SOURCES
	src/RPSpeechAction.cpp)

set(VAD_SOURCES
	src/VADSpeechInterface.cpp)

## Declare cpp executables
add_executable(rpspeechServer ${SPEECH_SOURCES})
add_dependencies(rpspeechServer ${catkin_EXPORTED_TARGETS})

add_executable(vadspeechServer ${VAD_SOURCES})
add_dependencies(vadspeechServer ${catkin_EXPORTED_TARGETS})

## Specify libraries against which to link a library or executable target
target_link_libraries(rpspeechServer ${catkin_LIBRARIES})
target_link_libraries(vadspeechServer ${catkin_LIBRARIES})

##########
## Test ##
##########

# please do not use add_rosttest_gtest (seems to be interfering with qtcreator and cmake)
# see test documentation: http://wiki.ros.org/gtest
