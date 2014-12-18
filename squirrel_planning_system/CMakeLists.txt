cmake_minimum_required(VERSION 2.8.3)
project(squirrel_planning_system)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  std_srvs
  actionlib
  squirrel_planning_dispatch_msgs
  squirrel_planning_knowledge_msgs
  mongodb_store
  diagnostic_msgs
)

find_package(Boost REQUIRED COMPONENTS
  filesystem
)

###################################
## catkin specific configuration ##
###################################

## Declare things to be passed to dependent projects
catkin_package(
  LIBRARIES squirrel_planning_system
  CATKIN_DEPENDS roscpp rospy std_msgs std_srvs actionlib squirrel_planning_dispatch_msgs squirrel_planning_knowledge_msgs mongodb_store diagnostic_msgs
  DEPENDS
)

###########
## Build ##
###########

## include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
  src/VALfiles
  include
)

## val sources
set(VAL_SOURCES
	src/VALfiles/DebugWriteController.cpp
	src/VALfiles/FastEnvironment.cpp
	src/VALfiles/FuncAnalysis.cpp
	src/VALfiles/SASActions.cpp
	src/VALfiles/SimpleEval.cpp
	src/VALfiles/TIM.cpp
	src/VALfiles/TimSupport.cpp
	src/VALfiles/ToFunction.cpp
	src/VALfiles/TypedAnalyser.cpp
	src/VALfiles/ValidatorAPI.cpp
	src/VALfiles/instantiation.cpp
	src/VALfiles/pddl+.cpp
	src/VALfiles/ptree.cpp
	src/VALfiles/typecheck.cpp
	src/VALfiles/Validator.cpp
	src/VALfiles/Action.cpp
	src/VALfiles/Proposition.cpp
	src/VALfiles/FuncExp.cpp
	src/VALfiles/RepairAdvice.cpp
	src/VALfiles/State.cpp
	src/VALfiles/Plan.cpp
	src/VALfiles/Ownership.cpp
	src/VALfiles/Environment.cpp
	src/VALfiles/Events.cpp
	src/VALfiles/Polynomial.cpp
	src/VALfiles/TrajectoryConstraints.cpp
	src/VALfiles/RobustAnalyse.cpp
	src/VALfiles/PrettyPrinter.cpp
	src/VALfiles/random.cpp
	src/VALfiles/LaTeXSupport.cpp
	src/VALfiles/Utils.cpp)

## Declare cpp executables
add_executable(planner src/PlanningLoop.cpp	${VAL_SOURCES})
add_executable(rpmovebase src/interface/RPMoveBase.cpp)
add_executable(rpperceptionaction src/interface/RPPerceptionAction.cpp)
add_executable(trpmovebase test/utRPMoveBase.cpp)

## Specify libraries against which to link a library or executable target
target_link_libraries(planner ${catkin_LIBRARIES})
target_link_libraries(rpmovebase ${catkin_LIBRARIES} ${Boost_LIBRARIES})
target_link_libraries(rpperceptionaction ${catkin_LIBRARIES} ${Boost_LIBRARIES})
target_link_libraries(trpmovebase ${catkin_LIBRARIES} ${Boost_LIBRARIES})

# add_dependencies(rpmovebase ${learning_actionlib_EXPORTED_TARGETS})

##########
## Test ##
##########
# please do not use add_rosttest_gtest (seems to be interfering with qtcreator and cmake)
# see test documentation: http://wiki.ros.org/gtest
#add_rostest_gtest(${PROJECT_NAME}_utest test/utest.cpp)
#target_link_libraries(testPlanningSystem ${catkin_LIBRARIES} ${GTEST_LIBRARIES})
