cmake_minimum_required(VERSION 2.8.3)
project(kcl_planning_system)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
	roscpp
    dynamic_reconfigure 
	std_msgs
	planning_dispatch_msgs
	planning_knowledge_msgs
)

# protected region additional user defined REQUIREMENTS on begin #
# protected region additional user defined REQUIREMENTS end #

generate_dynamic_reconfigure_options(cfg/planning_system.cfg cfg/manipulation_planning_interface.cfg cfg/pathfinding_planning_interface.cfg )

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  INCLUDE_DIRS
    common/src
  CATKIN_DEPENDS 
  	roscpp
    dynamic_reconfigure
	std_msgs
	planning_dispatch_msgs
	planning_knowledge_msgs
   
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(common/include)
include_directories(common/src)
include_directories(${catkin_INCLUDE_DIRS})


## Declare a cpp executable
add_executable(planning_system ros/src/planning_system_ros.cpp)

## Add cmake target dependencies of the executable/library
## as an example, message headers may need to be generated before nodes
add_dependencies(planning_system planning_system_gencfg)
add_dependencies(planning_system ${kcl_planning_system_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
target_link_libraries(planning_system
  ${catkin_LIBRARIES}
)
## Declare a cpp executable
add_executable(manipulation_planning_interface ros/src/manipulation_planning_interface_ros.cpp)

## Add cmake target dependencies of the executable/library
## as an example, message headers may need to be generated before nodes
add_dependencies(manipulation_planning_interface manipulation_planning_interface_gencfg)
add_dependencies(manipulation_planning_interface ${kcl_planning_system_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
target_link_libraries(manipulation_planning_interface
  ${catkin_LIBRARIES}
)
## Declare a cpp executable
add_executable(pathfinding_planning_interface ros/src/pathfinding_planning_interface_ros.cpp)

## Add cmake target dependencies of the executable/library
## as an example, message headers may need to be generated before nodes
add_dependencies(pathfinding_planning_interface pathfinding_planning_interface_gencfg)
add_dependencies(pathfinding_planning_interface ${kcl_planning_system_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
target_link_libraries(pathfinding_planning_interface
  ${catkin_LIBRARIES}
)

# protected region additional user defined BUILD STATEMENTS on begin #
# protected region additional user defined BUILD STATEMENTS end #


#############
## Install ##
#############

## Mark executables and/or libraries for installation
install(TARGETS planning_system
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
## Mark executables and/or libraries for installation
install(TARGETS manipulation_planning_interface
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
## Mark executables and/or libraries for installation
install(TARGETS pathfinding_planning_interface
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# protected region user Cmake macros on begin #
# protected region user Cmake macros end #
