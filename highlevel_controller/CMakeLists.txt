cmake_minimum_required(VERSION 3.10)
project(highlevel_controller)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

#add_definitions(-Wall -Werror) 					# make sure this is not used

#find_package(PkgConfig) 							# find pkg_config
#pkg_search_module(Eigen3 REQUIRED eigen3)			# use pkg_config to find eigen3


## Find catkin macros and libraries
find_package(catkin 
	REQUIRED
	COMPONENTS
		roscpp
		geometry_msgs
		std_msgs
    	genmsg
		actionlib_msgs 								# required for new action types
		actionlib 									
)

# Declare ROS action
add_action_files(#
	DIRECTORY action#
	FILES 
		MoveTo.action		
)

find_package(Eigen3 REQUIRED)						# find eigen3
find_package(pinocchio REQUIRED)					# find pinocchio
find_package(Boost REQUIRED)
add_definitions("-DBOOST_MPL_LIMIT_LIST_SIZE=30") 	# something for pinocchio

# # Declare ROS service
# add_service_files(
# 	FILES 
# 		move_robot.srv
# )

generate_messages(DEPENDENCIES actionlib_msgs)

catkin_package(
	INCLUDE_DIRS
    	include  
    	${Eigen_INCLUDE_DIRS}
    	${PINOCCHIO_INCLUDE_DIRS}
  	LIBRARIES
  		pinocchio    	  		
  	CATKIN_DEPENDS
    	roscpp
		std_msgs
    	geometry_msgs
		actionlib    	 	    
  	DEPENDS
  	    	
)


###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
	include
	${catkin_INCLUDE_DIRS}		
	${Eigen3_INCLUDE_DIRS}		
	${PINOCCHIO_INCLUDE_DIRS}
)

add_executable(highlevel_controller_server
	src/highlevel_controller_server.cpp
	src/ActionServer.cpp
)

target_link_libraries(highlevel_controller_server
  	${catkin_LIBRARIES}  	
  	${PINOCCHIO_LIBRARIES}
)
add_dependencies(${PROJECT_NAME}_server ${catkin_EXPORTED_TARGETS} highlevel_controller_gencpp)

add_executable(highlevel_controller_client
	src/highlevel_controller_client.cpp
	src/ActionClient.cpp
)

target_link_libraries(highlevel_controller_client
  	${catkin_LIBRARIES}  	
  	${PINOCCHIO_LIBRARIES}
)
add_dependencies(${PROJECT_NAME}_client ${catkin_EXPORTED_TARGETS} highlevel_controller_gencpp)


#############
## Install ##
#############

# # Mark executables and/or libraries for installation
# install(
#   TARGETS ${PROJECT_NAME} ${PROJECT_NAME}_node
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

# Mark cpp header files for installation
install(
  DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.hpp"
)

# Mark other files for installation
install(
  DIRECTORY doc
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)