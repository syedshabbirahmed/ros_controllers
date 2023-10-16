cmake_minimum_required(VERSION 3.10)
project(cubic_polynomial_planner)

## Use C++14
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

## By adding -Wall and -Werror, the compiler does not ignore warnings anymore,
## enforcing cleaner code.
add_definitions(-Wall -Werror)

## Find catkin macros and libraries
find_package(catkin REQUIRED
  COMPONENTS
    roscpp
    std_msgs
    genmsg
    geometry_msgs
)

# Declare ROS service
add_service_files(
	FILES 
		move_robot.srv
)

generate_messages(DEPENDENCIES )

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  INCLUDE_DIRS
    #include   
  LIBRARIES
    ${PROJECT_NAME}
  CATKIN_DEPENDS
    roscpp
    std_msgs
    geometry_msgs
  DEPENDS 
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
	#include
	${catkin_INCLUDE_DIRS}
)

add_executable(			${PROJECT_NAME}_server src/server.cpp)
target_link_libraries(	${PROJECT_NAME}_server ${catkin_LIBRARIES})
add_dependencies(${PROJECT_NAME}_server ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})