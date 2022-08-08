cmake_minimum_required(VERSION 2.8.3)
add_compile_options(-std=c++14)
project(pharos_v2x)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-std=c++14")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g -pthread")

find_package(catkin REQUIRED COMPONENTS
  # ROS MAIN
  tf
  roscpp
  rospy
  

  # ROS MES
  std_msgs
  sensor_msgs
  geometry_msgs
  ublox_msgs
  nav_msgs
  message_generation
  
  pharos_msgs
)




generate_messages(
  DEPENDENCIES
  geometry_msgs
  std_msgs
  ublox_msgs
  nav_msgs
  sensor_msgs
)

catkin_package(

  CATKIN_DEPENDS
  std_msgs
  nav_msgs
  ublox_msgs
  geometry_msgs
  sensor_msgs
  message_runtime
  message_generation
  
)

# include directories
include_directories(
	${catkin_INCLUDE_DIRS}
	
	
)

# link directories
link_directories(
	#include
	
  
)

###########
## Build ##
###########


# Mapping Optimization
add_executable(${PROJECT_NAME}_ros_to_obu src/ros_to_obu.cpp)
add_dependencies(${PROJECT_NAME}_ros_to_obu ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_generate_messages_cpp)
target_compile_options(${PROJECT_NAME}_ros_to_obu PRIVATE )
target_include_directories(${PROJECT_NAME}_ros_to_obu PUBLIC )
target_link_libraries(${PROJECT_NAME}_ros_to_obu ${catkin_LIBRARIES} )


