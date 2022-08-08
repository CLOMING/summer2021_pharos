cmake_minimum_required(VERSION 2.8.3)
project(pharos_utm_odometry CXX)

add_compile_options(-std=c++11)
find_package(cmake_modules REQUIRED)
FIND_PACKAGE(PCL REQUIRED)
find_package(Eigen REQUIRED)
include_directories(${Eigen_INCLUDE_DIRS})
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

############
## Catkin ##
############

find_package(catkin REQUIRED
  message_generation
  message_filters
  roscpp
  std_msgs
  nav_msgs
  sensor_msgs
  tf
  sbg_driver
)

#######################################
## Declare ROS messages and services ##
#######################################

add_message_files(
  FILES
    GPSStatus.msg
    GPSFix.msg
)

generate_messages(DEPENDENCIES std_msgs)

###################################################
## Declare things to be passed to other projects ##
###################################################

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS message_runtime
)

###########
## Build ##
###########

include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include ${catkin_INCLUDE_DIRS})
# set_target_properties(${PROJECT_NAME}/utm_odometry_node PROPERTIES OUTPUT_NAME "utm_odometry_node")
# target_link_libraries(${PROJECT_NAME}/utm_odometry_node ${catkin_LIBRARIES} ${PCL_LIBRARIES})
# add_dependencies(${PROJECT_NAME}/utm_odometry_node ${PROJECT_NAME}_generate_messages_cpp)

add_executable(pharos_utm_odometry_node src/pharos_utm_odometry_node.cpp)
target_link_libraries(pharos_utm_odometry_node ${catkin_LIBRARIES} ${PCL_LIBRARIES})
add_dependencies(pharos_utm_odometry_node ${PROJECT_NAME}_generate_messages_cpp)

add_executable(kcity_utm_odometry_node src/kcity_utm_odometry_node.cpp)
target_link_libraries(kcity_utm_odometry_node ${catkin_LIBRARIES} ${PCL_LIBRARIES})
add_dependencies(kcity_utm_odometry_node ${PROJECT_NAME}_generate_messages_cpp)

add_executable(sbg_utm_odometry_node src/sbg_utm_odometry_node.cpp)
target_link_libraries(sbg_utm_odometry_node ${catkin_LIBRARIES} ${PCL_LIBRARIES})
add_dependencies(sbg_utm_odometry_node ${PROJECT_NAME}_generate_messages_cpp)

#############
## Install ##
#############

# install(TARGETS ${PROJECT_NAME}/utm_odometry_node
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

# install(DIRECTORY include/${PROJECT_NAME}/ 
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
# )
