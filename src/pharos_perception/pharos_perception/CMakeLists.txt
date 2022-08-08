cmake_minimum_required(VERSION 2.8.3)
add_compile_options(-std=c++14)
project(pharos_perception)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-std=c++11")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g -pthread")

find_package(catkin REQUIRED COMPONENTS
  # ROS MAIN
  tf
  roscpp
  rospy
  cv_bridge

  # ROS MES
  std_msgs
  sensor_msgs
  geometry_msgs
  nav_msgs
  message_generation
  visualization_msgs

  # PCL LIBRARIES
  pcl_conversions
  pcl_ros

  # CUDA - P2P REG
  # ndt_gpu
  # fast_gicp
)

find_package(OpenMP REQUIRED)
find_package(PCL REQUIRED QUIET)
find_package(OpenCV REQUIRED QUIET)
# CUDA #############################################
find_package(CUDA)
include_directories(${CUDA_INCLUDE_DIRS})
link_directories(${CUDA_LIBRARY_DIRS})
add_definitions(-DCUDA_FOUND)
#add_definitions(-DUSE_VGICP_CUDA)



generate_messages(
  DEPENDENCIES
  geometry_msgs
  std_msgs
  nav_msgs
  sensor_msgs
)

catkin_package(
  INCLUDE_DIRS include
  DEPENDS PCL 

  CATKIN_DEPENDS
  std_msgs
  nav_msgs
  geometry_msgs
  sensor_msgs
  message_runtime
  message_generation
  visualization_msgs
)

# include directories
include_directories(
	include
	${catkin_INCLUDE_DIRS}
	${PCL_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
	
)

# link directories
link_directories(
	include
	${PCL_LIBRARY_DIRS}
  ${OpenCV_LIBRARY_DIRS}
  
)

###########
## Build ##
###########


# Mapping Optimization
add_executable(${PROJECT_NAME}_lidar_segmentation src/lidar_Segmentation.cpp)
add_dependencies(${PROJECT_NAME}_lidar_segmentation ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_generate_messages_cpp)
target_compile_options(${PROJECT_NAME}_lidar_segmentation PRIVATE ${OpenMP_CXX_FLAGS})
target_include_directories(${PROJECT_NAME}_lidar_segmentation PUBLIC ${CUDA_INCLUDE_DIRS})
target_link_libraries(${PROJECT_NAME}_lidar_segmentation ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBRARIES} ${OpenMP_CXX_FLAGS})

add_executable(${PROJECT_NAME}_lidar_transform src/lidar_Transform.cpp)
add_dependencies(${PROJECT_NAME}_lidar_transform ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_generate_messages_cpp)
target_compile_options(${PROJECT_NAME}_lidar_transform PRIVATE ${OpenMP_CXX_FLAGS})
target_include_directories(${PROJECT_NAME}_lidar_transform PUBLIC ${CUDA_INCLUDE_DIRS})
target_link_libraries(${PROJECT_NAME}_lidar_transform ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBRARIES} ${OpenMP_CXX_FLAGS})

add_executable(${PROJECT_NAME}_lidar_obstacle_cluster src/lidar_ClusterObstacle.cpp)
add_dependencies(${PROJECT_NAME}_lidar_obstacle_cluster ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_generate_messages_cpp)
target_compile_options(${PROJECT_NAME}_lidar_obstacle_cluster PRIVATE ${OpenMP_CXX_FLAGS})
target_include_directories(${PROJECT_NAME}_lidar_obstacle_cluster PUBLIC ${CUDA_INCLUDE_DIRS})
target_link_libraries(${PROJECT_NAME}_lidar_obstacle_cluster ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBRARIES} ${OpenMP_CXX_FLAGS})

add_executable(${PROJECT_NAME}_lidar_mapFilter src/lidar_mapFilter.cpp)
add_dependencies(${PROJECT_NAME}_lidar_mapFilter ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_generate_messages_cpp)
target_compile_options(${PROJECT_NAME}_lidar_mapFilter PRIVATE ${OpenMP_CXX_FLAGS})
target_include_directories(${PROJECT_NAME}_lidar_mapFilter PUBLIC ${CUDA_INCLUDE_DIRS})
target_link_libraries(${PROJECT_NAME}_lidar_mapFilter ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBRARIES} ${OpenMP_CXX_FLAGS})

#add_executable(${PROJECT_NAME}_lidar_ClusterObstacle_O3D src/lidar_Open3D.py)

