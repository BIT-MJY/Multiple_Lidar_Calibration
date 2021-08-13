cmake_minimum_required(VERSION 2.8.3)
project(multi_lidar_calib)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-std=c++11")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  sensor_msgs
  roscpp
  rospy
  rosbag
  std_msgs
  tf
  pcl_conversions
)

# include_directories("/usr/local/include/eigen3")
set(PCL_DIR "/usr/share/pcl-1.8")
find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED)
find_package(Ceres REQUIRED)

include_directories(
  include
	${catkin_INCLUDE_DIRS} 
	${PCL_INCLUDE_DIRS}
	${EIGEN3_INCLUDE_DIR}
  ${CERES_INCLUDE_DIRS}
  )

catkin_package(
  CATKIN_DEPENDS geometry_msgs  roscpp rospy std_msgs pcl_conversions
  DEPENDS EIGEN3 PCL 
  INCLUDE_DIRS include
)




add_executable(multi_lidar_calib src/multi_lidar_calib.cpp)
target_link_libraries(multi_lidar_calib ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${CERES_LIBRARIES})







