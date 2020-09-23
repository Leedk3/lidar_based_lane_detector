cmake_minimum_required(VERSION 3.0.2)
project(lidar_based_lane_detector)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  geometry_msgs
  nav_msgs
  pcl_ros
  pcl_conversions
  roscpp
  rospy
  std_msgs
  tf
)

find_package(Boost REQUIRED COMPONENTS system)
find_package(OpenCV REQUIRED QUIET)
find_package(PCL 1.7 REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

message(STATUS "PCL_INCLUDE_DIRS:" ${PCL_INCLUDE_DIRS})
message(STATUS "PCL_LIBRARY_DIRS:" ${PCL_LIBRARY_DIRS})
message(STATUS "PCL_DEFINITIONS:" ${PCL_DEFINITIONS})


catkin_package(
  INCLUDE_DIRS include
  LIBRARIES lidar_based_lane_detector
  CATKIN_DEPENDS cv_bridge geometry_msgs nav_msgs pcl_ros roscpp rospy std_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
	${PCL_INCLUDE_DIRS}
	${OpenCV_INCLUDE_DIRS}
)

## Declare a C++ library
add_library(${PROJECT_NAME}_lib
  src/lidar_based_lane_detector.cpp
)
add_dependencies(${PROJECT_NAME}_lib ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME}_lib
   ${catkin_LIBRARIES}
   ${PCL_LIBRARIES}
   ${Boost_LIBRARIES}
   ${OpenCV_LIBRARIES}
)

add_executable(${PROJECT_NAME}_node src/lidar_based_lane_detector_node.cpp)

add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(${PROJECT_NAME}_node
  ${catkin_LIBRARIES}
  ${PROJECT_NAME}_lib
  ${PCL_LIBRARIES}
  ${Boost_LIBRARIES}
  ${OpenCV_LIBRARIES}
)

install(
  TARGETS ${PROJECT_NAME}_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(
  DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)