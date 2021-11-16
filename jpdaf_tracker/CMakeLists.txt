cmake_minimum_required(VERSION 3.0.2)
project(jpdaf_tracker)

find_package(catkin REQUIRED COMPONENTS
  ddynamic_reconfigure
  roscpp
  rospy
  sensor_msgs
  std_msgs
  vision_msgs
)

find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
)

add_executable(jpdaf_tracker_node
  src/jpdaf_tracker_node.cpp
  src/jpdaf_tracker.cpp
  src/global_tracker.cpp
  src/hungarianAlg.cpp
  src/kalman.cpp
  src/local_tracker.cpp
  src/track.cpp
  src/tracker.cpp
  src/tracker_param.cpp
)

target_link_libraries(jpdaf_tracker_node
  ${catkin_LIBRARIES}
  ${OpenCV_LIBS}
  ${Boost_LIBRARIES}
)
