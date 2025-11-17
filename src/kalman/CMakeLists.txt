cmake_minimum_required(VERSION 3.0.2)
project(kalman)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(Eigen3 REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  tf
)

include_directories(
  include
  SYSTEM
  ${catkin_INCLUDE_DIRS} 
  ${EIGEN3_INCLUDE_DIR}
  ${PROJECT_SOURCE_DIR}/include
 
)

add_executable(ekf src/ekf_node.cpp)
target_link_libraries(ekf 
  ${catkin_LIBRARIES}
  )

add_executable(test_odom src/test.cpp)
target_link_libraries(test_odom 
  ${catkin_LIBRARIES}
  )
