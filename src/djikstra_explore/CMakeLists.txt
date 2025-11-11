cmake_minimum_required(VERSION 3.0.2)
project(djikstra_explore)


find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
)

include_directories(
  include
  SYSTEM
  ${catkin_INCLUDE_DIRS} ${PROJECT_SOURCE_DIR}/include
 
)

add_executable(djikstra_explore src/djikstra.cpp)
target_link_libraries(djikstra_explore 
  ${catkin_LIBRARIES}
  )