cmake_minimum_required(VERSION 2.8.3)
project(u06)

find_package(catkin REQUIRED roscpp std_msgs tf visualization_msgs)

catkin_package(
  CATKIN_DEPENDS message_runtime
)

add_executable(u06_trajectory src/trajectory.cpp)
target_link_libraries(u06_trajectory ${catkin_LIBRARIES})
