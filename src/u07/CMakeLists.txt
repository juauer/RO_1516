cmake_minimum_required(VERSION 2.8.3)
project(u07)

find_package(catkin REQUIRED roscpp tf std_msgs visualization_msgs nav_msgs ackermann_msgs)

catkin_package(
  CATKIN_DEPENDS message_runtime
)

add_executable(u07_controller src/controller.h src/controller.cpp)
target_link_libraries(u07_controller ${catkin_LIBRARIES})
