cmake_minimum_required(VERSION 2.8.3)
project(u11)

find_package(catkin REQUIRED roscpp tf std_msgs visualization_msgs nav_msgs ackermann_msgs geometry_msgs)

catkin_package(
  CATKIN_DEPENDS message_runtime
)

add_executable(u11_draw src/draw.cpp src/rrt.h src/rrt.cpp)
target_link_libraries(u11_draw ${catkin_LIBRARIES})

add_executable(u11_drive src/drive.cpp src/rrt.h src/rrt.cpp)
target_link_libraries(u11_drive ${catkin_LIBRARIES})
