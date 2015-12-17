cmake_minimum_required(VERSION 2.8.3)
project( u09 )
find_package( catkin REQUIRED roscpp tf std_msgs geometry_msgs nav_msgs visualization_msgs )

catkin_package(
  CATKIN_DEPENDS message_runtime
)

add_executable( showpath src/main.cpp src/astar.h src/astar.cpp )
target_link_libraries( showpath ${catkin_LIBRARIES} )