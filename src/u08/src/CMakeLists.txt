cmake_minimum_required(VERSION 2.8)
project( u08 )
find_package( OpenCV REQUIRED )
add_executable( astar astar.cpp )
target_link_libraries( astar ${OpenCV_LIBS} )