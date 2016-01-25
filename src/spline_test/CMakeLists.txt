cmake_minimum_required(VERSION 2.8.3)
project(spline_test)

find_package(catkin REQUIRED roscpp
  ecl_geometry
  std_msgs
  tf std_msgs sensor_msgs geometry_msgs nav_msgs
)

## System dependencies are found with CMake's conventions
find_package(Boost REQUIRED COMPONENTS system filesystem)
find_package(OpenCV REQUIRED)

catkin_package(
)
###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(include)
include_directories(${Boost_INCLUDE_DIRS})
include_directories(${catkin_INCLUDE_DIRS})
include_directories(${OpenCV_INCLUDE_DIRS})
include_directories(gtest/include)


## Declare a cpp library

#add_library(spline_test src/spline_test.cpp)
add_executable(spline_test src/spline_test.cpp)
target_link_libraries(spline_test ${Boost_LIBRARIES} ${catkin_LIBRARIES})


