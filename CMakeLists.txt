cmake_minimum_required(VERSION 2.8 FATAL_ERROR)
project(move_box_gazebo_ros)

find_package(catkin REQUIRED COMPONENTS 
  roscpp 
  geometry_msgs
  )
find_package(gazebo REQUIRED)
include_directories(include ${GAZEBO_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS})
link_directories(${GAZEBO_LIBRARY_DIRS})

find_package(gazebo REQUIRED)
find_package(Boost REQUIRED COMPONENTS thread)
# set (CMAKE_CXX_FLAGS "-std=c++11")
add_definitions(-std=c++11)
list(APPEND CMAKE_CXX_FLAGS "${GAZEBO_CXX_FLAGS} -O2")


add_library(move_box_gazebo_ros SHARED src/move_box.cc)
target_link_libraries(move_box_gazebo_ros ${GAZEBO_LIBRARIES} ${catkin_LIBRARIES} ${Boost_LIBRARIES})
  
