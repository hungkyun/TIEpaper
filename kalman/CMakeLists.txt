project(lidar)
cmake_minimum_required(VERSION 3.6)
add_definitions(-std=c++17)


find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  std_msgs
  pcl_ros
  pcl_conversions
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)
set(SOURCES src/Kalman.cpp include/Kalman.hpp)

#OpenCV
find_package(OpenCV REQUIRED)
include_directories(
  ${OpenCV_INCLUDE_DIRS}
  ./include
)
FIND_PACKAGE( PCL REQUIRED COMPONENTS common io visualization )
catkin_package()

link_directories(${PCL_LIBRARY_DIRS} ${OpenCV_LIBRARY_DIRS})

add_executable(kalman ${SOURCES})
target_link_libraries(kalman
   ${catkin_LIBRARIES}
   ${PCL_LIBRARIES}
   ${OpenCV_LIBS}
)
