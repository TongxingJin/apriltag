project(apriltag C CXX)

cmake_minimum_required ( VERSION 2.8 )

if (POLICY CMP0042)
cmake_policy(SET CMP0042 NEW)
endif(POLICY CMP0042)

find_package (Threads REQUIRED)

set(EXTRA_FLAGS "-Wall -Wsign-compare -g -march=native ${CMAKE_THREAD_LIBS_INIT}")
set(EXTRA_C_FLAGS "${EXTRA_FLAGS} -std=gnu99")

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${EXTRA_FLAGS}")
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${EXTRA_C_FLAGS}")

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)

if(APPLE)
  SET(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)
endif(APPLE)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  sensor_msgs
  roscpp
  rospy
  std_msgs
  pcl_ros
  tf
  # livox_ros_driver
  message_generation
  eigen_conversions
)

find_package(Eigen3 REQUIRED)
find_package(PCL 1.8 REQUIRED)

message(Eigen: ${EIGEN3_INCLUDE_DIR})

include_directories(
  ${catkin_INCLUDE_DIRS} 
  ${EIGEN3_INCLUDE_DIR}
  ${PCL_INCLUDE_DIRS}
  ${PYTHON_INCLUDE_DIRS}
  include)
  
catkin_package(
    CATKIN_DEPENDS geometry_msgs nav_msgs roscpp rospy std_msgs message_runtime
    DEPENDS EIGEN3 PCL
    INCLUDE_DIRS
  )

add_subdirectory(src)
include_directories(src src/common src/contrib)

include(FindPkgConfig)
pkg_search_module(OPENCV opencv>=2.3)

if(OPENCV_FOUND)
  include_directories(${OPENCV_INCLUDE_DIRS})
  add_subdirectory(src/opencv)
endif(OPENCV_FOUND)

