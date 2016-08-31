cmake_minimum_required(VERSION 2.8)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
#set(ROS_BUILD_TYPE RelWithDebInfo)

rosbuild_init()

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)
include_directories(${PROJECT_SOURCE_DIR}/src)

#set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
#list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake)
#find_package(TBB REQUIRED)
#include_directories(${TBB_INCLUDE_DIRS})

#FIND_PATH(TBB_INCLUDE_DIR tbb/tbb.h 
#  /usr/include
#  )

#find_library(TBB_LIBRARY tbb  
#  /usr/lib
#  )

# Eigen3 library
list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake)
find_package(Eigen REQUIRED)
include_directories(${Eigen_INCLUDE_DIRS})

include_directories(${PROJECT_SOURCE_DIR}/thirdparty/Sophus)

#set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
#list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake)
#find_package(TBB REQUIRED)
#include_directories(${TBB_INCLUDE_DIRS})

# dynamic reconfigure
rosbuild_find_ros_package(dynamic_reconfigure)
include(${dynamic_reconfigure_PACKAGE_PATH}/cmake/cfgbuild.cmake)
gencfg()

# VTK library
find_package(VTK REQUIRED)
include_directories(${VTK_INCLUDE_DIRS})

# OpenCV Library
find_package(OpenCV)
include_directories(${OpenCV_INCLUDE_DIRS})

# SSE flags
rosbuild_check_for_sse()

set(CMAKE_CXX_FLAGS_RELWITHDEBINFO
   "${SSE_FLAGS} -O3 -march=native"
)
#uncomment if you have defined messages
#rosbuild_genmsg()
#uncomment if you have defined services
#rosbuild_gensrv()

#common commands for building c++ executables and libraries
#rosbuild_add_library(${PROJECT_NAME} src/example.cpp)
#target_link_libraries(${PROJECT_NAME} another_library)
#rosbuild_add_boost_directories()
#rosbuild_link_boost(${PROJECT_NAME} thread)

# Set source files
set(SOURCE_FILES
  src/dvo_ros/camera_base.cpp
  src/dvo_ros/camera_dense_tracking.cpp
  #${PROJECT_SOURCE_DIR}/src/dvo_ros/camera_tracker_nodelet.cpp
  src/dvo_core/dense_tracking.cpp
  src/dvo_core/dense_tracking_config.cpp
  src/dvo_core/interpolation.cpp
  src/dvo_core/intrinsic_matrix.cpp
  src/dvo_core/least_squares.cpp
  src/dvo_core/math_sse.cpp
  src/dvo_core/rgbd_image.cpp
  src/dvo_core/rgbd_image_sse.cpp
  src/dvo_core/surface_pyramid.cpp
  src/dvo_core/weight_calculation.cpp
  src/util/histogram.cpp
  src/util/id_generator.cpp
  src/visualization/async_point_cloud_builder.cpp
  src/visualization/camera_trajectory_visualizer.cpp
  src/visualization/pcl_camera_trajetory_visualizer.cpp
  src/visualization/point_cloud_aggregator.cpp
  src/visualization/ros_camera_trajectory_visualizer.cpp
  src/visualization/visualizer.cpp
)

rosbuild_add_library(fast SHARED ${SOURCE_FILES})

target_link_libraries(fast tbb boost_thread)  
rosbuild_add_executable(fast_slam src/main.cpp)

target_link_libraries(fast_slam fast) 

