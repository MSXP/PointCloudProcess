cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(pointcloud_process)

find_package(PCL 1.8 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
include_directories(include)

link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

aux_source_directory(. DIR_SRCS)
add_executable (pointcloud_process ${DIR_SRCS})
target_link_libraries (pointcloud_process ${PCL_LIBRARIES})
