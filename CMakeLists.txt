cmake_minimum_required(VERSION 2.6 FATAL_ERROR)

project(scan_viz_demo)

find_package(PCL 1.6 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable (scan_viz_demo src/scan_viz_demo.cpp)
target_link_libraries (scan_viz_demo ${PCL_LIBRARIES})

