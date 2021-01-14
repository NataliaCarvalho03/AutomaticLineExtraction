cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(3_elimina)

find_package(PCL 1.2 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable (3_elimina 3_elimina.cpp)
target_link_libraries (3_elimina ${PCL_LIBRARIES})
