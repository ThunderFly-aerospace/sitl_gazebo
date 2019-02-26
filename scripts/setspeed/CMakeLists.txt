cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

# Find Gazebo
find_package(gazebo REQUIRED)
include_directories(${GAZEBO_INCLUDE_DIRS})
link_directories(${GAZEBO_LIBRARY_DIRS})
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GAZEBO_CXX_FLAGS}")

# Build our plugins
#add_library(control_plugin SHARED control_plugin.cpp)
#add_library(simple_rotor_plugin SHARED simple_rotor_plugin.cpp)
#target_link_libraries(control_plugin ${GAZEBO_LIBRARIES})

add_executable(setspeed setspeed.cpp)
target_link_libraries(setspeed ${GAZEBO_LIBRARIES})
