cmake_minimum_required(VERSION 2.8)
project(Theta_ROS)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
set(SOURCE_FILES
		main.cpp
        map.cpp
        theta.cpp
		structs.h
        map.h
        theta.h
        )
add_executable(Theta_ROS ${SOURCE_FILES})