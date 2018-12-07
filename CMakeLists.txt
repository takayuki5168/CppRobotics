cmake_minimum_required(VERSION 3.0)

project(CppRobotics CXX C)

# Options
option(HOGE "hoge" ON)

# Sources and headers
file(GLOB_RECURSE MANIPULATION_SOURCES src/manipulation/*.cpp)
add_library(CppRobotics ${MANIPULATION_SOURCES})
target_include_directories(CppRobotics PUBLIC include/cpp_robotics/manipulation)
