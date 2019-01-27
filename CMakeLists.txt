cmake_minimum_required(VERSION 3.0)

project(CppRobotics CXX C)

# Compiler
include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=gnu++14" COMPILER_SUPPORTS_CXX14)
if(COMPILER_SUPPORTS_CXX14)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=gnu++14")
else()
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=gnu++11")
endif()

# Eigen
set(EIGEN_INCLUDE_DIR ${CMAKE_CURRENT_LIST_DIR}/external/Eigen)
include_directories(SYSTEM ${EIGEN_INCLUDE_DIR})

# qpOASES
set(QP_OASES_INCLUDE_DIR ${CMAKE_CURRENT_LIST_DIR}/external/qpOASES/include)
file(GLOB QP_OASES_SOURCE ${CMAKE_CURRENT_LIST_DIR}/external/qpOASES/src/*.cpp)
add_library(QP_OASES STATIC ${QP_OASES_SOURCE})
include_directories(SYSTEM ${QP_OASES_INCLUDE_DIR})

# OpenGL
find_package(OpenGL REQUIRED)
find_package(GLUT REQUIRED)
include_directories(${OPENGL_INCLUDE_DIRS} ${GLUT_INCLUDE_DIRS})

# CppRobotics
file(GLOB_RECURSE LIB_SOURCES ${CMAKE_CURRENT_LIST_DIR}/lib/src/core/*.cpp)
include_directories(LIB_HEADERS ${CMAKE_CURRENT_LIST_DIR}/lib/include/)

# main
file(GLOB MAIN_SOURCE main.cpp) 
add_executable(main
  ${MAIN_SOURCE}
  ${LIB_SOURCES})
target_link_libraries(main
  ${OPENGL_LIBRARIES}
  QP_OASES
  ${GLUT_LIBRARY}
  pthread)
