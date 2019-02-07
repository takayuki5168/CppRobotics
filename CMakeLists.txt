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

# OSQP
set(IS_LINUX ON)
set(CMAKE_C_STANDARD_LIBRARIES "${CMAKE_C_STANDARD_LIBRARIES} -lrt -ldl")
option(ENABLE_MKL_PARDISO "Enable MKL Pardiso solver" ON)
set(OSQP_INCLUDE_DIR ${CMAKE_CURRENT_LIST_DIR}/external/OSQP/include)
include_directories(SYSTEM ${OSQP_INCLUDE_DIR})
file(GLOB osqp_src ${CMAKE_CURRENT_LIST_DIR}/external/OSQP/src/*.c)

# OSQP/lin_sys
add_subdirectory(${CMAKE_CURRENT_LIST_DIR}/external/OSQP)
#add_subdirectory(${CMAKE_CURRENT_LIST_DIR}/external/OSQP/lin_sys)
#add_library (osqpstatic STATIC ${osqp_src} ${osqp_headers} ${linsys_solvers})
#target_include_directories(osqpstatic PRIVATE ${osqp_headers} ${linsys_solvers_includes})

# linenoise
set(LINENOISE_INCLUDE_DIR ${CMAKE_CURRENT_LIST_DIR}/external/linenoise)
include_directories(SYSTEM ${LINENOISE_INCLUDE_DIR})
file(GLOB LINENOISE_SOURCE ${CMAKE_CURRENT_LIST_DIR}/external/linenoise/*.c)
add_library(LINENOISE STATIC ${LINENOISE_SOURCE})

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
  osqpstatic
  dl
  LINENOISE  
  ${GLUT_LIBRARY}
  pthread)
