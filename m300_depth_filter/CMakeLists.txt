cmake_minimum_required(VERSION 2.8)
project(m300_depth_filter)

set(CMAKE_CXX_FLAGS_DEBUG
    "$ENV{CXXFLAGS} -std=c++11 -O0 -Wall -ggdb -fopenmp -pthread")
set(CMAKE_CXX_FLAGS_RELEASE "$ENV{CXXFLAGS} -std=c++11 -O3 -fopenmp -pthread")

set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

include_directories(${PROJECT_SOURCE_DIR}/include)

# Eigen
include_directories("/usr/include/eigen3")

# OpenCV
find_package(OpenCV 4 REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

# pangolin
find_package(Pangolin REQUIRED)
include_directories(${Pangolin_INCLUDE_DIRS})

# Sophus
find_package(Sophus REQUIRED)
include_directories(${Sophus_INCLUDE_DIRS})

# fmt
find_package(fmt REQUIRED)
include_directories(${fmt_INCLUDE_DIRS})

# yaml
find_package(yaml-cpp REQUIRED)

set(THIRD_PARTY_LIBS
    ${OpenCV_LIBS}
    ${Pangolin_LIBRARIES}
    GL
    GLU
    GLEW
    glut
    yaml-cpp
    ${Sophus_LIBRARIES}
    fmt::fmt)

add_subdirectory(src)
