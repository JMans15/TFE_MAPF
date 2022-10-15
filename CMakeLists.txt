cmake_minimum_required(VERSION 3.23)
project(TFE_MAPF)

set(CMAKE_CXX_STANDARD 14)

add_library(TFE_MAPF library.cpp)

# For testing
add_executable(TFE_MAPF_test test.cpp library.cpp)
