cmake_minimum_required(VERSION 3.23)
project(TFE_MAPF)

set(CMAKE_CXX_STANDARD 14)

add_library(TFE_MAPF library.cpp astar.cpp astar.h searchable_pqueue.cpp searchable_pqueue.h searchable_queue.cpp searchable_queue.h)

# For testing
add_executable(TFE_MAPF_test test.cpp)
target_link_libraries(TFE_MAPF_test TFE_MAPF)

add_executable(TFE_MAPF_test_astar test_astar.cpp)
target_link_libraries(TFE_MAPF_test_astar TFE_MAPF)