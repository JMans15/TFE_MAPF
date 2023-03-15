cmake_minimum_required(VERSION 3.23)
project(TFE_MAPF)

set(CMAKE_CXX_STANDARD 23)

set(Boost_USE_STATIC_LIBS OFF)
set(Boost_USE_MULTITHREADED ON)
set(Boost_USE_STATIC_RUNTIME OFF)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3 -Wall -Wextra")

find_package(Boost REQUIRED)

include_directories(${Boost_INCLUDE_DIRS})

# define library
add_library(TFE_MAPF Problem.h Node.h State.h Parser.cpp Parser.h Graph.cpp Graph.h Solution.cpp Solution.h MultiAgentProblem.cpp MultiAgentProblem.h HeuristicManhattan.h HeuristicOptimal.h SingleAgentProblem.cpp SingleAgentProblem.h MultiAgentState.cpp MultiAgentState.h SingleAgentState.h SingleAgentSpaceTimeProblem.cpp SingleAgentSpaceTimeProblem.h SingleAgentSpaceTimeState.cpp SingleAgentSpaceTimeState.h AStar.h CooperativeAStar.h ReverseResumableAStar.cpp ReverseResumableAStar.h IndependentDetection.cpp IndependentDetection.h Boost_tests/test1.cpp)

# first test
add_executable(TFE_MAPF_test test.cpp)
target_link_libraries(TFE_MAPF_test TFE_MAPF)
target_link_libraries(TFE_MAPF_test ${Boost_LIBRARIES})

# runner with args
add_executable(TFE_MAPF_visu runner_for_visualization.cpp)
target_link_libraries(TFE_MAPF_visu TFE_MAPF)
target_link_libraries(TFE_MAPF_visu ${Boost_LIBRARIES})

add_subdirectory(Boost_tests)