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
add_library(TFE_MAPF Problems/Problem.h Solvers/Node.h States/State.h GraphParser/Parser.cpp GraphParser/Parser.h GraphParser/Graph.cpp GraphParser/Graph.h Solution/Solution.cpp Solution/Solution.h Problems/MultiAgentProblemWithConstraints.cpp Problems/MultiAgentProblemWithConstraints.h Heuristics/HeuristicManhattan.h Heuristics/HeuristicOptimal.h Problems/SingleAgentProblem.cpp Problems/SingleAgentProblem.h States/MultiAgentState.cpp States/MultiAgentState.h States/SingleAgentState.h Problems/SingleAgentProblemWithConstraints.cpp Problems/SingleAgentProblemWithConstraints.h States/SingleAgentSpaceTimeState.cpp States/SingleAgentSpaceTimeState.h Solvers/AStar.h Solvers/CooperativeAStar.h Solvers/ReverseResumableAStar.cpp Solvers/ReverseResumableAStar.h Solvers/IndependenceDetection.cpp Solvers/IndependenceDetection.h Boost_tests/test1.cpp Solvers/SimpleIndependenceDetection.cpp Solvers/SimpleIndependenceDetection.h Solvers/Group.h Constraints/VertexConstraint.h Constraints/EdgeConstraint.h Solvers/ConflictBasedSearch.cpp Solvers/ConflictBasedSearch.h)

# first test
add_executable(TFE_MAPF_test test.cpp)
target_link_libraries(TFE_MAPF_test TFE_MAPF)
target_link_libraries(TFE_MAPF_test ${Boost_LIBRARIES})

# runner with args
add_executable(TFE_MAPF_visu runner_for_visualization.cpp)
target_link_libraries(TFE_MAPF_visu TFE_MAPF)
target_link_libraries(TFE_MAPF_visu ${Boost_LIBRARIES})

add_subdirectory(Boost_tests)