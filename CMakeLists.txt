cmake_minimum_required(VERSION 3.23)
project(TFE_MAPF)

set(CMAKE_CXX_STANDARD 23)

# add_library(TFE_MAPF library.cpp parser.cpp Graph.cpp astar_single.cpp searchable_pqueue.cpp Node.cpp)

# add_executable(TFE_MAPF_test test.cpp)
# target_link_libraries(TFE_MAPF_test TFE_MAPF)


add_library(TFE_MAPF library.cpp)

# For testing
add_executable(TFE_MAPF_test test.cpp library.cpp Problem.cpp Problem.h Node.cpp Node.h State.cpp State.h parser.cpp parser.h Graph.cpp Graph.h Solution.cpp Solution.h MultiAgentProblem.cpp MultiAgentProblem.h Heuristic.cpp Heuristic.h SingleAgentProblem.cpp SingleAgentProblem.h MultiAgentState.cpp MultiAgentState.h SingleAgentState.cpp SingleAgentState.h)

add_executable(TFE_MAPF_visu runner_for_visualization.cpp library.cpp Problem.cpp Problem.h Node.cpp Node.h State.cpp State.h parser.cpp parser.h Graph.cpp Graph.h Solution.cpp Solution.h MultiAgentProblem.cpp MultiAgentProblem.h Heuristic.cpp Heuristic.h SingleAgentProblem.cpp SingleAgentProblem.h MultiAgentState.cpp MultiAgentState.h SingleAgentState.cpp SingleAgentState.h)
