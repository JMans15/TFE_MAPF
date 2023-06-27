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
add_library(TFE_MAPF AStarProblems/AStarProblem.h Solvers/AStar/Node.h States/State.h GraphParser/Parser.cpp GraphParser/Parser.h GraphParser/Graph.cpp GraphParser/Graph.h Solution/Solution.cpp Solution/Solution.h Problems/MultiAgentProblem.cpp Problems/MultiAgentProblem.h Problems/SingleAgentProblem.cpp Problems/SingleAgentProblem.h Heuristics/HeuristicManhattan.h Heuristics/HeuristicOptimal.h States/ODMultiAgentSpaceTimeState.cpp States/ODMultiAgentSpaceTimeState.h States/SingleAgentState.h States/SingleAgentSpaceTimeState.cpp States/SingleAgentSpaceTimeState.h Solvers/AStar/AStar.h Solvers/SuboptimalSolver/CooperativeAStar.h Solvers/AStar/ReverseResumableAStar.cpp Solvers/AStar/ReverseResumableAStar.h Boost_tests/test1.cpp Solvers/ID/Group.h Constraints/VertexConstraint.h Constraints/EdgeConstraint.h Solvers/CBS/ConflictBasedSearch.cpp Solvers/CBS/ConflictBasedSearch.h Solvers/CBS/ConflictTreeNode.h Solvers/CBS/AgentConflict.h Solvers/ID/GroupConflict.h Solvers/ID/SimpleIndependenceDetection.cpp Solvers/ID/SimpleIndependenceDetection.h Solvers/ID/IndependenceDetection.cpp Solvers/ID/IndependenceDetection.h Solvers/ID/AlreadyConflictedBeforeSet.h States/StandardMultiAgentSpaceTimeState.cpp States/StandardMultiAgentSpaceTimeState.h Constraints/ConstraintsSet.h States/ODMultiAgentState.cpp States/ODMultiAgentState.h Solvers/AStar/GeneralAStar.h AStarProblems/SingleAgentAStarProblem.cpp AStarProblems/SingleAgentAStarProblem.h AStarProblems/SingleAgentAStarProblemWithConstraints.cpp AStarProblems/SingleAgentAStarProblemWithConstraints.h AStarProblems/ODMultiAgentAStarProblemWithConstraints.cpp AStarProblems/ODMultiAgentAStarProblemWithConstraints.h AStarProblems/ODMultiAgentAStarProblem.cpp AStarProblems/ODMultiAgentAStarProblem.h AStarProblems/StandardMultiAgentAStarProblemWithConstraints.cpp AStarProblems/StandardMultiAgentAStarProblemWithConstraints.h AStarProblems/StandardMultiAgentAStarProblem.cpp AStarProblems/StandardMultiAgentAStarProblem.h States/StandardMultiAgentState.cpp States/StandardMultiAgentState.h Problems/Problem.h)

# first test
add_executable(TFE_MAPF_test test.cpp)
target_link_libraries(TFE_MAPF_test TFE_MAPF)
target_link_libraries(TFE_MAPF_test ${Boost_LIBRARIES})

# runner with args
add_executable(TFE_MAPF_visu Visualisation/visualization-runner.cpp)
target_link_libraries(TFE_MAPF_visu TFE_MAPF)
target_link_libraries(TFE_MAPF_visu ${Boost_LIBRARIES})

add_subdirectory(Boost_tests)