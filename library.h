#ifndef TFE_MAPF_LIBRARY_H
#define TFE_MAPF_LIBRARY_H

#include "Problem.h"
#include "MultiAgentProblem.h"
#include "SingleAgentSpaceTimeProblem.h"
#include <vector>
#include "State.h"
#include "Solution.h"
using namespace std;

// Basic A* search
// Can be applied for multi agent (operator decomposition) and single agent problems
Solution aStarSearch(Problem* problem, TypeOfHeuristic typeOfHeuristic);

// Cooperative A* search
// Only for multi agent problem
// Consists of (numberOfAgents) single agent A* searches
// where we update a reservation table with the several (position, timestep) of the planned paths
// These (position, timestep) will be avoided during the searches of the remaining agents
// The heuristic for the single agent A* search is the Manhattan distance
// The cooperative A* search could be modified / improved by modifying the order of the agents (see prioritized planning Latombe 1991)
Solution cooperativeAStarSearch(MultiAgentProblem* problem, int verbose = 1);

#endif //TFE_MAPF_LIBRARY_H
