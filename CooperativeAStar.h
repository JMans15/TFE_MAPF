//
// Created by Arthur Mahy on 27/02/2023.
//

#ifndef TFE_MAPF_COOPERATIVEASTAR_H
#define TFE_MAPF_COOPERATIVEASTAR_H
#include "MultiAgentProblem.h"
#include "AStar.h"
#include "SingleAgentSpaceTimeProblem.h"

// Cooperative A* search - not optimal
// Only for multi agent problem
// Consists of (numberOfAgents) single agent A* searches
// where we update a reservation table with the several (position, timestep) of the planned paths
// These (position, timestep) will be avoided during the searches of the remaining agents
// typeOfHeuristic is the heuristic for the single agent A* search : Manhattan (Cooperative A*) or OptimalDistance (Hierarchical Cooperative A* with a Reverse Resumable A*)
// The cooperative A* search could be modified / improved by modifying the order of the agents (see prioritized planning Latombe 1991)
class CooperativeAStar {
public :
    CooperativeAStar(MultiAgentProblem* problem, TypeOfHeuristic typeOfHeuristic);
    Solution getSolution();
private:
    Solution solution;
};


#endif //TFE_MAPF_COOPERATIVEASTAR_H
