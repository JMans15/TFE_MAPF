//
// Created by Arthur Mahy on 27/02/2023.
//

#ifndef TFE_MAPF_COOPERATIVEASTAR_H
#define TFE_MAPF_COOPERATIVEASTAR_H

#include "AStar.h"
#include "MultiAgentProblem.h"
#include "SingleAgentSpaceTimeProblem.h"

// Cooperative A* search - not optimal
// Only for multi agent problem
//
// Consists of (numberOfAgents) single agent A* searches
// where we update a reservation table with the several (position, timestep) points of the planned paths
// These (position, timestep) points will be avoided during the searches of the remaining agents
//
// typeOfHeuristic is the heuristic for the single agent A* searches : Manhattan (Cooperative A*) or OptimalDistance (Hierarchical Cooperative A* with a Reverse Resumable A*)
//
// The search has 3 drawbacks here:
// - Agents don't cooperate once they have a planned path (for example, problem if narrow corridor)
//   Ideally, agent should allow to move of its target to allow others to pass.
// - It depends on the order of the agents (see prioritized planning Latombe 1991)
// - To avoid edge conflict, we made the following decision:
//   when we know that a planned path has a (p,t) point, we put (p,t) in the reservation AND ALSO (p,t+1) !
//   We know this removes possible solutions
// Could be improved with WINDOWED Hierarchical Cooperative A*
class CooperativeAStar {
public:
    CooperativeAStar(std::shared_ptr<MultiAgentProblem> problem, TypeOfHeuristic typeOfHeuristic);
    Solution solve();
private:
    std::shared_ptr<MultiAgentProblem> problem;
    TypeOfHeuristic typeOfHeuristic;
};


#endif //TFE_MAPF_COOPERATIVEASTAR_H
