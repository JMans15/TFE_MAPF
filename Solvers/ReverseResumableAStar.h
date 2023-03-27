//
// Created by Arthur Mahy on 27/02/2023.
//

#ifndef TFE_MAPF_REVERSERESUMABLEASTAR_H
#define TFE_MAPF_REVERSERESUMABLEASTAR_H

#include "../Heuristics/HeuristicManhattan.h"
#include "Node.h"
#include "../Problems/SingleAgentProblem.h"

#include <queue>
#include <unordered_map>
#include <unordered_set>

template <class S>
class Heuristic;

// Reverse Resumable A* search
// Only for single agent problem
// Consists of a single agent search where the beginning of the search is the goal state (target position) of the problem
// Possibility to continue the search (with the resume method) even when the start state of the problem is goal tested
// (https://www.davidsilver.uk/wp-content/uploads/2020/03/coop-path-AIWisdom.pdf)
class ReverseResumableAStar {
public:
    ReverseResumableAStar(std::shared_ptr<SingleAgentProblem> problem);

    // Continues the search from target to start until state is met
    // Returns the optimal cost between goal state and state
    int resume(const std::shared_ptr<SingleAgentState>& state);

    // Returns the optimal distance (walls are taken into account) between position and target
    int optimalDistance(int position);

    std::unordered_map<std::shared_ptr<SingleAgentState>, int, StateHasher<SingleAgentState>, StateEquality<SingleAgentState>> getDistance();

private:
    std::shared_ptr<SingleAgentProblem> problem;
    std::shared_ptr<Heuristic<SingleAgentState>> heuristic;
    std::multiset<std::shared_ptr<Node<SingleAgentState>>, NodeComparator<SingleAgentState>> fringe;
    std::unordered_map<std::shared_ptr<SingleAgentState>, int, StateHasher<SingleAgentState>, StateEquality<SingleAgentState>> distance;
    std::unordered_set<std::shared_ptr<SingleAgentState>, StateHasher<SingleAgentState>, StateEquality<SingleAgentState>> closed;
};


#endif //TFE_MAPF_REVERSERESUMABLEASTAR_H
