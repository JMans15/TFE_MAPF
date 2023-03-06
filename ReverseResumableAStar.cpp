//
// Created by Arthur Mahy on 27/02/2023.
//

#include "ReverseResumableAStar.h"

ReverseResumableAStar::ReverseResumableAStar(std::shared_ptr<SingleAgentProblem> problem) : problem(problem) {
    LOG("===== Reverse Resumable A* Search ====");
    heuristic = std::make_shared<ManhattanHeuristic<SingleAgentState>>(problem->getStart(), problem->getGraph()->getWidth());
    auto start = problem->getStartState();
    auto goal = problem->getGoalState();
    fringe.insert(std::make_shared<Node<SingleAgentState>>(goal, 0, heuristic->heuristicFunction(goal)));
    distance[goal] = 0;
}

int ReverseResumableAStar::resume(const std::shared_ptr<SingleAgentState>& target) {
    while (!fringe.empty()) {
        auto it = fringe.begin();
        auto node = *it;
        auto state = node->getState();

        if (node->getCost() > distance[state]) {
            fringe.erase(it);
            continue;
        }

        if (StateEquality<SingleAgentState>()(state, target)) {
            return node->getCost();
        }

        fringe.erase(it);
        closed.insert(state);

        auto successors = problem->getSuccessors(state);
        for (auto &[successor, edgeCost] : successors) {
            auto successorCost = node->getCost() + edgeCost;
            auto it = distance.find(successor);
            if (it == distance.end() || successorCost < it->second) {
                distance[successor] = successorCost;
                auto h = heuristic->heuristicFunction(successor);
                fringe.insert(std::make_shared<Node<SingleAgentState>>(successor, successorCost, h));
            }
        }
    }
    return -1;
}

int ReverseResumableAStar::optimalDistance(int position) {
    auto state = std::make_shared<SingleAgentState>(position);
    if (closed.count(state)>0) { // if state is already in explored
        LOG("position is already explored")
        return distance[state];
    }
    LOG("position is not yet explored, we'll continue the RRA* search to visit position")
    int cost = resume(state);
    if (cost == -1) {
        return INT_MAX;
    }
    return cost;
}

std::unordered_map<std::shared_ptr<SingleAgentState>, int, StateHasher<SingleAgentState>, StateEquality<SingleAgentState>>
ReverseResumableAStar::getDistance() {
    return distance;
}
