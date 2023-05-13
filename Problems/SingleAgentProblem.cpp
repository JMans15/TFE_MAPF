//
// Created by Arthur Mahy on 16/01/2023.
//

#include "SingleAgentProblem.h"

SingleAgentProblem::SingleAgentProblem(std::shared_ptr<Graph> graph, int start, int target, int agentId, int maxCost)
    : Problem(graph, 1, maxCost)
    , start(start)
    , target(target)
    , agentId(agentId)
{
    LOG("==== Single Agent Problem ====");
    LOG("Id of the agent : " << agentId);
    impossible = false;
    LOG("Start position of the agent : " << start);
    if (graph->getNeighbors(start).empty()){
        LOG("   The start position is unreachable.");
        impossible = true;
    }
    LOG("Target position of the agent : " << target);
    if (graph->getNeighbors(target).empty()){
        LOG("   The target position is unreachable.");
        impossible = true;
    }
    if (maxCost!=INT_MAX){
        LOG("The solution of this problem must have a cost inferior or equal to " << maxCost);
    }
    LOG(" ");
}

std::shared_ptr<SingleAgentState> SingleAgentProblem::getStartState() const {
    return std::make_shared<SingleAgentState>(start);
}

std::shared_ptr<SingleAgentState> SingleAgentProblem::getGoalState() const {
    return std::make_shared<SingleAgentState>(target);
}

bool SingleAgentProblem::isGoalState(std::shared_ptr<SingleAgentState> state) const {
    return state->getPosition() == target;
}

std::vector<std::tuple<std::shared_ptr<SingleAgentState>, int, int>> SingleAgentProblem::getSuccessors(std::shared_ptr<SingleAgentState> state) const {
    std::vector<std::tuple<std::shared_ptr<SingleAgentState>, int, int>> successors;

    for (int newPosition : graph->getNeighbors(state->getPosition())){
        auto successor = std::make_shared<SingleAgentState>(newPosition);
        successors.emplace_back(successor, 1, 0);
    }

    return successors;
}

std::unordered_map<int, std::vector<int>> SingleAgentProblem::getPositions(std::vector<std::shared_ptr<SingleAgentState>> states) const {
    std::unordered_map<int, std::vector<int>> positions;
    for (auto state : states) {
        positions[agentId].push_back(state->getPosition());
    }
    return positions;
}

std::vector<int> SingleAgentProblem::getAgentIds() const {
    return {agentId};
}

const int SingleAgentProblem::getStart() const {
    return start;
}

const int SingleAgentProblem::getTarget() const {
    return target;
}

bool SingleAgentProblem::isImpossible() const {
    return impossible;
}
