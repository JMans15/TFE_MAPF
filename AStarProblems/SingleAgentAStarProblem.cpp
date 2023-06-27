//
// Created by Arthur Mahy on 16/01/2023.
//

#include "SingleAgentAStarProblem.h"

#include <utility>

SingleAgentAStarProblem::SingleAgentAStarProblem(const std::shared_ptr<SingleAgentProblem>&  problem)
    : AStarProblem<SingleAgentState>()
    , problem(problem)
{}

std::shared_ptr<SingleAgentState> SingleAgentAStarProblem::getStartState() const {
    return std::make_shared<SingleAgentState>(problem->getStart());
}

std::shared_ptr<SingleAgentState> SingleAgentAStarProblem::getGoalState() const {
    return std::make_shared<SingleAgentState>(problem->getTarget());
}

bool SingleAgentAStarProblem::isGoalState(std::shared_ptr<SingleAgentState> state) const {
    return state->getPosition() == problem->getTarget();
}

std::vector<std::tuple<std::shared_ptr<SingleAgentState>, int, int>> SingleAgentAStarProblem::getSuccessors(std::shared_ptr<SingleAgentState> state) const {
    std::vector<std::tuple<std::shared_ptr<SingleAgentState>, int, int>> successors;

    for (int newPosition : problem->getGraph()->getNeighbors(state->getPosition())){
        auto successor = std::make_shared<SingleAgentState>(newPosition);
        successors.emplace_back(successor, 1, 0);
    }

    return successors;
}

std::unordered_map<int, std::vector<int>> SingleAgentAStarProblem::getPositions(std::vector<std::shared_ptr<SingleAgentState>> states) const {
    std::unordered_map<int, std::vector<int>> positions;
    for (auto state : states) {
        positions[problem->getAgentId()].push_back(state->getPosition());
    }
    return positions;
}

std::shared_ptr<SingleAgentProblem> SingleAgentAStarProblem::getProblem() {
    return problem;
}

int SingleAgentAStarProblem::getMaxCost() const {
    return problem->getMaxCost();
}

int SingleAgentAStarProblem::getStartTime() const {
    return problem->getStartTime();
}
