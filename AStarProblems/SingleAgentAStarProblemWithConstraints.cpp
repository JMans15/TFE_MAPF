//
// Created by Arthur Mahy on 11/06/2023.
//

#include "SingleAgentAStarProblemWithConstraints.h"

#include <utility>

SingleAgentAStarProblemWithConstraints::SingleAgentAStarProblemWithConstraints(std::shared_ptr<SingleAgentProblem> problem)
        : AStarProblem<SingleAgentSpaceTimeState>()
        , problem(std::move(problem))
{}

std::shared_ptr<SingleAgentSpaceTimeState> SingleAgentAStarProblemWithConstraints::getStartState() const {
    return std::make_shared<SingleAgentSpaceTimeState>(problem->getStart(), problem->getStartTime());
}

bool SingleAgentAStarProblemWithConstraints::isGoalState(std::shared_ptr<SingleAgentSpaceTimeState> state) const {
    return state->getPosition() == problem->getTarget();
}

std::vector<std::tuple<std::shared_ptr<SingleAgentSpaceTimeState>, int, int>> SingleAgentAStarProblemWithConstraints::getSuccessors(std::shared_ptr<SingleAgentSpaceTimeState> state) const {
    std::vector<std::tuple<std::shared_ptr<SingleAgentSpaceTimeState>, int, int>> successors;
    int position = state->getPosition();
    int t = state->getTimestep();
    int nextT = t+1;
    int costMovement;
    int costWait;
    if (problem->getObjFunction() == Fuel) {
        costMovement = 1;
        costWait = 0;
    } else { // obj_function==Makespan
        costMovement = 1;
        costWait = 1;
    }

    // Move
    for (int newPosition : problem->getGraph()->getNeighbors(position)){
        if (problem->okForConstraints(position, newPosition, nextT)){
            auto successor = std::make_shared<SingleAgentSpaceTimeState>(newPosition, nextT);
            successors.emplace_back(successor, costMovement, problem->numberOfViolations(position, newPosition, nextT));
        }
    }

    // Wait
    if (problem->okForConstraints(position, nextT)){
        auto successor = std::make_shared<SingleAgentSpaceTimeState>(position, nextT);
        successors.emplace_back(successor, costWait, problem->numberOfViolations(position, nextT));
    }

    return successors;
}

std::unordered_map<int, std::vector<int>> SingleAgentAStarProblemWithConstraints::getPositions(std::vector<std::shared_ptr<SingleAgentSpaceTimeState>> states) const {
    std::unordered_map<int, std::vector<int>> positions;
    for (auto state : states) {
        positions[problem->getAgentId()].push_back(state->getPosition());
    }
    return positions;
}

int SingleAgentAStarProblemWithConstraints::getMaxCost() const {
    return problem->getMaxCost();
}

int SingleAgentAStarProblemWithConstraints::getStartTime() const {
    return problem->getStartTime();
}

std::shared_ptr<SingleAgentProblem> SingleAgentAStarProblemWithConstraints::getProblem() {
    return problem;
}
