//
// Created by Arthur Mahy on 13/02/2023.
//

#include "SingleAgentSpaceTimeProblem.h"

SingleAgentSpaceTimeProblem::SingleAgentSpaceTimeProblem(std::shared_ptr<Graph> graph, int start, int target, ObjectiveFunction objective,
                                                         int agentId, const std::set<Constraint> &setOfConstraints)
    : Problem(graph, 1)
    , start(start)
    , target(target)
    , objective(objective)
    , setOfConstraints(setOfConstraints)
    , agentId(agentId)
{
    LOG("==== Single Agent Space Time Problem ====");
    if (objective != Makespan && objective != Fuel){
        LOG("The input for the objective function is not correct.");
        LOG("So, the default objective function will be applied.");
        objective = Fuel;
    }
    if (objective == Makespan){
        LOG("Objective function : Makespan (costWait = 1)");
    } else {
        LOG("Objective function : Fuel (costWait = 0)");
    }
    LOG("Start position of the agent : " << start);
    if (graph->getNeighbors(start).empty()){
        LOG("   The start position is unreachable.");
    }
    LOG("Target position of the agent : " << target);
    if (graph->getNeighbors(target).empty()){
        LOG("   The target position is unreachable.");
    }
    if (!setOfConstraints.empty()){
        LOG("The problem has the following constraints :");
        for (Constraint constraint : setOfConstraints){
            if (constraint.agent == agentId){
                LOG("   (" << agentId << ", " << constraint.position << ", " << constraint.time << ")");
            }
        }
    }
    LOG(" ");

}

std::shared_ptr<SingleAgentSpaceTimeState> SingleAgentSpaceTimeProblem::getStartState() const {
    return std::make_shared<SingleAgentSpaceTimeState>(start, 0);
}

bool SingleAgentSpaceTimeProblem::isGoalState(std::shared_ptr<SingleAgentSpaceTimeState> state) const {
    return state->getPosition() == target;
}

// Returns true if the agent is allowed to be at position at time (according to the set of constraints of the problem)
bool SingleAgentSpaceTimeProblem::notInForbiddenPositions(int position, int time) const {
    return setOfConstraints.find({agentId, position, time}) == setOfConstraints.end();
}

std::vector<std::pair<std::shared_ptr<SingleAgentSpaceTimeState>, int>> SingleAgentSpaceTimeProblem::getSuccessors(std::shared_ptr<SingleAgentSpaceTimeState> state) const {
    std::vector<std::pair<std::shared_ptr<SingleAgentSpaceTimeState>, int>> successors;
    int position = state->getPosition();
    int t = state->getTimestep();
    int nextT = t+1;
    int costMovement;
    int costWait;
    if (objective == Fuel) {
        costMovement = 1;
        costWait = 0;
    } else { // obj_function==Makespan
        costMovement = 1;
        costWait = 1;
    }

    // Move
    for (int newPosition : graph->getNeighbors(position)){
        if (notInForbiddenPositions(newPosition, nextT)){
            auto successor = std::make_shared<SingleAgentSpaceTimeState>(newPosition, nextT);
            successors.emplace_back(successor, costMovement);
        }
    }

    // Wait
    if (notInForbiddenPositions(position, nextT)){
        auto successor = std::make_shared<SingleAgentSpaceTimeState>(position, nextT);
        successors.emplace_back(successor, costWait);
    }

    return successors;
}

std::unordered_map<int, std::vector<int>> SingleAgentSpaceTimeProblem::getPositions(std::vector<std::shared_ptr<SingleAgentSpaceTimeState>> states) const {
    std::unordered_map<int, std::vector<int>> positions;
    for (auto state : states) {
        positions[agentId].push_back(state->getPosition());
    }
    return positions;
}

const int SingleAgentSpaceTimeProblem::getStart() const {
    return start;
}

const int SingleAgentSpaceTimeProblem::getTarget() const {
    return target;
}

ObjectiveFunction SingleAgentSpaceTimeProblem::getObjFunction() {
    return objective;
}

std::vector<int> SingleAgentSpaceTimeProblem::getAgentIds() const {
    return {agentId};
}
