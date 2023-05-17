//
// Created by Arthur Mahy on 13/02/2023.
//

#include "SingleAgentProblemWithConstraints.h"

SingleAgentProblemWithConstraints::SingleAgentProblemWithConstraints(std::shared_ptr<Graph> graph, int start, int target, ObjectiveFunction objective,
                                                                     int agentId, const std::set<VertexConstraint> &setOfHardVertexConstraints,
                                                                     const std::set<EdgeConstraint> &setOfHardEdgeConstraints, int maxCost,
                                                                     const std::set<VertexConstraint> &setOfSoftVertexConstraints,
                                                                     const std::set<EdgeConstraint> &setOfSoftEdgeConstraints)
    : Problem(graph, 1, maxCost)
    , start(start)
    , target(target)
    , agentId(agentId)
    , objective(objective)
    , setOfHardVertexConstraints(setOfHardVertexConstraints)
    , setOfHardEdgeConstraints(setOfHardEdgeConstraints)
    , setOfSoftVertexConstraints(setOfSoftVertexConstraints)
    , setOfSoftEdgeConstraints(setOfSoftEdgeConstraints)
{
    LOG("==== Single Agent Problem With Constraints ====");
    if (objective == SumOfCosts){
        objective = Makespan;
    }
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
    impossible = false;
    LOG("Id of the agent : " << agentId);
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
    if (!setOfHardVertexConstraints.empty()){
        LOG("The problem has the following hard vertex constraints :");
        for (const auto& constraint : setOfHardVertexConstraints){
            LOG("   Agent " << constraint.getAgent() << " cannot be at position " << constraint.getPosition() << " at time " << constraint.getTime() << ".");
        }
    }
    if (!setOfHardEdgeConstraints.empty()){
        LOG("The problem has the following hard edge constraints :");
        for (const auto& constraint : setOfHardEdgeConstraints){
            LOG("   Agent " << constraint.getAgent() << " cannot go from position " << constraint.getPosition1() << " to position " << constraint.getPosition2() << " between " << constraint.getTime()-1 << " and time "<< constraint.getTime() << ".");
        }
    }
    if (!setOfSoftVertexConstraints.empty()){
        LOG("The problem has the following soft vertex constraints :");
        for (const auto& constraint : setOfSoftVertexConstraints){
            LOG("   Agent " << constraint.getAgent() << " cannot be at position " << constraint.getPosition() << " at time " << constraint.getTime() << ".");
        }
    }
    if (!setOfSoftEdgeConstraints.empty()){
        LOG("The problem has the following soft edge constraints :");
        for (const auto& constraint : setOfSoftEdgeConstraints){
            LOG("   Agent " << constraint.getAgent() << " cannot go from position " << constraint.getPosition1() << " to position " << constraint.getPosition2() << " between " << constraint.getTime()-1 << " and time "<< constraint.getTime() << ".");
        }
    }
    if (maxCost!=INT_MAX){
        LOG("The solution of this problem must have a cost inferior or equal to " << maxCost);
    }
    LOG(" ");

}

std::shared_ptr<SingleAgentSpaceTimeState> SingleAgentProblemWithConstraints::getStartState() const {
    return std::make_shared<SingleAgentSpaceTimeState>(start, 0);
}

bool SingleAgentProblemWithConstraints::isGoalState(std::shared_ptr<SingleAgentSpaceTimeState> state) const {
    return state->getPosition() == target;
}

bool SingleAgentProblemWithConstraints::okForConstraints(int position, int newPosition, int time) const {
    if (setOfHardVertexConstraints.find({agentId, newPosition, time}) == setOfHardVertexConstraints.end()){
        return setOfHardEdgeConstraints.find({agentId, position, newPosition, time}) == setOfHardEdgeConstraints.end();
    }
    return false;
}

bool SingleAgentProblemWithConstraints::okForConstraints(int newPosition, int time) const {
    return setOfHardVertexConstraints.find({agentId, newPosition, time}) == setOfHardVertexConstraints.end();
}

int SingleAgentProblemWithConstraints::numberOfViolations(int position, int newPosition, int time) const {
    int count = 0;
    if (setOfSoftVertexConstraints.find({agentId, newPosition, time}) != setOfSoftVertexConstraints.end()){
        count += 1;
    }
    if (setOfSoftEdgeConstraints.find({agentId, position, newPosition, time}) != setOfSoftEdgeConstraints.end()){
        count += 1;
    }
    return count;
}

int SingleAgentProblemWithConstraints::numberOfViolations(int newPosition, int time) const {
    if (setOfSoftVertexConstraints.find({agentId, newPosition, time}) == setOfSoftVertexConstraints.end()){
        return 0;
    }
    return 1;
}

std::vector<std::tuple<std::shared_ptr<SingleAgentSpaceTimeState>, int, int>> SingleAgentProblemWithConstraints::getSuccessors(std::shared_ptr<SingleAgentSpaceTimeState> state) const {
    std::vector<std::tuple<std::shared_ptr<SingleAgentSpaceTimeState>, int, int>> successors;
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
        if (okForConstraints(position, newPosition, nextT)){
            auto successor = std::make_shared<SingleAgentSpaceTimeState>(newPosition, nextT);
            successors.emplace_back(successor, costMovement, numberOfViolations(position, newPosition, nextT));
        }
    }

    // Wait
    if (okForConstraints(position, nextT)){
        auto successor = std::make_shared<SingleAgentSpaceTimeState>(position, nextT);
        successors.emplace_back(successor, costWait, numberOfViolations(position, nextT));
    }

    return successors;
}

std::unordered_map<int, std::vector<int>> SingleAgentProblemWithConstraints::getPositions(std::vector<std::shared_ptr<SingleAgentSpaceTimeState>> states) const {
    std::unordered_map<int, std::vector<int>> positions;
    for (auto state : states) {
        positions[agentId].push_back(state->getPosition());
    }
    return positions;
}

const int SingleAgentProblemWithConstraints::getStart() const {
    return start;
}

const int SingleAgentProblemWithConstraints::getTarget() const {
    return target;
}

ObjectiveFunction SingleAgentProblemWithConstraints::getObjFunction() {
    return objective;
}

std::vector<int> SingleAgentProblemWithConstraints::getAgentIds() const {
    return {agentId};
}

bool SingleAgentProblemWithConstraints::isImpossible() const {
    return impossible;
}