//
// Created by Arthur Mahy on 13/02/2023.
//

#include "SingleAgentSpaceTimeProblem.h"

SingleAgentSpaceTimeProblem::SingleAgentSpaceTimeProblem(std::shared_ptr<Graph> graph, int start, int target, ObjectiveFunction objective,
                                                         int agentId, const std::set<VertexConstraint> &setOfVertexConstraints,
                                                         const std::set<EdgeConstraint> &setOfEdgeConstraints, int maxCost)
    : Problem(graph, 1, maxCost)
    , start(start)
    , target(target)
    , agentId(agentId)
    , objective(objective)
    , setOfVertexConstraints(setOfVertexConstraints)
    , setOfEdgeConstraints(setOfEdgeConstraints)
{
    LOG("==== Single Agent Space Time Problem ====");
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
    LOG("Id of the agent : " << agentId);
    LOG("Start position of the agent : " << start);
    if (graph->getNeighbors(start).empty()){
        LOG("   The start position is unreachable.");
    }
    LOG("Target position of the agent : " << target);
    if (graph->getNeighbors(target).empty()){
        LOG("   The target position is unreachable.");
    }
    if (!setOfVertexConstraints.empty()){
        LOG("The problem has the following vertex constraints :");
        for (const auto& constraint : setOfVertexConstraints){
            LOG("   Agent " << constraint.getAgent() << " cannot be at position " << constraint.getPosition() << " at time " << constraint.getTime() << ".");
        }
    }
    if (!setOfEdgeConstraints.empty()){
        LOG("The problem has the following edge constraints :");
        for (const auto& constraint : setOfEdgeConstraints){
            LOG("   Agent " << constraint.getAgent() << " cannot go from position " << constraint.getPosition1() << " to position " << constraint.getPosition2() << " between time " << constraint.getTime()-1 << " and time "<< constraint.getTime() << ".");
        }
    }
    if (maxCost!=INT_MAX){
        LOG("The solution of this problem must have a cost inferior or equal to " << maxCost);
    }
    LOG(" ");

}

std::shared_ptr<SingleAgentSpaceTimeState> SingleAgentSpaceTimeProblem::getStartState() const {
    return std::make_shared<SingleAgentSpaceTimeState>(start, 0);
}

bool SingleAgentSpaceTimeProblem::isGoalState(std::shared_ptr<SingleAgentSpaceTimeState> state) const {
    return state->getPosition() == target;
}

bool SingleAgentSpaceTimeProblem::okForConstraints(int position, int newPosition, int time) const {
    if (setOfVertexConstraints.find({agentId, newPosition, time}) == setOfVertexConstraints.end()){
        return setOfEdgeConstraints.find({agentId, position, newPosition, time}) == setOfEdgeConstraints.end();
    }
    return false;
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
        if (okForConstraints(position, newPosition, nextT)){
            auto successor = std::make_shared<SingleAgentSpaceTimeState>(newPosition, nextT);
            successors.emplace_back(successor, costMovement);
        }
    }

    // Wait
    if (okForConstraints(position, position, nextT)){
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
