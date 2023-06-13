//
// Created by Arthur Mahy on 13/02/2023.
//

#include "SingleAgentProblem.h"

#include <utility>

SingleAgentProblem::SingleAgentProblem(std::shared_ptr<Graph> graph, int start, int target, ObjectiveFunction m_objective,
                                                                     int agentId, const HardVertexConstraintsSet &setOfHardVertexConstraints,
                                                                     const HardEdgeConstraintsSet &setOfHardEdgeConstraints, int maxCost,
                                                                     const SoftVertexConstraintsMultiSet& setOfSoftVertexConstraints,
                                                                     const SoftEdgeConstraintsMultiSet& setOfSoftEdgeConstraints,
                                                                     int startTime)
    : graph(graph)
    , numberOfAgents(1)
    , start(start)
    , target(target)
    , agentId(agentId)
    , objective(m_objective)
    , setOfHardVertexConstraints(setOfHardVertexConstraints)
    , setOfHardEdgeConstraints(setOfHardEdgeConstraints)
    , setOfSoftVertexConstraints(setOfSoftVertexConstraints)
    , setOfSoftEdgeConstraints(setOfSoftEdgeConstraints)
    , maxCost(maxCost)
    , startTime(startTime)
{
    LOG("==== Single Agent Problem ====");
    if (m_objective == SumOfCosts){
        objective = Makespan;
    }
    if (m_objective != Makespan && m_objective != Fuel && m_objective != SumOfCosts){
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
            LOG("   Agent " << constraint.getAgent() << " is at position " << constraint.getPosition() << " at time " << constraint.getTime() << ".");
        }
    }
    if (!setOfSoftEdgeConstraints.empty()){
        LOG("The problem has the following soft edge constraints :");
        for (const auto& constraint : setOfSoftEdgeConstraints){
            LOG("   Agent " << constraint.getAgent() << " is occupying the edge (" << constraint.getPosition1() << ", " << constraint.getPosition2() << ") between time " << constraint.getTime()-1 << " and time "<< constraint.getTime() << ".");
        }
    }
    if (maxCost!=INT_MAX){
        LOG("The solution of this problem must have a cost inferior or equal to " << maxCost);
    }
    if (startTime!=0){
        LOG("The start time of the problem is " << startTime);
    }
    LOG(" ");

}

SingleAgentProblem::SingleAgentProblem(std::shared_ptr<Graph> graph, int start, int target, int agentId, int maxCost)
    : SingleAgentProblem(std::move(graph), start, target, Fuel, agentId, HardVertexConstraintsSet(), HardEdgeConstraintsSet(), maxCost,
                         SoftVertexConstraintsMultiSet(),SoftEdgeConstraintsMultiSet(), 0)
{}

bool SingleAgentProblem::okForConstraints(int position, int newPosition, int time) const {
    if (setOfHardVertexConstraints.find({agentId, newPosition, time}) == setOfHardVertexConstraints.end()){
        return setOfHardEdgeConstraints.find({agentId, position, newPosition, time}) == setOfHardEdgeConstraints.end();
    }
    return false;
}

bool SingleAgentProblem::okForConstraints(int newPosition, int time) const {
    return setOfHardVertexConstraints.find({agentId, newPosition, time}) == setOfHardVertexConstraints.end();
}

int SingleAgentProblem::numberOfViolations(int position, int newPosition, int time) const {
    int count = 0;
    auto range = setOfSoftVertexConstraints.equal_range({0, newPosition, time});
    for (auto it = range.first; it != range.second; ++it) {
        if (it->getAgent()!=agentId){
            count += 1;
        }
    }
    auto range2 = setOfSoftEdgeConstraints.equal_range({0, position, newPosition, time});
    for (auto it = range2.first; it != range2.second; ++it) {
        if (it->getAgent()!=agentId){
            count += 1;
        }
    }
    return count;
}

int SingleAgentProblem::numberOfViolations(int newPosition, int time) const {
    int count = 0;
    auto range = setOfSoftVertexConstraints.equal_range({0, newPosition, time});
    for (auto it = range.first; it != range.second; ++it) {
        if (it->getAgent()!=agentId){
            count += 1;
        }
    }
    return count;
}

const int SingleAgentProblem::getStart() const {
    return start;
}

const int SingleAgentProblem::getTarget() const {
    return target;
}

ObjectiveFunction SingleAgentProblem::getObjFunction() {
    return objective;
}

int SingleAgentProblem::getAgentId() const {
    return agentId;
}

bool SingleAgentProblem::isImpossible() const {
    return impossible;
}

std::shared_ptr<Graph> SingleAgentProblem::getGraph() const {
    return graph;
}

int SingleAgentProblem::getNumberOfAgents() const {
    return numberOfAgents;
}

int SingleAgentProblem::getMaxCost() const {
    return maxCost;
}

int SingleAgentProblem::getStartTime() const {
    return startTime;
}

bool SingleAgentProblem::hasTimeConstraints() const {
    if (not setOfHardVertexConstraints.empty()){
        return true;
    }
    if (not setOfHardEdgeConstraints.empty()){
        return true;
    }
    if (not setOfSoftVertexConstraints.empty()){
        return true;
    }
    if (not setOfSoftEdgeConstraints.empty()){
        return true;
    }
    return false;
}

bool SingleAgentProblem::isMultiAgentProblem() const {
    return false;
}
