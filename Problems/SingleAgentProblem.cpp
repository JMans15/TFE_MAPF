//
// Created by Arthur Mahy on 13/02/2023.
//

#include "SingleAgentProblem.h"

SingleAgentProblem::SingleAgentProblem(const std::shared_ptr<Graph>& graph, int start, int target, ObjectiveFunction m_objective,
                                                                     int agentId, const HardVertexConstraintsSet &setOfHardVertexConstraints,
                                                                     const HardEdgeConstraintsSet &setOfHardEdgeConstraints, int maxCost,
                                                                     const SoftVertexConstraintsMultiSet& setOfSoftVertexConstraints,
                                                                     const SoftEdgeConstraintsMultiSet& setOfSoftEdgeConstraints,
                                                                     int startTime)
    : graph(graph)
    , start(start)
    , target(target)
    , agentId(agentId)
    , maxCost(maxCost)
    , objective(m_objective)
    , setOfHardVertexConstraints(setOfHardVertexConstraints)
    , setOfHardEdgeConstraints(setOfHardEdgeConstraints)
    , setOfSoftVertexConstraints(setOfSoftVertexConstraints)
    , setOfSoftEdgeConstraints(setOfSoftEdgeConstraints)
    , startTime(startTime)
{
    if (m_objective == SumOfCosts){
        objective = Makespan;
    }
    if (m_objective != Makespan && m_objective != Fuel && m_objective != SumOfCosts){
        objective = Makespan;
    }
    impossible = false;
    if (graph->getNeighbors(start).empty()){
        impossible = true;
    }
    if (graph->getNeighbors(target).empty()){
        impossible = true;
    }
    externalConstraints = false;
    if (not setOfHardVertexConstraints.empty()){
        externalConstraints = true;
    }
    if (not setOfHardEdgeConstraints.empty()){
        externalConstraints = true;
    }
    if (not setOfSoftVertexConstraints.empty()){
        externalConstraints = true;
    }
    if (not setOfSoftEdgeConstraints.empty()){
        externalConstraints = true;
    }
}

SingleAgentProblem::SingleAgentProblem(const std::shared_ptr<Graph>& graph, int start, int target, int agentId, int maxCost)
    : SingleAgentProblem(graph, start, target, Makespan, agentId, HardVertexConstraintsSet(), HardEdgeConstraintsSet(), maxCost,
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

int SingleAgentProblem::getStart() const {
    return start;
}

int SingleAgentProblem::getTarget() const {
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
    return 1;
}

int SingleAgentProblem::getMaxCost() const {
    return maxCost;
}

int SingleAgentProblem::getStartTime() const {
    return startTime;
}

bool SingleAgentProblem::hasExternalConstraints() const {
    return externalConstraints;
}

SoftEdgeConstraintsMultiSet SingleAgentProblem::getSetOfSoftEdgeConstraints() const {
    return setOfSoftEdgeConstraints;
}

SoftVertexConstraintsMultiSet SingleAgentProblem::getSetOfSoftVertexConstraints() const {
    return setOfSoftVertexConstraints;
}

HardEdgeConstraintsSet SingleAgentProblem::getSetOfHardEdgeConstraints() const {
    return setOfHardEdgeConstraints;
}

HardVertexConstraintsSet SingleAgentProblem::getSetOfHardVertexConstraints() const {
    return setOfHardVertexConstraints;
}

void SingleAgentProblem::print() const {
    std::cout<<"==== Single Agent Problem ===="<<std::endl;
    if (externalConstraints){
        if (objective == Makespan){
            std::cout<<"Objective function : Makespan (costWait = 1)"<<std::endl;
        } else {
            std::cout<<"Objective function : Fuel (costWait = 0)"<<std::endl;
        }
    }
    std::cout<<"Id of the agent : " << agentId<<std::endl;
    std::cout<<"Start position of the agent : " << start<<std::endl;
    if (graph->getNeighbors(start).empty()){
        std::cout<<"   The start position is unreachable."<<std::endl;
    }
    std::cout<<"Target position of the agent : " << target<<std::endl;
    if (graph->getNeighbors(target).empty()){
        std::cout<<"   The target position is unreachable."<<std::endl;
    }
    if (!setOfHardVertexConstraints.empty()){
        std::cout<<"The problem has the following hard vertex constraints :"<<std::endl;
        for (const auto& constraint : setOfHardVertexConstraints){
            std::cout<<"   Agent " << constraint.getAgent() << " cannot be at position " << constraint.getPosition() << " at time " << constraint.getTime() << "."<<std::endl;
        }
    }
    if (!setOfHardEdgeConstraints.empty()){
        std::cout<<"The problem has the following hard edge constraints :"<<std::endl;
        for (const auto& constraint : setOfHardEdgeConstraints){
            std::cout<<"   Agent " << constraint.getAgent() << " cannot go from position " << constraint.getPosition1() << " to position " << constraint.getPosition2() << " between " << constraint.getTime()-1 << " and time "<< constraint.getTime() << "."<<std::endl;
        }
    }
    if (!setOfSoftVertexConstraints.empty()){
        std::cout<<"The problem has the following soft vertex constraints :"<<std::endl;
        for (const auto& constraint : setOfSoftVertexConstraints){
            std::cout<<"   Agent " << constraint.getAgent() << " is at position " << constraint.getPosition() << " at time " << constraint.getTime() << "."<<std::endl;
        }
    }
    if (!setOfSoftEdgeConstraints.empty()){
        std::cout<<"The problem has the following soft edge constraints :"<<std::endl;
        for (const auto& constraint : setOfSoftEdgeConstraints){
            std::cout<<"   Agent " << constraint.getAgent() << " is occupying the edge (" << constraint.getPosition1() << ", " << constraint.getPosition2() << ") between time " << constraint.getTime()-1 << " and time "<< constraint.getTime() << "."<<std::endl;
        }
    }
    if (maxCost!=INT_MAX){
        std::cout<<"The solution of this problem must have a cost inferior or equal to " << maxCost<<std::endl;
    }
    if (startTime!=0){
        std::cout<<"The start time of the problem is " << startTime<<std::endl;
    }
}
