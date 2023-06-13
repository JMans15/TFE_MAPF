//
// Created by Arthur Mahy on 04/01/2023.
//

#include "MultiAgentProblem.h"

MultiAgentProblem::MultiAgentProblem(const std::shared_ptr<Graph>& graph, std::vector<int> starts, std::vector<int> targets,
                                                                   ObjectiveFunction objective, const std::vector<int>& m_agentIds,
                                                                   const HardVertexConstraintsSet &setOfHardVertexConstraints,
                                                                   const HardEdgeConstraintsSet &setOfHardEdgeConstraints, int maxCost,
                                                                   const SoftVertexConstraintsMultiSet& setOfSoftVertexConstraints,
                                                                   const SoftEdgeConstraintsMultiSet& setOfSoftEdgeConstraints,
                                                                   int startTime)
    : graph(graph)
    , numberOfAgents((int)starts.size())
    , starts(starts)
    , targets(targets)
    , agentIds(m_agentIds)
    , objective(objective)
    , setOfHardVertexConstraints(setOfHardVertexConstraints)
    , setOfHardEdgeConstraints(setOfHardEdgeConstraints)
    , setOfSoftVertexConstraints(setOfSoftVertexConstraints)
    , setOfSoftEdgeConstraints(setOfSoftEdgeConstraints)
    , maxCost(maxCost)
    , startTime(startTime)
{
    LOG("==== Multi Agent Problem With Constraints ====");
    LOG("Number of agents : " << numberOfAgents)
    
    if (objective != SumOfCosts && objective != Makespan && objective != Fuel) {
        LOG("The input for the objective function is not correct.");
        LOG("So, the default objective function will be applied.");
        objective = Fuel;
    }
    if (objective == SumOfCosts){
        LOG("Objective function : SumOfCosts");
    } else if (objective == Makespan){
        LOG("Objective function : Makespan");
    } else {
        LOG("Objective function : Fuel");
    }
    impossible = false;
    if ((not m_agentIds.empty()) and ((int)m_agentIds.size() != numberOfAgents)){
        impossible = true;
    }
    if (numberOfAgents != (int)targets.size()){
        impossible = true;
    }
    if (m_agentIds.empty()){
        for (int a = 0; a < numberOfAgents; a++){
            agentIds.emplace_back(a);
        }
    }
    for (int i = 0; i < numberOfAgents; i++){
        idToIndex[agentIds[i]] = i;
    }
    for (int agentId : agentIds){
        if (std::count(agentIds.begin(), agentIds.end(), agentId)>1){
            LOG("2 or more agents have the same id.");
            impossible = true;
            break;
        }
    }
    LOG("Start position of each agent :");
    for (int i = 0; i < numberOfAgents; i++) {
        LOG(" - Agent " << agentIds[i] << " : " << starts[i]);
        if (graph->getNeighbors(starts[i]).empty()) {
            LOG("   The start position of agent "<< agentIds[i] << " is unreachable.");
            impossible = true;
        }
    }
    LOG("Target position of each agent :");
    for (int i = 0; i < numberOfAgents; i++) {
        LOG(" - Agent " << agentIds[i] << " : " << targets[i]);
        if (graph->getNeighbors(targets[i]).empty()) {
            LOG("   The target position of agent "<< agentIds[i] << " is unreachable.");
            impossible = true;
        }
    }
    for (int start : starts){
        if (std::count(starts.begin(), starts.end(), start)>1){
            LOG("   2 or more agents have the same start position.");
            impossible = true;
            break;
        }
    }
    for (int target : targets){
        if (std::count(targets.begin(), targets.end(), target)>1){
            LOG("   2 or more agents have the same target position.");
            impossible = true;
            break;
        }
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

bool MultiAgentProblem::okForConstraints(int agent, int position, int newPosition, int time) const {
    if (setOfHardVertexConstraints.find({agentIds[agent], newPosition, time}) == setOfHardVertexConstraints.end()){
        return setOfHardEdgeConstraints.find({agentIds[agent], position, newPosition, time}) == setOfHardEdgeConstraints.end();
    }
    return false;
}

bool MultiAgentProblem::okForConstraints(int agent, int newPosition, int time) const {
    return setOfHardVertexConstraints.find({agentIds[agent], newPosition, time}) == setOfHardVertexConstraints.end();
}

int MultiAgentProblem::numberOfViolations(int agent, int position, int newPosition, int time) const {
    int count = 0;
    auto range = setOfSoftVertexConstraints.equal_range({0, newPosition, time});
    for (auto it = range.first; it != range.second; ++it) {
        if (it->getAgent()!=agentIds[agent]){
            count += 1;
        }
    }
    auto range2 = setOfSoftEdgeConstraints.equal_range({0, position, newPosition, time});
    for (auto it = range2.first; it != range2.second; ++it) {
        if (it->getAgent()!=agentIds[agent]){
            count += 1;
        }
    }
    return count;
}

int MultiAgentProblem::numberOfViolations(int agent, int newPosition, int time) const {
    int count = 0;
    auto range = setOfSoftVertexConstraints.equal_range({0, newPosition, time});
    for (auto it = range.first; it != range.second; ++it) {
        if (it->getAgent()!=agentIds[agent]){
            count += 1;
        }
    }
    return count;
}

const std::vector<int>& MultiAgentProblem::getStarts() const {
    return starts;
}

const std::vector<int>& MultiAgentProblem::getTargets() const {
    return targets;
}

ObjectiveFunction MultiAgentProblem::getObjFunction() {
    return objective;
}

HardVertexConstraintsSet MultiAgentProblem::getSetOfHardVertexConstraints() const {
    return setOfHardVertexConstraints;
}

HardEdgeConstraintsSet MultiAgentProblem::getSetOfHardEdgeConstraints() const {
    return setOfHardEdgeConstraints;
}

std::vector<int> MultiAgentProblem::getAgentIds() const {
    return agentIds;
}

int MultiAgentProblem::getStartOf(int id) {
    return starts[idToIndex[id]];
}

int MultiAgentProblem::getTargetOf(int id) {
    return targets[idToIndex[id]];
}

bool MultiAgentProblem::isImpossible() const {
    return impossible;
}

SoftVertexConstraintsMultiSet MultiAgentProblem::getSetOfSoftVertexConstraints() const {
    return setOfSoftVertexConstraints;
}

SoftEdgeConstraintsMultiSet MultiAgentProblem::getSetOfSoftEdgeConstraints() const {
    return setOfSoftEdgeConstraints;
}

std::shared_ptr<Graph> MultiAgentProblem::getGraph() const {
    return graph;
}

int MultiAgentProblem::getNumberOfAgents() const {
    return numberOfAgents;
}

int MultiAgentProblem::getMaxCost() const {
    return maxCost;
}

int MultiAgentProblem::getStartTime() const {
    return startTime;
}

bool MultiAgentProblem::hasTimeConstraints() const {
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

bool MultiAgentProblem::isMultiAgentProblem() const {
    return true;
}

