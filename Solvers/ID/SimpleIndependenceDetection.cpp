//
// Created by Arthur Mahy on 15/05/2023.
//

#include "SimpleIndependenceDetection.h"
#include <unordered_map>
#include <utility>

SimpleIndependenceDetection::SimpleIndependenceDetection(std::shared_ptr<MultiAgentProblemWithConstraints> problem, TypeOfHeuristic typeOfHeuristic, bool CAT)
        : problem(std::move(problem))
        , typeOfHeuristic(typeOfHeuristic)
        , CAT(CAT)
{}

bool SimpleIndependenceDetection::planSingletonGroups() {
    for (const std::shared_ptr<Group>& group : groups){
        int agentId = *group->getAgents().begin();
        auto prob = std::make_shared<SingleAgentProblemWithConstraints>(problem->getGraph(), problem->getStartOf(agentId), problem->getTargetOf(agentId), problem->getObjFunction(), agentId, problem->getSetOfHardVertexConstraints(), problem->getSetOfHardEdgeConstraints(), INT_MAX, problem->getSetOfSoftVertexConstraints(), problem->getSetOfSoftEdgeConstraints());
        auto search = AStar<SingleAgentProblemWithConstraints, SingleAgentSpaceTimeState>(prob, typeOfHeuristic);
        auto solution = search.solve();
        group->putSolution(solution);
        if (not solution->getFoundPath()){
            return false;
        }
    }
    return true;
}

void SimpleIndependenceDetection::calculateSetOfConflicts() {
    for (const auto& groupA : groups){
        for (const auto& groupB : groups){
            if (*groupA!=*groupB){
                for (auto [agentA, pathA] : groupA->getSolution()->getPositions()){
                    for (auto [agentB, pathB] : groupB->getSolution()->getPositions()){
                        if (agentA<agentB){
                            // Vertex conflict
                            for (int t = 0; t < std::min(pathA.size(), pathB.size()); t++){
                                if (pathA[t]==pathB[t]){
                                    setOfConflicts.insert({groupA, groupB, t});
                                }
                            }
                            // Edge conflict
                            for (int t = 0; t < std::min(pathA.size(), pathB.size())-1; t++){
                                // We only add an edge conflict if 2 agents are traversing the edge with different directions
                                // The other case is already token into account with 2 vertex conflicts
                                if (pathA[t]==pathB[t+1] and pathA[t+1]==pathB[t]){
                                    setOfConflicts.insert({groupA, groupB, t+1});
                                }
                            }
                        }
                    }
                }
            }
        }
    }
}

bool SimpleIndependenceDetection::mergeGroupsAndPlanNewGroup(std::shared_ptr<Group> groupA, std::shared_ptr<Group> groupB) {
    groups.erase(groupA);
    groups.erase(groupB);
    std::set<int> agents(groupA->getAgents());
    for (int i : groupB->getAgents()){
        agents.insert(i);
    }
    auto newGroup = std::make_shared<Group>(agents);
    groups.insert(newGroup);
    vector<int> starts;
    vector<int> targets;
    vector<int> agentIds;
    for (int agentId : newGroup->getAgents()){
        starts.push_back(problem->getStartOf(agentId));
        targets.push_back(problem->getTargetOf(agentId));
        agentIds.push_back(agentId);
    }
    std::set<VertexConstraint> vertexConflictAvoidanceTable = problem->getSetOfSoftVertexConstraints();
    std::set<EdgeConstraint> edgeConflictAvoidanceTable = problem->getSetOfSoftEdgeConstraints();
    if (CAT){
        for (const auto& group : groups){
            if (*group != *newGroup){
                for (const auto& a : group->getSolution()->getPositions()){
                    vector<int> pathOfAgent = a.second;
                    for (int agent : newGroup->getAgents()){
                        for (int t = 0; t < pathOfAgent.size(); t++){
                            vertexConflictAvoidanceTable.insert({agent, pathOfAgent[t], t});
                        }
                        for (int t = 1; t < pathOfAgent.size(); t++){
                            edgeConflictAvoidanceTable.insert({agent, pathOfAgent[t], pathOfAgent[t-1], t});
                            edgeConflictAvoidanceTable.insert({agent, pathOfAgent[t-1], pathOfAgent[t], t});
                        }
                    }
                }
            }
        }
    }
    auto prob = std::make_shared<MultiAgentProblemWithConstraints>(problem->getGraph(), starts, targets, problem->getObjFunction(), agentIds, problem->getSetOfHardVertexConstraints(), problem->getSetOfHardEdgeConstraints(), INT_MAX, vertexConflictAvoidanceTable, edgeConflictAvoidanceTable);
    auto solution = AStar<MultiAgentProblemWithConstraints, MultiAgentState>(prob, typeOfHeuristic).solve();
    newGroup->putSolution(solution);
    if (not solution->getFoundPath() or not solution->isValid()){
        return false;
    }

    // update the set of conflicts
    std::set<GroupConflict> newSetOfConflicts;
    for (const auto& conflict : setOfConflicts){
        if (*conflict.getGroupA()==*groupA or *conflict.getGroupB()==*groupA){
            continue;
        }
        if (*conflict.getGroupA()==*groupB or *conflict.getGroupB()==*groupB){
            continue;
        }
        newSetOfConflicts.insert(conflict);
    }
    setOfConflicts = newSetOfConflicts;
    for (const auto& group : groups){
        if (*group != *newGroup){
            for (auto [agentA, pathA] : group->getSolution()->getPositions()){
                for (auto [agentB, pathB] : newGroup->getSolution()->getPositions()){
                    // Vertex conflict
                    for (int t = 0; t < std::min(pathA.size(), pathB.size()); t++){
                        if (pathA[t]==pathB[t]){
                            setOfConflicts.insert({group, newGroup, t});
                        }
                    }
                    // Edge conflict
                    for (int t = 0; t < std::min(pathA.size(), pathB.size())-1; t++){
                        if (pathA[t]==pathB[t+1] and pathA[t+1]==pathB[t]){
                            setOfConflicts.insert({group, newGroup, t + 1});
                        }
                    }
                }
            }
        }
    }
    return true;
}

std::shared_ptr<Solution> SimpleIndependenceDetection::combineSolutions() {
    int numberOfTimesteps = 0;
    for (const std::shared_ptr<Group>& group : groups){
        for (int agent : group->getAgents()){
            numberOfTimesteps = std::max((int)group->getSolution()->getPositions()[agent].size(), numberOfTimesteps);
        }
    }
    for (const std::shared_ptr<Group>& group : groups){
        group->getSolution()->lengthenPositions(numberOfTimesteps);
    }
    std::unordered_map<int, vector<int>> positions;
    vector<int> ids = problem->getAgentIds();
    for (const std::shared_ptr<Group>& group : groups){
        for (int agentId : group->getAgents()){
            positions[agentId] = group->getSolution()->getPathOfAgent(agentId);
        }
    }
    auto solution = std::make_shared<Solution>(numberOfTimesteps, positions);
    if (problem->getMaxCost()!=INT_MAX){
        if (problem->getObjFunction()==Makespan){
            if (solution->getMakespanCost()>problem->getMaxCost()){
                return std::make_shared<Solution>();
            }
        } else if (problem->getObjFunction()==SumOfCosts){
            if (solution->getSumOfCostsCost()>problem->getMaxCost()){
                return std::make_shared<Solution>();
            }
        } else {
            if (solution->getFuelCost()>problem->getMaxCost()){
                return std::make_shared<Solution>();
            }
        }
    }
    return solution;
}

std::shared_ptr<Solution> SimpleIndependenceDetection::solve() {

    LOG("===== Simple Independent Detection Search (with set of conflicts) ====");

    if (problem->isImpossible()){
        return std::make_shared<Solution>();
    }

    // Assign each agent to a singleton group
    for (int agent : problem->getAgentIds()){
        groups.insert(std::make_shared<Group>(std::set<int>{agent}));
    }

    // Plan a path for each group
    if (not planSingletonGroups()){
        return std::make_shared<Solution>();
    }
    calculateSetOfConflicts();

    while (not setOfConflicts.empty()){
        auto conflict = setOfConflicts.begin();

        if (not mergeGroupsAndPlanNewGroup(conflict->getGroupA(), conflict->getGroupB())){
            return std::make_shared<Solution>();
        }
    }
    return combineSolutions();

}
