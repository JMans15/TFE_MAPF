//
// Created by Arthur Mahy on 27/03/2023.
//

#include "OldSimpleIndependenceDetection.h"
#include <unordered_map>
#include <utility>

OldSimpleIndependenceDetection::OldSimpleIndependenceDetection(std::shared_ptr<MultiAgentProblemWithConstraints> problem, TypeOfHeuristic typeOfHeuristic)
        : problem(std::move(problem))
        , typeOfHeuristic(typeOfHeuristic)
{}

bool OldSimpleIndependenceDetection::planSingletonGroups() {
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

struct PairHasher {
    std::size_t operator()(const std::pair<int, int> &pair) const {
        size_t result = 0;
        if (pair.first<pair.second){
            boost::hash_combine(result, pair.first);
            boost::hash_combine(result, pair.second);
        } else {
            boost::hash_combine(result, pair.second);
            boost::hash_combine(result, pair.first);
        }
        return result;
    }
};

struct PairEquality {
    bool operator()(const std::pair<int, int> &a, const std::pair<int, int> &b) const {
        return (a.first==b.first && a.second==b.second) or (a.first==b.second && a.second==b.first);
    }
};

std::tuple<bool, std::shared_ptr<Group>, std::shared_ptr<Group>> OldSimpleIndependenceDetection::findAConflict() {
    int numberOfTimesteps = 0;
    for (const std::shared_ptr<Group>& group : groups){
        for (int agent : group->getAgents()){
            numberOfTimesteps = std::max((int)group->getSolution()->getPositions()[agent].size(), numberOfTimesteps);
        }
    }
    for (const std::shared_ptr<Group>& group : groups){
        group->getSolution()->lengthenPositions(numberOfTimesteps);
    }
    // Vertex conflict
    for (int t = 0; t < numberOfTimesteps; t++){
        std::unordered_map<int, std::shared_ptr<Group>> positionsAtThisTimestep;
        for (const std::shared_ptr<Group>& group : groups){
            for (int agent : group->getAgents()){
                int positionOfAgentAtTimestep = group->getSolution()->getPositions()[agent][t];
                if (positionsAtThisTimestep.count(positionOfAgentAtTimestep)>0){
                    return {true, positionsAtThisTimestep[positionOfAgentAtTimestep], group};
                } else {
                    positionsAtThisTimestep[positionOfAgentAtTimestep]=group;
                }
            }
        }
    }
    // Edge conflict
    for (int t = 0; t < numberOfTimesteps-1; t++){
        std::unordered_map<std::pair<int, int>, std::shared_ptr<Group>, PairHasher, PairEquality> edgesAtThisTimestep;
        for (const std::shared_ptr<Group>& group : groups){
            for (int agent : group->getAgents()){
                auto pathOfAgent = group->getSolution()->getPositions()[agent];
                if (edgesAtThisTimestep.count({pathOfAgent[t], pathOfAgent[t + 1]})>0){
                    return {true, edgesAtThisTimestep[{pathOfAgent[t], pathOfAgent[t + 1]}], group};
                } else {
                    edgesAtThisTimestep[{pathOfAgent[t + 1], pathOfAgent[t]}] = group;
                }
            }
        }
    }
    return {false, {}, {}};
}

bool OldSimpleIndependenceDetection::mergeGroupsAndPlanNewGroup(std::shared_ptr<Group> groupA, std::shared_ptr<Group> groupB) {
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
    auto prob = std::make_shared<MultiAgentProblemWithConstraints>(problem->getGraph(), starts, targets, problem->getObjFunction(), agentIds, problem->getSetOfHardVertexConstraints(), problem->getSetOfHardEdgeConstraints(), INT_MAX, problem->getSetOfSoftVertexConstraints(), problem->getSetOfSoftEdgeConstraints());
    auto solution = AStar<MultiAgentProblemWithConstraints, MultiAgentState>(prob, typeOfHeuristic).solve();
    newGroup->putSolution(solution);
    if (not solution->getFoundPath()){
        return false;
    }
    return true;
}

std::shared_ptr<Solution> OldSimpleIndependenceDetection::combineSolutions() {
    std::unordered_map<int, vector<int>> positions;
    vector<int> ids = problem->getAgentIds();
    for (const std::shared_ptr<Group>& group : groups){
        for (int agentId : group->getAgents()){
            positions[agentId] = group->getSolution()->getPathOfAgent(agentId);
        }
    }
    int numberOfTimesteps = positions.begin()->second.size();
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

std::shared_ptr<Solution> OldSimpleIndependenceDetection::solve() {

    LOG("===== Simple Independent Detection Search ====");

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
    auto conflict = findAConflict();
    while (std::get<0>(conflict)){
        auto groupA = std::get<1>(conflict);
        auto groupB = std::get<2>(conflict);

        if (not mergeGroupsAndPlanNewGroup(groupA, groupB)){
            return std::make_shared<Solution>();
        }

        conflict = findAConflict();
    }
    return combineSolutions();

}