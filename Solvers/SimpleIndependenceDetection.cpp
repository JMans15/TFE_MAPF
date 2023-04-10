//
// Created by Arthur Mahy on 27/03/2023.
//

#include "SimpleIndependenceDetection.h"
#include <unordered_map>

SimpleIndependenceDetection::SimpleIndependenceDetection(std::shared_ptr<MultiAgentProblemWithConstraints> problem, TypeOfHeuristic typeOfHeuristic)
        : problem(problem)
        , typeOfHeuristic(typeOfHeuristic)
{}

bool SimpleIndependenceDetection::planSingletonGroups() {
    for (std::shared_ptr<Group> group : groups){
        int agentId = *group->getAgents().begin();
        auto prob = std::make_shared<SingleAgentProblem>(problem->getGraph(), problem->getStartOf(agentId), problem->getTargetOf(agentId), agentId);
        auto search = AStar<SingleAgentProblem, SingleAgentState>(prob, typeOfHeuristic);
        auto solution = search.solve();
        group->putSolution(solution);
        if (not solution->getFoundPath()){
            return false;
        }
    }
    return true;
}

std::tuple<bool, std::shared_ptr<Group>, std::shared_ptr<Group>> SimpleIndependenceDetection::findAConflict() {
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
                if (positionsAtThisTimestep.count(group->getSolution()->getPositions()[agent][t])>0){
                    return {true, positionsAtThisTimestep[group->getSolution()->getPositions()[agent][t]], group};
                } else {
                    positionsAtThisTimestep[group->getSolution()->getPositions()[agent][t]]=group;
                }
            }
        }
    }
    // Edge conflict
    for (int t = 0; t < numberOfTimesteps-1; t++){
        std::unordered_map<std::pair<int, int>, std::shared_ptr<Group>, PairHasher<int>, PairEquality<int>> edgesAtThisTimestep;
        for (const std::shared_ptr<Group>& group : groups){
            for (int agent : group->getAgents()){
                if (edgesAtThisTimestep.count(std::pair <int, int> (group->getSolution()->getPositions()[agent][t], group->getSolution()->getPositions()[agent][t+1]))>0){
                    return {true, edgesAtThisTimestep[std::pair <int, int> (group->getSolution()->getPositions()[agent][t], group->getSolution()->getPositions()[agent][t+1])], group};
                } else {
                    edgesAtThisTimestep[std::pair <int, int> (group->getSolution()->getPositions()[agent][t+1], group->getSolution()->getPositions()[agent][t])] = group;
                }
            }
        }
    }
    return {false, {}, {}};
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
    auto prob = std::make_shared<MultiAgentProblemWithConstraints>(problem->getGraph(), starts, targets, problem->getObjFunction(), agentIds);
    auto solution = AStar<MultiAgentProblemWithConstraints, MultiAgentState>(prob, typeOfHeuristic).solve();
    newGroup->putSolution(solution);
    if (not solution->getFoundPath() or not solution->isValid()){
        return false;
    }
    return true;
}

std::shared_ptr<Solution> SimpleIndependenceDetection::combineSolutions() {
    std::unordered_map<int, vector<int>> positions;
    vector<int> ids = problem->getAgentIds();
    for (const std::shared_ptr<Group>& group : groups){
        for (int agentId : group->getAgents()){
            positions[agentId] = group->getSolution()->getPathOfAgent(agentId);
        }
    }
    int numberOfTimesteps = positions.begin()->second.size();
    return std::make_shared<Solution>(numberOfTimesteps, positions);
}

std::shared_ptr<Solution> SimpleIndependenceDetection::solve() {

    std::cout << "===== Simple Independent Detection Search ====" << std::endl;
    // Assign each agent to a singleton group
    for (int agent : problem->getAgentIds()){
        groups.insert(std::make_shared<Group>(std::set<int>{agent}));
    }

    if (not planSingletonGroups()){
        return {};
    }
    auto conflict = findAConflict();
    while (std::get<0>(conflict)){
        auto groupA = std::get<1>(conflict);
        auto groupB = std::get<2>(conflict);

        if (not mergeGroupsAndPlanNewGroup(groupA, groupB)){
            return {};
        }

        conflict = findAConflict();
    }
    return combineSolutions();

}