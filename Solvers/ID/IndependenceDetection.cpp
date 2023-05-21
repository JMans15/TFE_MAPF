//
// Created by Arthur Mahy on 16/05/2023.
//

#include "IndependenceDetection.h"

#include <utility>

IndependenceDetection::IndependenceDetection(std::shared_ptr<MultiAgentProblemWithConstraints> problem, TypeOfHeuristic typeOfHeuristic, bool CAT)
        : SimpleIndependenceDetection(std::move(problem), typeOfHeuristic), CAT(CAT)
{}


bool IndependenceDetection::replanGroupAAvoidingGroupB(const std::shared_ptr<Group>& groupA, const std::shared_ptr<Group>& groupB){
    std::set<VertexConstraint> vertexIllegalTable = problem->getSetOfHardVertexConstraints();
    std::set<EdgeConstraint> edgeIllegalTable = problem->getSetOfHardEdgeConstraints();
    for (auto a : groupB->getSolution()->getPositions()){
        vector<int> pathOfAgent = a.second;
        for (int agentA : groupA->getAgents()){
            for (int t = 0; t < pathOfAgent.size(); t++){
                // agent cannot go at position pathofagent[t] at time t
                vertexIllegalTable.insert({agentA, pathOfAgent[t], t});
            }
            for (int t = 1; t < pathOfAgent.size(); t++){
                // to avoid edge conflict
                edgeIllegalTable.insert({agentA, pathOfAgent[t], pathOfAgent[t-1], t});
            }
        }
    }
    std::set<VertexConstraint> vertexConflictAvoidanceTable = problem->getSetOfSoftVertexConstraints();
    std::set<EdgeConstraint> edgeConflictAvoidanceTable = problem->getSetOfSoftEdgeConstraints();
    if (CAT){
        for (const auto& group : groups){
            if (*group != *groupA){
                for (const auto& a : group->getSolution()->getPositions()){
                    vector<int> pathOfAgent = a.second;
                    for (int agentA : groupA->getAgents()){
                        for (int t = 0; t < pathOfAgent.size(); t++){
                            vertexConflictAvoidanceTable.insert({agentA, pathOfAgent[t], t});
                        }
                        for (int t = 1; t < pathOfAgent.size(); t++){
                            edgeConflictAvoidanceTable.insert({agentA, pathOfAgent[t], pathOfAgent[t-1], t});
                            edgeConflictAvoidanceTable.insert({agentA, pathOfAgent[t-1], pathOfAgent[t], t});
                        }
                    }
                }
            }
        }
    }
    vector<int> starts;
    vector<int> targets;
    vector<int> agentIds;
    for (int agentId : groupA->getAgents()){
        starts.push_back(problem->getStartOf(agentId));
        targets.push_back(problem->getTargetOf(agentId));
        agentIds.push_back(agentId);
    }
    std::shared_ptr<Solution> solution;
    if (groupA->getAgents().size()==1){
        auto prob = std::make_shared<SingleAgentProblemWithConstraints>(problem->getGraph(), starts[0], targets[0], problem->getObjFunction(), agentIds[0], vertexIllegalTable, edgeIllegalTable, groupA->getSolution()->getCost(), vertexConflictAvoidanceTable, edgeConflictAvoidanceTable);
        solution = AStar<SingleAgentProblemWithConstraints, SingleAgentSpaceTimeState>(prob, typeOfHeuristic).solve();
    } else {
        auto prob = std::make_shared<MultiAgentProblemWithConstraints>(problem->getGraph(), starts, targets, problem->getObjFunction(), agentIds, vertexIllegalTable, edgeIllegalTable, groupA->getSolution()->getCost(), vertexConflictAvoidanceTable, edgeConflictAvoidanceTable);
        solution = AStar<MultiAgentProblemWithConstraints, MultiAgentState>(prob, typeOfHeuristic).solve();
    }
    if (solution->getFoundPath() and solution->isValid()) {
        groupA->putSolution(solution);

        // update the set of conflicts
        std::set<GroupConflict> newSetOfConflicts;
        for (const auto& conflict : setOfConflicts){
            if (*conflict.getGroupA()==*groupA or *conflict.getGroupB()==*groupA){
                continue;
            }
            newSetOfConflicts.insert(conflict);
        }
        setOfConflicts = newSetOfConflicts;
        for (const auto& group : groups){
            if (*group != *groupA){
                for (auto [agent, path] : group->getSolution()->getPositions()){
                    for (auto [agentA, pathA] : groupA->getSolution()->getPositions()){
                        // Vertex conflict
                        for (int t = 0; t < std::min(path.size(), pathA.size()); t++){
                            if (path[t]==pathA[t]){
                                setOfConflicts.insert({group, groupA, t});
                            }
                        }
                        // Edge conflict
                        for (int t = 0; t < std::min(path.size(), pathA.size())-1; t++){
                            if (path[t]==pathA[t+1] and path[t+1]==pathA[t]){
                                setOfConflicts.insert({group, groupA, t + 1});
                            }
                        }
                    }
                }
            }
        }

        return true;
    }
    return false;
}

bool IndependenceDetection::mergeGroupsAndPlanNewGroup(std::shared_ptr<Group> groupA, std::shared_ptr<Group> groupB) {
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
                for (auto a : group->getSolution()->getPositions()){
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

std::shared_ptr<Solution> IndependenceDetection::solve() {

    LOG("===== Independent Detection Search (with set of conflicts) ====");

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

    /*std::cout << "The groups are the following:" << std::endl;
    for (auto group : groups){
        std::cout << "group:" << std::endl;
        for (auto agent : group->getAgents()){
            std::cout << agent << std::endl;
        }
    }*/

    calculateSetOfConflicts();

    while (not setOfConflicts.empty()){
        auto conflict = setOfConflicts.begin();

        auto groupA = conflict->getGroupA();
        auto groupB = conflict->getGroupB();

        /*std::cout << "These 2 groups are in conflict : " << std::endl;
        std::cout << "groupA:" << std::endl;
        for (auto agent : groupA->getAgents()){
            std::cout << agent << std::endl;
        }
        std::cout << "groupB:" << std::endl;
        for (auto agent : groupB->getAgents()){
            std::cout << agent << std::endl;
        }*/

        if (alreadyConflictedBefore.count({groupA, groupB})==0){
            alreadyConflictedBefore.insert({groupA, groupB});
            if (not replanGroupAAvoidingGroupB(groupA, groupB)){
                if (not replanGroupAAvoidingGroupB(groupB, groupA)) {
                    if (not mergeGroupsAndPlanNewGroup(groupA, groupB)){
                        return std::make_shared<Solution>();
                    }
                    //std::cout << "We couldn't find an alternate optimal solution for group A and group B -> we merge." << std::endl;
                } else {
                    /*std::cout << "We replanned this group : " << std::endl;
                    std::cout << "group:" << std::endl;
                    for (auto agent : groupB->getAgents()){
                        std::cout << agent << std::endl;
                    }*/
                }
            } else {
                /*std::cout << "We replanned this group : " << std::endl;
                std::cout << "group:" << std::endl;
                for (auto agent : groupA->getAgents()){
                    std::cout << agent << std::endl;
                }*/
            }
        } else {
            //std::cout << "Group A and group B already conflicted before -> we merge." << std::endl;
            if (not mergeGroupsAndPlanNewGroup(groupA, groupB)){
                return std::make_shared<Solution>();
            }
        }

        /*std::cout << "The groups are the following:" << std::endl;
        for (auto group : groups){
            std::cout << "group:" << std::endl;
            for (auto agent : group->getAgents()){
                std::cout << agent << std::endl;
            }
        }*/

    }
    return combineSolutions();

}