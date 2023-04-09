//
// Created by Arthur Mahy on 28/02/2023.
//

#include "IndependenceDetection.h"

IndependenceDetection::IndependenceDetection(std::shared_ptr<MultiAgentProblem> problem, TypeOfHeuristic typeOfHeuristic)
        : SimpleIndependenceDetection(problem, typeOfHeuristic)
{}


bool IndependenceDetection::replanGroupAAvoidingGroupB(std::shared_ptr<Group> groupA, std::shared_ptr<Group> groupB){
    std::set<VertexConstraint> vertexIllegalTable;
    std::set<EdgeConstraint> edgeIllegalTable;
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
        auto prob = std::make_shared<SingleAgentSpaceTimeProblem>(problem->getGraph(), starts[0], targets[0], problem->getObjFunction(), agentIds[0], vertexIllegalTable, edgeIllegalTable, groupA->getSolution()->getCost());
        solution = AStar<SingleAgentSpaceTimeProblem, SingleAgentSpaceTimeState>(prob, typeOfHeuristic).solve();
    } else {
        auto prob = std::make_shared<MultiAgentProblem>(problem->getGraph(), starts, targets, problem->getObjFunction(), agentIds, vertexIllegalTable, edgeIllegalTable, groupA->getSolution()->getCost());
        solution = AStar<MultiAgentProblem, MultiAgentState>(prob, typeOfHeuristic).solve();
    }
    if (solution->getFoundPath() and solution->isValid()) {
        groupA->putSolution(solution);
        return true;
    }
    return false;
}

std::shared_ptr<Solution> IndependenceDetection::solve() {

    std::cout << "===== Independent Detection Search ====" << std::endl;

    // Assign each agent to a singleton group
    for (int agent : problem->getAgentIds()){
        groups.insert(std::make_shared<Group>(std::set<int>{agent}));
    }

    // Plan a path for each group
    if (not planSingletonGroups()){
        return {};
    }

    std::cout << "The groups are the following:" << std::endl;
    for (auto group : groups){
        std::cout << "group:" << std::endl;
        for (auto agent : group->getAgents()){
            std::cout << agent << std::endl;
        }
    }

    // TODO : the conflict avoidance table (and modify AStar accordingly)
    // Fill conflict avoidance table with every path here

    auto conflict = findAConflict();
    while (std::get<0>(conflict)){
        auto groupA = std::get<1>(conflict);
        auto groupB = std::get<2>(conflict);

        std::cout << "These 2 groups are in conflict : " << std::endl;
        std::cout << "groupA:" << std::endl;
        for (auto agent : groupA->getAgents()){
            std::cout << agent << std::endl;
        }
        std::cout << "groupB:" << std::endl;
        for (auto agent : groupB->getAgents()){
            std::cout << agent << std::endl;
        }

        // TODO : does alreadyConflictedBefore work ??
        if (alreadyConflictedBefore.count(std::pair <std::shared_ptr<Group>, std::shared_ptr<Group>>(groupA, groupB))==0){
            alreadyConflictedBefore.insert(std::pair <std::shared_ptr<Group>, std::shared_ptr<Group>>(groupA, groupB));
            if (not replanGroupAAvoidingGroupB(groupA, groupB)){
                if (not replanGroupAAvoidingGroupB(groupB, groupA)) {
                    if (not mergeGroupsAndPlanNewGroup(groupA, groupB)){
                        return {};
                    }
                    std::cout << "We couldn't find an alternate optimal solution for group A and group B -> we merge." << std::endl;
                } else {
                    std::cout << "We replanned this group : " << std::endl;
                    std::cout << "group:" << std::endl;
                    for (auto agent : groupB->getAgents()){
                        std::cout << agent << std::endl;
                    }
                }
            } else {
                std::cout << "We replanned this group : " << std::endl;
                std::cout << "group:" << std::endl;
                for (auto agent : groupA->getAgents()){
                    std::cout << agent << std::endl;
                }
            }
        } else {
            std::cout << "Group A and group B already conflicted before -> we merge." << std::endl;
            if (not mergeGroupsAndPlanNewGroup(groupA, groupB)){
                return {};
            }
        }

        std::cout << "The groups are the following:" << std::endl;
        for (auto group : groups){
            std::cout << "group:" << std::endl;
            for (auto agent : group->getAgents()){
                std::cout << agent << std::endl;
            }
        }

        // TODO : Update conflict avoidance table with changes made to paths

        conflict = findAConflict();
    }
    return combineSolutions();

}