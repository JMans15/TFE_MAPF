//
// Created by Arthur Mahy on 10/04/2023.
//

#include "ConflictBasedSearch.h"

#include <utility>

ConflictBasedSearch::ConflictBasedSearch(std::shared_ptr<MultiAgentProblemWithConstraints> problem, TypeOfHeuristic typeOfHeuristic)
    : problem(std::move(problem))
    , typeOfHeuristic(typeOfHeuristic)
    , numberOfVisitedNodes(0)
    {}

std::tuple<std::unordered_map<int, std::vector<int>>,int, std::unordered_map<int, int>> ConflictBasedSearch::planIndividualPaths() {
    std::unordered_map<int, std::vector<int>> solutions;
    std::unordered_map<int, int> costs;
    int cost = 0;
    for (int i = 0; i < problem->getNumberOfAgents(); i++){
        int agentId = problem->getAgentIds()[i];
        auto prob = std::make_shared<SingleAgentProblemWithConstraints>(problem->getGraph(), problem->getStarts()[i], problem->getTargets()[i], problem->getObjFunction(), agentId, problem->getSetOfHardVertexConstraints(), problem->getSetOfHardEdgeConstraints(), INT_MAX, problem->getSetOfSoftVertexConstraints(), problem->getSetOfSoftEdgeConstraints());
        auto solution = AStar<SingleAgentProblemWithConstraints, SingleAgentSpaceTimeState>(prob, typeOfHeuristic).solve();
        if (not solution->getFoundPath()){
            return {solutions, -1, costs};
        }
        solutions[agentId] = solution->getPathOfAgent(agentId);
        costs[agentId] = solution->getCost();
        if (problem->getObjFunction()==Makespan){
            cost = std::max(solution->getCost(), cost);
        } else {
            cost += solution->getCost();
        }
    }
    return {solutions, cost, costs};
}

std::set<AgentConflict> ConflictBasedSearch::calculateSetOfConflicts(const std::unordered_map<int, std::vector<int>>& solutions) {
    std::set<AgentConflict> setOfConflicts;
    for (auto [agentA, pathA] : solutions){
        for (auto [agentB, pathB] : solutions){
            if (agentA < agentB){
                // Vertex conflict
                for (int t = 0; t < std::min(pathA.size(), pathB.size()); t++){
                    if (pathA[t]==pathB[t]){
                        setOfConflicts.insert({agentA, agentB, pathA[t], t});
                    }
                }
                // Edge conflict
                for (int t = 0; t < std::min(pathA.size(), pathB.size())-1; t++){
                    // We only add an edge conflict if 2 agents are traversing the edge with different directions
                    // The other case is already token into account with 2 vertex conflicts
                    if (pathA[t]==pathB[t+1] and pathA[t+1]==pathB[t]){
                        setOfConflicts.insert({agentA, agentB, pathA[t], pathA[t + 1], t+1});
                    }
                }
            }
        }
    }
    return setOfConflicts;
}

std::set<AgentConflict> ConflictBasedSearch::updateSetOfConflicts(const std::unordered_map<int, std::vector<int>>& fullSolutions, const std::set<AgentConflict>& setOfConflicts, std::unordered_map<int, std::vector<int>> successorSolution){
    std::set<AgentConflict> successorSetOfConflicts;
    int agentId = successorSolution.begin()->first;
    vector<int> pathOfAgentId = successorSolution.begin()->second;
    for (const auto& conflict : setOfConflicts){
        if (conflict.getAgent1()!=agentId and conflict.getAgent2()!=agentId){
            successorSetOfConflicts.insert(conflict);
        }
    }
    for (auto [agentA, pathA] : fullSolutions){
        if (agentA != agentId){
            // Vertex conflict
            for (int t = 0; t < std::min(pathA.size(), pathOfAgentId.size()); t++){
                if (pathA[t]==pathOfAgentId[t]){
                    successorSetOfConflicts.insert({agentA, agentId, pathA[t], t});
                }
            }
            // Edge conflict
            for (int t = 0; t < std::min(pathA.size(), pathOfAgentId.size())-1; t++){
                if (pathA[t]==pathOfAgentId[t+1] and pathA[t+1]==pathOfAgentId[t]){
                    successorSetOfConflicts.insert({agentA, agentId, pathA[t], pathA[t + 1], t+1});
                }
            }
        }
    }
    return successorSetOfConflicts;
}

std::tuple<std::set<VertexConstraint>, std::set<EdgeConstraint>, std::unordered_map<int, int>, std::unordered_map<int, vector<int>>> ConflictBasedSearch::retrieveSetsOfConstraintsAndCostsAndSolutions(std::shared_ptr<ConflictTreeNode> node) {
    std::set<VertexConstraint> fullSetOfVertexConstraints = problem->getSetOfHardVertexConstraints();
    std::set<EdgeConstraint> fullSetOfEdgeConstraints = problem->getSetOfHardEdgeConstraints();
    std::unordered_map<int, int> fullCosts;
    std::unordered_map<int, std::vector<int>> fullSolutions;

    while (node) {
        if (node->getVertexConstraint()!=nullptr){
            fullSetOfVertexConstraints.insert(*node->getVertexConstraint());
        } else {
            if (node->getEdgeConstraint()!=nullptr){
                fullSetOfEdgeConstraints.insert(*node->getEdgeConstraint());
            }
        }
        for (auto [agentId, cost] : node->getCosts()){
            if (fullCosts.count(agentId)==0){
                fullCosts[agentId] = cost;
            }
        }
        for (auto [agentId, path] : node->getSolution()){
            if (fullSolutions.count(agentId)==0){
                fullSolutions[agentId] = path;
            }
        }
        node = node->getParent();
    }

    return {fullSetOfVertexConstraints, fullSetOfEdgeConstraints, fullCosts, fullSolutions};
}

std::unordered_map<int, vector<int>> ConflictBasedSearch::retrieveSolutions(std::shared_ptr<ConflictTreeNode> node) {
    std::unordered_map<int, std::vector<int>> solutions;

    while (node) {
        for (auto [agentId, path] : node->getSolution()){
            if (solutions.count(agentId)==0){
                solutions[agentId] = path;
            }
        }
        if ((int)solutions.size() == problem->getNumberOfAgents()){
            break;
        }
        node = node->getParent();
    }

    return solutions;
}

std::shared_ptr<Solution> ConflictBasedSearch::combineSolutions(const std::shared_ptr<ConflictTreeNode>& node) {
    auto solutions = retrieveSolutions(node);
    int numberOfTimesteps = 0;
    for (const auto& a : solutions){
        numberOfTimesteps = std::max((int)a.second.size(), numberOfTimesteps);
    }
    for (auto i : solutions){
        int actualLength = (int) i.second.size();
        if (actualLength < numberOfTimesteps){
            solutions[i.first].resize(numberOfTimesteps, i.second[actualLength-1]);
        }
    }
    return std::make_shared<Solution>(node->getCost(), numberOfVisitedNodes, numberOfTimesteps, solutions);
}

std::shared_ptr<Solution> ConflictBasedSearch::solve() {
    LOG("===== Conflict Based Search ====");

    if (problem->isImpossible()){
        return std::make_shared<Solution>();
    }

    auto [solution, cost, costs] = planIndividualPaths();
    if (cost == -1){
        return std::make_shared<Solution>();
    }

    std::set<AgentConflict> setOfConflicts = calculateSetOfConflicts(solution);

    if (cost <= problem->getMaxCost()){
        fringe.insert(std::make_shared<ConflictTreeNode>(nullptr, nullptr, solution, costs, cost, nullptr, setOfConflicts));
    }

    while (!fringe.empty()){
        auto it = fringe.begin();
        const auto node = *it;
        fringe.erase(it);

        numberOfVisitedNodes += 1;

        if (node->getSetOfConflicts().empty()){
            return combineSolutions(node);
        }

        auto conflict = *node->getSetOfConflicts().begin();

        int agent1 = conflict.getAgent1();
        int agent2 = conflict.getAgent2();

        auto [fullSetOfVertexConstraints, fullSetOfEdgeConstraints, fullCosts, fullSolutions] = retrieveSetsOfConstraintsAndCostsAndSolutions(node);

        for (int agentId : {agent1, agent2}){
            std::shared_ptr<EdgeConstraint> successorEdgeConstraint;
            std::shared_ptr<VertexConstraint> successorVertexConstraint;
            auto newFullSetOfVertexConstraints = fullSetOfVertexConstraints;
            auto newFullSetOfEdgeConstraints = fullSetOfEdgeConstraints;
            if (conflict.isVertexConflict()){
                successorEdgeConstraint = nullptr;
                successorVertexConstraint = std::make_shared<VertexConstraint>(agentId, conflict.getPosition1(), conflict.getTime());
                newFullSetOfVertexConstraints.insert({agentId, conflict.getPosition1(), conflict.getTime()});
            } else { // edge conflict
                successorVertexConstraint = nullptr;
                if (agentId==agent1){
                    successorEdgeConstraint = std::make_shared<EdgeConstraint>(agentId, conflict.getPosition1(), conflict.getPosition2(), conflict.getTime());
                    newFullSetOfEdgeConstraints.insert({agentId, conflict.getPosition1(), conflict.getPosition2(), conflict.getTime()});
                } else {
                    successorEdgeConstraint = std::make_shared<EdgeConstraint>(agentId, conflict.getPosition2(), conflict.getPosition1(), conflict.getTime());
                    newFullSetOfEdgeConstraints.insert({agentId, conflict.getPosition2(), conflict.getPosition1(), conflict.getTime()});
                }
            }
            auto prob = std::make_shared<SingleAgentProblemWithConstraints>(problem->getGraph(), problem->getStartOf(agentId), problem->getTargetOf(agentId), problem->getObjFunction(), agentId, newFullSetOfVertexConstraints, newFullSetOfEdgeConstraints, INT_MAX, problem->getSetOfSoftVertexConstraints(), problem->getSetOfSoftEdgeConstraints());
            auto solution = AStar<SingleAgentProblemWithConstraints, SingleAgentSpaceTimeState>(prob, typeOfHeuristic).solve();
            if (solution->getFoundPath()){
                std::unordered_map<int,int> successorCosts;
                successorCosts[agentId] = solution->getCost();
                std::unordered_map<int, std::vector<int>> successorSolution;
                successorSolution[agentId] = solution->getPathOfAgent(agentId);
                int successorCost;
                if (problem->getObjFunction() == Makespan){
                    successorCost = 0;
                    for (auto a : fullCosts){
                        if (a.first == agentId){
                            successorCost = std::max(successorCost, solution->getCost());
                        } else {
                            successorCost = std::max(successorCost, a.second);
                        }
                    }
                } else {
                    successorCost = node->getCost() - fullCosts[agentId] + solution->getCost();
                }
                if (successorCost <= problem->getMaxCost()){
                    auto successorSetOfConflicts = updateSetOfConflicts(fullSolutions, node->getSetOfConflicts(), successorSolution);
                    fringe.insert(std::make_shared<ConflictTreeNode>(successorEdgeConstraint, successorVertexConstraint, successorSolution, successorCosts, successorCost, node, successorSetOfConflicts));
                }
            }
        }
    }
    LOG("No path has been found.");
    return std::make_shared<Solution>();
}
