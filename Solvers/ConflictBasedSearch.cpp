//
// Created by Arthur Mahy on 10/04/2023.
//

#include "ConflictBasedSearch.h"

#include <utility>

ConflictBasedSearch::ConflictBasedSearch(std::shared_ptr<MultiAgentProblemWithConstraints> problem, TypeOfHeuristic typeOfHeuristic)
    : problem(std::move(problem))
    , typeOfHeuristic(typeOfHeuristic)
    {}

std::tuple<std::unordered_map<int, std::vector<int>>,int, std::unordered_map<int, int>> ConflictBasedSearch::planIndividualPaths() {
    std::unordered_map<int, std::vector<int>> solutions;
    std::unordered_map<int, int> costs;
    int cost = 0;
    for (int i = 0; i < problem->getNumberOfAgents(); i++){
        int agentId = problem->getAgentIds()[i];
        auto prob = std::make_shared<SingleAgentProblem>(problem->getGraph(), problem->getStarts()[i], problem->getTargets()[i], agentId);
        auto solution = AStar<SingleAgentProblem, SingleAgentState>(prob, typeOfHeuristic).solve();
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

std::shared_ptr<Conflict> ConflictBasedSearch::findAConflict(std::shared_ptr<ConflictTreeNode> node) {
    auto solution = node->getSolution();
    int numberOfTimesteps = 0;
    for (const auto& a : solution){
        numberOfTimesteps = std::max((int)a.second.size(), numberOfTimesteps);
    }
    for (auto i : solution){
        int actualLength = (int) i.second.size();
        if (actualLength < numberOfTimesteps){
            solution[i.first].resize(numberOfTimesteps, i.second[actualLength-1]);
        }
    }
    // Vertex conflict
    for (int t = 0; t < numberOfTimesteps; t++){
        std::unordered_map<int, int> positionsAtThisTimestep;
        for (auto a : solution){
            int positionOfAgentAtTimestep = a.second[t];
            if (positionsAtThisTimestep.count(positionOfAgentAtTimestep)>0){
                return std::make_shared<Conflict>(positionsAtThisTimestep[positionOfAgentAtTimestep], a.first, positionOfAgentAtTimestep, t);
            } else {
                positionsAtThisTimestep[positionOfAgentAtTimestep] = a.first;
            }
        }
    }
    // Edge conflict
    for (int t = 0; t < numberOfTimesteps-1; t++){
        std::unordered_map<std::pair<int, int>, int, PairHasher<int>, PairEquality<int>> edgesAtThisTimestep;
        for (const auto& a : solution){
            auto pathOfAgent = a.second;
            if (edgesAtThisTimestep.count({pathOfAgent[t], pathOfAgent[t + 1]})>0){
                return std::make_shared<Conflict>(edgesAtThisTimestep[{pathOfAgent[t], pathOfAgent[t + 1]}], a.first, pathOfAgent[t], pathOfAgent[t + 1], t+1);
            } else {
                edgesAtThisTimestep[{pathOfAgent[t + 1], pathOfAgent[t]}] = a.first;
            }
        }
    }
    return nullptr;
}

std::shared_ptr<Solution> ConflictBasedSearch::combineSolutions(std::shared_ptr<ConflictTreeNode> node, int numberOfVisitedNodes) {
    int numberOfTimesteps = node->getSolution().begin()->second.size();
    return std::make_shared<Solution>(node->getCost(), numberOfVisitedNodes, numberOfTimesteps, node->getSolution());
}

std::shared_ptr<Solution> ConflictBasedSearch::solve() {
    LOG("===== Conflict Based Search ====");

    std::set<VertexConstraint> setOfVertexConstraints;
    std::set<EdgeConstraint> setOfEdgeConstraints;

    auto [solution, cost, costs] = planIndividualPaths();
    if (cost == -1){
        return {};
    }

    // TODO : implement numberOfConflicts for each solution and keep updated a list of conflicts in the current solution
    // int numberOfConflicts;

    fringe.insert(std::make_shared<ConflictTreeNode>(setOfVertexConstraints, setOfEdgeConstraints, solution, costs, cost));

    int numberOfVisitedNodes = 0;

    while (!fringe.empty()){
        auto it = fringe.begin();
        const auto node = *it;

        fringe.erase(it);

        numberOfVisitedNodes += 1;

        auto conflict = findAConflict(node);

        if (conflict == nullptr){
            return combineSolutions(node, numberOfVisitedNodes);
        }

        conflict->print();

        int agent1 = conflict->getAgent1();
        int agent2 = conflict->getAgent2();

        for (int agentId : {agent1, agent2}){
            auto successorSetOfVertexConstraints = node->getSetOfVertexConstraints();
            auto successorSetOfEdgeConstraints = node->getSetOfEdgeConstraints();
            if (conflict->isVertexConflict()){
                successorSetOfVertexConstraints.insert({agentId, conflict->getPosition1(), conflict->getTime()});
            } else { // edge conflict
                successorSetOfEdgeConstraints.insert({agentId, conflict->getPosition1(), conflict->getPosition2(), conflict->getTime()});
                successorSetOfEdgeConstraints.insert({agentId, conflict->getPosition2(), conflict->getPosition1(), conflict->getTime()});
            }
            auto successorSolution = node->getSolution();
            auto prob = std::make_shared<SingleAgentProblemWithConstraints>(problem->getGraph(), problem->getStartOf(agentId), problem->getTargetOf(agentId), problem->getObjFunction(), agentId, successorSetOfVertexConstraints, successorSetOfEdgeConstraints);
            auto solution = AStar<SingleAgentProblemWithConstraints, SingleAgentSpaceTimeState>(prob, typeOfHeuristic).solve();
            auto successorCosts = node->getCosts();
            successorCosts[agentId] = solution->getCost();
            successorSolution[agentId] = solution->getPathOfAgent(agentId);
            int successorCost;
            if (problem->getObjFunction() == Makespan){
                successorCost = 0;
                for (auto a : successorCosts){
                    successorCost = std::max(successorCost, a.second);
                }
            } else {
                successorCost = node->getCost() - node->getCosts()[agentId] + solution->getCost();
            }
            // auto successorNumberOfConflicts;
            if (solution->getFoundPath()){
                fringe.insert(std::make_shared<ConflictTreeNode>(successorSetOfVertexConstraints, successorSetOfEdgeConstraints, successorSolution, successorCosts, successorCost));
            }
        }
    }

    LOG("No path has been found.");
    return {};
}
