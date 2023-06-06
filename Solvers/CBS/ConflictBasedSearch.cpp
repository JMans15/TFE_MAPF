//
// Created by Arthur Mahy on 10/04/2023.
//

#include "ConflictBasedSearch.h"

#include <utility>

ConflictBasedSearch::ConflictBasedSearch(std::shared_ptr<MultiAgentProblemWithConstraints> problem, TypeOfHeuristic typeOfHeuristic, bool CAT, bool disjointSplitting)
    : problem(std::move(problem))
    , typeOfHeuristic(typeOfHeuristic)
    , numberOfVisitedNodes(0)
    , CAT(CAT)
    , disjointSplitting(disjointSplitting)
    {}

std::tuple<std::unordered_map<int, std::vector<int>>,int, std::unordered_map<int, int>> ConflictBasedSearch::planIndividualPaths() {
    std::unordered_map<int, std::vector<int>> solutions;
    std::unordered_map<int, int> costs;
    int cost = 0;
    SoftEdgeConstraintsMultiSet edgeConflictAvoidanceTable = problem->getSetOfSoftEdgeConstraints();
    SoftVertexConstraintsMultiSet vertexConflictAvoidanceTable = problem->getSetOfSoftVertexConstraints();
    for (int i = 0; i < problem->getNumberOfAgents(); i++){
        int agentId = problem->getAgentIds()[i];
        auto prob = std::make_shared<SingleAgentProblemWithConstraints>(problem->getGraph(), problem->getStarts()[i], problem->getTargets()[i], problem->getObjFunction(), agentId, problem->getSetOfHardVertexConstraints(), problem->getSetOfHardEdgeConstraints(), INT_MAX, vertexConflictAvoidanceTable, edgeConflictAvoidanceTable);
        auto solution = AStar<SingleAgentProblemWithConstraints, SingleAgentSpaceTimeState>(prob, typeOfHeuristic).solve();
        if (not solution->getFoundPath()){
            return {solutions, -1, costs};
        }
        if (CAT){
            vector<int> pathOfAgent = solution->getPathOfAgent(agentId);
            for (int t = 0; t < pathOfAgent.size(); t++){
                vertexConflictAvoidanceTable.insert({agentId, pathOfAgent[t], t});
            }
            for (int t = 1; t < pathOfAgent.size(); t++){
                edgeConflictAvoidanceTable.insert({agentId, pathOfAgent[t], pathOfAgent[t-1], t});
                // edgeConflictAvoidanceTable.insert({agentId, pathOfAgent[t-1], pathOfAgent[t], t});
            }
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
            if (agentA!=agentB and pathA.size() <= pathB.size()){
                // Vertex conflict
                for (int t = 0; t < pathA.size(); t++){
                    if (pathA[t]==pathB[t]){
                        setOfConflicts.insert({agentA, agentB, pathA[t], t});
                    }
                }
                int targetA = problem->getTargetOf(agentA);
                for (int t = pathA.size(); t < pathB.size(); t++){
                    if (targetA==pathB[t]){
                        setOfConflicts.insert({agentA, agentB, targetA, t});
                    }
                }
                // Edge conflict
                for (int t = 0; t < pathA.size()-1; t++){
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

std::set<AgentConflict> ConflictBasedSearch::updateSetOfConflicts(const std::unordered_map<int, std::vector<int>>& fullSolutions, const std::set<AgentConflict>& setOfConflicts, int agentId, std::vector<int> newPath){
    std::set<AgentConflict> successorSetOfConflicts;
    for (const auto& conflict : setOfConflicts){
        if (conflict.getAgent1()!=agentId and conflict.getAgent2()!=agentId){
            successorSetOfConflicts.insert(conflict);
        }
    }
    for (auto [agentA, pathA] : fullSolutions){
        if (agentA != agentId){
            // Vertex conflict
            if (pathA.size() < newPath.size()){
                for (int t = 0; t < pathA.size(); t++){
                    if (pathA[t]==newPath[t]){
                        successorSetOfConflicts.insert({agentA, agentId, pathA[t], t});
                    }
                }
                int targetA = problem->getTargetOf(agentA);
                for (int t = pathA.size(); t < newPath.size(); t++){
                    if (targetA==newPath[t]){
                        successorSetOfConflicts.insert({agentA, agentId, targetA, t});
                    }
                }
            } else {
                for (int t = 0; t < newPath.size(); t++){
                    if (pathA[t]==newPath[t]){
                        successorSetOfConflicts.insert({agentA, agentId, pathA[t], t});
                    }
                }
                int target = problem->getTargetOf(agentId);
                for (int t = newPath.size(); t < pathA.size(); t++){
                    if (pathA[t]==target){
                        successorSetOfConflicts.insert({agentA, agentId, pathA[t], t});
                    }
                }
            }
            // Edge conflict
            for (int t = 0; t < std::min(pathA.size(), newPath.size())-1; t++){
                if (pathA[t]==newPath[t+1] and pathA[t+1]==newPath[t]){
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

std::tuple<std::set<VertexConstraint>, std::set<EdgeConstraint>, std::unordered_map<int, int>, std::unordered_map<int, vector<int>>, std::set<VertexConstraint>> ConflictBasedSearch::retrieveSetsOfConstraintsAndCostsAndSolutionsDisjointSplitting(std::shared_ptr<ConflictTreeNode> node) {
    std::set<VertexConstraint> fullSetOfVertexConstraints = problem->getSetOfHardVertexConstraints();
    std::set<EdgeConstraint> fullSetOfEdgeConstraints = problem->getSetOfHardEdgeConstraints();
    std::set<VertexConstraint> setOfPositiveConstraints;
    std::unordered_map<int, int> fullCosts;
    std::unordered_map<int, std::vector<int>> fullSolutions;

    while (node) {
        if (node->getVertexConstraint()!=nullptr){
            if (node->getVertexConstraint()->isPositive()){
                for (auto agentId : problem->getAgentIds()){
                    if (agentId!=node->getVertexConstraint()->getAgent()){
                        fullSetOfVertexConstraints.insert({agentId, node->getVertexConstraint()->getPosition(), node->getVertexConstraint()->getTime()});
                    }
                }
                setOfPositiveConstraints.insert(*node->getVertexConstraint());
            } else {
                fullSetOfVertexConstraints.insert(*node->getVertexConstraint());
            }
        } else {
            if (node->getEdgeConstraint()!=nullptr){
                if (node->getEdgeConstraint()->isPositive()){
                    for (auto agentId : problem->getAgentIds()){
                        if (agentId!=node->getEdgeConstraint()->getAgent()){
                            fullSetOfEdgeConstraints.insert({agentId, node->getEdgeConstraint()->getPosition2(), node->getEdgeConstraint()->getPosition1(), node->getEdgeConstraint()->getTime()});
                            fullSetOfEdgeConstraints.insert({agentId, node->getEdgeConstraint()->getPosition1(), node->getEdgeConstraint()->getPosition2(), node->getEdgeConstraint()->getTime()});
                        }
                    }
                    setOfPositiveConstraints.insert({node->getEdgeConstraint()->getAgent(), node->getEdgeConstraint()->getPosition1(), node->getEdgeConstraint()->getTime()-1, true});
                    setOfPositiveConstraints.insert({node->getEdgeConstraint()->getAgent(), node->getEdgeConstraint()->getPosition2(), node->getEdgeConstraint()->getTime(), true});
                } else {
                    fullSetOfEdgeConstraints.insert(*node->getEdgeConstraint());
                }
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

    return {fullSetOfVertexConstraints, fullSetOfEdgeConstraints, fullCosts, fullSolutions, setOfPositiveConstraints};
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
    return std::make_shared<Solution>(node->getCost(), numberOfVisitedNodes, numberOfTimesteps, solutions,  numberOfNodesLeftInTheFringe);
}

std::shared_ptr<Solution> ConflictBasedSearch::solve() {
    LOG("===== Conflict Based Search ====");

    if (disjointSplitting){
        return disjointSplittingSolve();
    }

    if (problem->isImpossible()){
        return std::make_shared<Solution>();
    }

    auto [solution, cost, costs] = planIndividualPaths();
    if (cost == -1){
        return std::make_shared<Solution>();
    }

    // solution[1] = std::vector<int>{1,2,3,8,13};

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
            numberOfNodesLeftInTheFringe = (int) fringe.size();
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
            SoftVertexConstraintsMultiSet vertexConflictAvoidanceTable = problem->getSetOfSoftVertexConstraints();
            SoftEdgeConstraintsMultiSet edgeConflictAvoidanceTable = problem->getSetOfSoftEdgeConstraints();
            if (CAT){
                for (auto [agent, pathOfAgent] : fullSolutions){
                    if (agent != agentId){
                        for (int t = 0; t < pathOfAgent.size(); t++){
                            vertexConflictAvoidanceTable.insert({agent, pathOfAgent[t], t});
                        }
                        for (int t = 1; t < pathOfAgent.size(); t++){
                            edgeConflictAvoidanceTable.insert({agent, pathOfAgent[t], pathOfAgent[t-1], t});
                            // edgeConflictAvoidanceTable.insert({agent, pathOfAgent[t-1], pathOfAgent[t], t});
                        }
                    }
                }
            }
            auto prob = std::make_shared<SingleAgentProblemWithConstraints>(problem->getGraph(), problem->getStartOf(agentId), problem->getTargetOf(agentId), problem->getObjFunction(), agentId, newFullSetOfVertexConstraints, newFullSetOfEdgeConstraints, INT_MAX, vertexConflictAvoidanceTable, edgeConflictAvoidanceTable);
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
                    auto successorSetOfConflicts = updateSetOfConflicts(fullSolutions, node->getSetOfConflicts(), agentId, solution->getPathOfAgent(agentId));
                    /*std::cout << "we add a node" << std::endl;
                    for (auto constraint : newFullSetOfEdgeConstraints){
                        constraint.print();
                    }
                    for (auto constraint : newFullSetOfVertexConstraints){
                        constraint.print();
                    }
                    for (auto[agent, path] : fullSolutions){
                        std::cout << "path of " << agent << std::endl;
                        if (agent==agentId){
                            for (int t = 0 ; t < successorSolution[agentId].size(); t++){
                                std::cout << t << " : " << successorSolution[agentId][t] << std::endl;
                            }
                        } else {
                            for (int t = 0 ; t < path.size(); t++){
                                std::cout << t << " : " << path[t] << std::endl;
                            }
                        }
                    }
                    for (auto conflict : successorSetOfConflicts){
                        conflict.print();
                    }*/
                    fringe.insert(std::make_shared<ConflictTreeNode>(successorEdgeConstraint, successorVertexConstraint, successorSolution, successorCosts, successorCost, node, successorSetOfConflicts));
                }
            }
        }
    }
    LOG("No path has been found.");
    return std::make_shared<Solution>();
}

std::shared_ptr<Solution> ConflictBasedSearch::disjointSplittingSolve() {

    if (problem->isImpossible()){
        return std::make_shared<Solution>();
    }

    auto [solution, cost, costs] = planIndividualPaths();
    if (cost == -1){
        return std::make_shared<Solution>();
    }

    // solution[1] = std::vector<int>{1,2,3,8,13};

    std::set<AgentConflict> setOfConflicts = calculateSetOfConflicts(solution);

    if (cost <= problem->getMaxCost()){
        /*std::cout << "we add a node" << std::endl;
        for (auto[agent, path] : solution){
            std::cout << "path of " << agent << std::endl;
            for (int t = 0 ; t < path.size(); t++){
                std::cout << t << " : " << path[t] << std::endl;
            }
        }
        for (auto conflict : setOfConflicts){
            conflict.print();
        }*/
        fringe.insert(std::make_shared<ConflictTreeNode>(nullptr, nullptr, solution, costs, cost, nullptr, setOfConflicts));
    }

    while (!fringe.empty()){
        auto it = fringe.begin();
        const auto node = *it;
        fringe.erase(it);

        numberOfVisitedNodes += 1;

        if (node->getSetOfConflicts().empty()){
            numberOfNodesLeftInTheFringe = (int) fringe.size();
            return combineSolutions(node);
        }

        auto conflict = *node->getSetOfConflicts().begin();

        int agent1 = conflict.getAgent1();
        int agent2 = conflict.getAgent2();

        auto [fullSetOfVertexConstraints, fullSetOfEdgeConstraints, fullCosts, fullSolutions, setOfPositiveConstraints] = retrieveSetsOfConstraintsAndCostsAndSolutionsDisjointSplitting(node);

        /*std::cout << "we pull a node" << std::endl;
        std::cout << node->getCost() << std::endl;
        for (auto constraint : fullSetOfEdgeConstraints){
            constraint.print();
        }
        for (auto constraint : fullSetOfVertexConstraints){
            constraint.print();
        }
        for (auto constraint : setOfPositiveConstraints){
            constraint.print();
        }
        for (auto[agent, path] : fullSolutions){
            std::cout << "path of " << agent << std::endl;
            for (int t = 0 ; t < path.size(); t++){
                std::cout << t << " : " << path[t] << std::endl;
            }
        }
        for (auto conflict : node->getSetOfConflicts()){
            conflict.print();
        }*/

        int agentId = (std::rand()<RAND_MAX/2) ? agent1 : agent2;; // choose random agent
        // int agentId = agent1;

        // Branch 1 : negative constraint
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
        SoftVertexConstraintsMultiSet vertexConflictAvoidanceTable = problem->getSetOfSoftVertexConstraints();
        SoftEdgeConstraintsMultiSet edgeConflictAvoidanceTable = problem->getSetOfSoftEdgeConstraints();
        if (CAT){
            for (auto [agent, pathOfAgent] : fullSolutions){
                if (agent != agentId){
                    for (int t = 0; t < pathOfAgent.size(); t++){
                        vertexConflictAvoidanceTable.insert({agent, pathOfAgent[t], t});
                    }
                    for (int t = 1; t < pathOfAgent.size(); t++){
                        edgeConflictAvoidanceTable.insert({agent, pathOfAgent[t], pathOfAgent[t-1], t});
                        // edgeConflictAvoidanceTable.insert({agent, pathOfAgent[t-1], pathOfAgent[t], t});
                    }
                }
            }
        }

        // Find the interval to replan between 2 landmarks
        int t1;
        int t2;
        int position1;
        int position2;
        auto next = setOfPositiveConstraints.lower_bound(VertexConstraint{agentId, 0, conflict.getTime()});
        if (next != setOfPositiveConstraints.end() and next->getAgent()==agentId){
            t2 = next->getTime();
            position2 = next->getPosition();
        } else {
            t2 = INT_MAX;
            position2 = problem->getTargetOf(agentId);
        }
        if (next != setOfPositiveConstraints.begin()){
            auto previous = --next;
            if (previous->getAgent()==agentId){
                t1 = previous->getTime();
                position1 = previous->getPosition();
            } else {
                t1 = 0;
                position1 = problem->getStartOf(agentId);
            }
        } else {
            t1 = 0;
            position1 = problem->getStartOf(agentId);
        }
        auto prob = std::make_shared<SingleAgentProblemWithConstraints>(problem->getGraph(), position1, position2, Makespan, agentId, newFullSetOfVertexConstraints, newFullSetOfEdgeConstraints, INT_MAX, vertexConflictAvoidanceTable, edgeConflictAvoidanceTable, t1);
        auto solution = AStar<SingleAgentProblemWithConstraints, SingleAgentSpaceTimeState>(prob, typeOfHeuristic).solve();
        if (solution->getFoundPath() and t1+solution->getMakespanCost()<=t2 and okForHardConstraints(newFullSetOfVertexConstraints, t1+solution->getMakespanCost()+1, t2-1, position2, agentId)){
            std::unordered_map<int, std::vector<int>> successorSolution;
            successorSolution[agentId] = fullSolutions[agentId];
            auto newPath = solution->getPathOfAgent(agentId);
            if (t2!=INT_MAX){
                for (int t = 1; t < newPath.size(); t++){
                    successorSolution[agentId][t+t1] = newPath[t];
                }
                if (t1+solution->getMakespanCost()<t2){
                    for (int t = t1+newPath.size()-1; t < t2; t++){
                        successorSolution[agentId][t]=position2;
                    }
                }
            } else {
                // Last interval
                for (int t = 1; t+t1 < successorSolution[agentId].size(); t++){
                    successorSolution[agentId][t+t1] = newPath[t];
                }
                for (int t = successorSolution[agentId].size()-t1; t < newPath.size(); t++){
                    successorSolution[agentId].emplace_back(newPath[t]);
                }
            }
            std::unordered_map<int,int> successorCosts;
            if (problem->getObjFunction()==Fuel){
                successorCosts[agentId] = fuelCost(successorSolution[agentId]);
            } else {
                successorCosts[agentId] = makespanCost(successorSolution[agentId]);
            }
            int successorCost;
            if (problem->getObjFunction() == Makespan){
                successorCost = 0;
                for (auto a : fullCosts){
                    if (a.first == agentId){
                        successorCost = std::max(successorCost, successorCosts[agentId]);
                    } else {
                        successorCost = std::max(successorCost, a.second);
                    }
                }
            } else {
                successorCost = node->getCost() - fullCosts[agentId] + successorCosts[agentId];
            }
            if (successorCost <= problem->getMaxCost()){
                auto successorSetOfConflicts = updateSetOfConflicts(fullSolutions, node->getSetOfConflicts(), agentId, successorSolution[agentId]);
                /*std::cout << "we add a node" << std::endl;
                for (auto constraint : newFullSetOfEdgeConstraints){
                    constraint.print();
                }
                for (auto constraint : newFullSetOfVertexConstraints){
                    constraint.print();
                }
                for (auto constraint : setOfPositiveConstraints){
                    constraint.print();
                }
                for (auto[agent, path] : fullSolutions){
                    std::cout << "path of " << agent << std::endl;
                    if (agent==agentId){
                        for (int t = 0 ; t < successorSolution[agentId].size(); t++){
                            std::cout << t << " : " << successorSolution[agentId][t] << std::endl;
                        }
                    } else {
                        for (int t = 0 ; t < path.size(); t++){
                            std::cout << t << " : " << path[t] << std::endl;
                        }
                    }
                }
                for (auto conflict : successorSetOfConflicts){
                    conflict.print();
                }*/
                fringe.insert(std::make_shared<ConflictTreeNode>(successorEdgeConstraint, successorVertexConstraint, successorSolution, successorCosts, successorCost, node, successorSetOfConflicts));
            }
        }

        // Branch 2 : positive constraint
        newFullSetOfVertexConstraints = fullSetOfVertexConstraints;
        newFullSetOfEdgeConstraints = fullSetOfEdgeConstraints;
        if (conflict.isVertexConflict()){
            successorEdgeConstraint = nullptr;
            successorVertexConstraint = std::make_shared<VertexConstraint>(agentId, conflict.getPosition1(), conflict.getTime(), true);
            for (auto agent : problem->getAgentIds()){
                if (agent!=agentId){
                    newFullSetOfVertexConstraints.insert({agent, conflict.getPosition1(), conflict.getTime()});
                }
            }
            setOfPositiveConstraints.insert({agentId, conflict.getPosition1(), conflict.getTime(), true});
        } else { // edge conflict
            successorVertexConstraint = nullptr;
            if (agentId==agent1){
                successorEdgeConstraint = std::make_shared<EdgeConstraint>(agentId, conflict.getPosition1(), conflict.getPosition2(), conflict.getTime(), true);
                for (auto agent : problem->getAgentIds()){
                    if (agent!=agentId){
                        newFullSetOfEdgeConstraints.insert({agent, conflict.getPosition2(), conflict.getPosition1(), conflict.getTime()});
                        newFullSetOfEdgeConstraints.insert({agent, conflict.getPosition1(), conflict.getPosition2(), conflict.getTime()});
                    }
                }
                setOfPositiveConstraints.insert({agentId, conflict.getPosition1(), conflict.getTime()-1, true});
                setOfPositiveConstraints.insert({agentId, conflict.getPosition2(), conflict.getTime(), true});
            } else {
                successorEdgeConstraint = std::make_shared<EdgeConstraint>(agentId, conflict.getPosition2(), conflict.getPosition1(), conflict.getTime(), true);
                for (auto agent : problem->getAgentIds()){
                    if (agent!=agentId){
                        newFullSetOfEdgeConstraints.insert({agent, conflict.getPosition1(), conflict.getPosition2(), conflict.getTime()});
                        newFullSetOfEdgeConstraints.insert({agent, conflict.getPosition2(), conflict.getPosition1(), conflict.getTime()});
                    }
                }
                setOfPositiveConstraints.insert({agentId, conflict.getPosition2(), conflict.getTime()-1, true});
                setOfPositiveConstraints.insert({agentId, conflict.getPosition1(), conflict.getTime(), true});
            }
        }
        std::set<int> agentsToReplan;
        auto oldFullSolutions = fullSolutions;
        for (auto agentK : problem->getAgentIds()) {
            if (agentK != agentId){
                if (conflict.isVertexConflict()) {
                    if (fullSolutions[agentK][conflict.getTime()]==conflict.getPosition1()){
                        agentsToReplan.insert(agentK);
                        fullSolutions.erase(agentK);
                    }
                } else {
                    if (fullSolutions[agentK][conflict.getTime()-1]==conflict.getPosition1() and fullSolutions[agentK][conflict.getTime()]==conflict.getPosition2()){
                        agentsToReplan.insert(agentK);
                        fullSolutions.erase(agentK);
                    } else if (fullSolutions[agentK][conflict.getTime()-1]==conflict.getPosition2() and fullSolutions[agentK][conflict.getTime()]==conflict.getPosition1()){
                        agentsToReplan.insert(agentK);
                        fullSolutions.erase(agentK);
                    }
                }
            }
        }
        vertexConflictAvoidanceTable = problem->getSetOfSoftVertexConstraints();
        edgeConflictAvoidanceTable = problem->getSetOfSoftEdgeConstraints();
        if (CAT){
            for (auto [agent, pathOfAgent] : fullSolutions){
                for (int t = 0; t < pathOfAgent.size(); t++){
                    vertexConflictAvoidanceTable.insert({agent, pathOfAgent[t], t});
                }
                for (int t = 1; t < pathOfAgent.size(); t++){
                    edgeConflictAvoidanceTable.insert({agent, pathOfAgent[t], pathOfAgent[t-1], t});
                    // edgeConflictAvoidanceTable.insert({agent, pathOfAgent[t-1], pathOfAgent[t], t});
                }
            }
        }
        std::unordered_map<int,int> successorCosts;
        std::unordered_map<int, std::vector<int>> successorSolution;
        auto successorSetOfConflicts = node->getSetOfConflicts();
        bool solutionOk;
        for (auto agentK : agentsToReplan){
            // Find the interval to replan between 2 landmarks
            auto next = setOfPositiveConstraints.lower_bound(VertexConstraint{agentK, 0, conflict.getTime()});
            if (next != setOfPositiveConstraints.end() and next->getAgent()==agentK){
                t2 = next->getTime();
                position2 = next->getPosition();
            } else {
                t2 = INT_MAX;
                position2 = problem->getTargetOf(agentK);
            }
            if (next != setOfPositiveConstraints.begin()){
                auto previous = --next;
                if (previous->getAgent()==agentK){
                    t1 = previous->getTime();
                    position1 = previous->getPosition();
                } else {
                    t1 = 0;
                    position1 = problem->getStartOf(agentK);
                }
            } else {
                t1 = 0;
                position1 = problem->getStartOf(agentK);
            }
            auto prob = std::make_shared<SingleAgentProblemWithConstraints>(problem->getGraph(), position1, position2, Makespan, agentK, newFullSetOfVertexConstraints, newFullSetOfEdgeConstraints, INT_MAX, vertexConflictAvoidanceTable, edgeConflictAvoidanceTable, t1);
            auto solution = AStar<SingleAgentProblemWithConstraints, SingleAgentSpaceTimeState>(prob, typeOfHeuristic).solve();
            solutionOk = solution->getFoundPath() and t1+solution->getMakespanCost()<=t2 and okForHardConstraints(newFullSetOfVertexConstraints, t1+solution->getMakespanCost()+1, t2-1, position2, agentK);
            if (solutionOk){
                successorSolution[agentK] = oldFullSolutions[agentK];
                auto newPath = solution->getPathOfAgent(agentK);
                if (t2!=INT_MAX){
                    for (int t = 1; t < newPath.size(); t++){
                        successorSolution[agentK][t+t1] = newPath[t];
                    }
                    if (t1+solution->getMakespanCost()<t2){
                        for (int t = t1+newPath.size()-1; t < t2; t++){
                            successorSolution[agentK][t]=position2;
                        }
                    }
                } else {
                    // Last interval
                    for (int t = 1; t+t1 < successorSolution[agentK].size(); t++){
                        successorSolution[agentK][t+t1] = newPath[t];
                    }
                    for (int t = successorSolution[agentK].size()-t1; t < newPath.size(); t++){
                        successorSolution[agentK].emplace_back(newPath[t]);
                    }
                }
                fullSolutions[agentK] = successorSolution[agentK];
                if (problem->getObjFunction()==Fuel){
                    successorCosts[agentK] = fuelCost(successorSolution[agentK]);
                } else {
                    successorCosts[agentK] = makespanCost(successorSolution[agentK]);
                }
                fullCosts[agentK] = successorCosts[agentK];
                successorSetOfConflicts = updateSetOfConflicts(fullSolutions, successorSetOfConflicts, agentK, successorSolution[agentK]);
                if (CAT){
                    for (int t = 0; t < fullSolutions[agentK].size(); t++){
                        vertexConflictAvoidanceTable.insert({agentK, fullSolutions[agentK][t], t});
                    }
                    for (int t = 1; t < fullSolutions[agentK].size(); t++){
                        edgeConflictAvoidanceTable.insert({agentK, fullSolutions[agentK][t], fullSolutions[agentK][t-1], t});
                        // edgeConflictAvoidanceTable.insert({agentK, fullSolutions[agentK][t-1], fullSolutions[agentK][t], t});
                    }
                }
            } else {
                break;
            }
        }
        if (not solutionOk){
            continue;
        }
        int successorCost = 0;
        if (problem->getObjFunction() == Makespan){
            for (auto a : fullCosts){
                successorCost = std::max(successorCost, a.second);
            }
        } else {
            for (auto a : fullCosts){
                successorCost += a.second;
            }
        }
        if (successorCost <= problem->getMaxCost()){
            /*std::cout << "we add a node" << std::endl;
            for (auto constraint : newFullSetOfEdgeConstraints){
                constraint.print();
            }
            for (auto constraint : newFullSetOfVertexConstraints){
                constraint.print();
            }
            for (auto constraint : setOfPositiveConstraints){
                constraint.print();
            }
            for (auto[agent, path] : fullSolutions){
                std::cout << "path of " << agent << std::endl;
                for (int t = 0 ; t < path.size(); t++){
                    std::cout << t << " : " << path[t] << std::endl;
                }
            }
            for (auto conflict : successorSetOfConflicts){
                conflict.print();
            }*/
            fringe.insert(std::make_shared<ConflictTreeNode>(successorEdgeConstraint, successorVertexConstraint, successorSolution, successorCosts, successorCost, node, successorSetOfConflicts));
        }

    }
    LOG("No path has been found.");
    return std::make_shared<Solution>();
}

bool ConflictBasedSearch::okForHardConstraints(std::set<VertexConstraint> setOfVertexConstraints, int t1, int t2,
                                               int position, int agentId) {
    if(t2+1==INT_MAX){
        return true;
    }
    if (t1<=t2){
        for (int t=t1; t<=t2; t++){
            if (setOfVertexConstraints.find({agentId, position, t}) != setOfVertexConstraints.end()){
                return false;
            }
        }
    }
    return true;
}
