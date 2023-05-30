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
            if (pathA.size() < pathB.size()){
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
            std::set<VertexConstraint> vertexConflictAvoidanceTable = problem->getSetOfSoftVertexConstraints();
            std::set<EdgeConstraint> edgeConflictAvoidanceTable = problem->getSetOfSoftEdgeConstraints();
            if (CAT){
                for (auto [agent, pathOfAgent] : fullSolutions){
                    if (agent != agentId){
                        for (int t = 0; t < pathOfAgent.size(); t++){
                            vertexConflictAvoidanceTable.insert({agentId, pathOfAgent[t], t});
                        }
                        for (int t = 1; t < pathOfAgent.size(); t++){
                            edgeConflictAvoidanceTable.insert({agentId, pathOfAgent[t], pathOfAgent[t-1], t});
                            edgeConflictAvoidanceTable.insert({agentId, pathOfAgent[t-1], pathOfAgent[t], t});
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
        std::set<VertexConstraint> vertexConflictAvoidanceTable = problem->getSetOfSoftVertexConstraints();
        std::set<EdgeConstraint> edgeConflictAvoidanceTable = problem->getSetOfSoftEdgeConstraints();
        if (CAT){
            for (auto [agent, pathOfAgent] : fullSolutions){
                if (agent != agentId){
                    for (int t = 0; t < pathOfAgent.size(); t++){
                        vertexConflictAvoidanceTable.insert({agentId, pathOfAgent[t], t});
                    }
                    for (int t = 1; t < pathOfAgent.size(); t++){
                        edgeConflictAvoidanceTable.insert({agentId, pathOfAgent[t], pathOfAgent[t-1], t});
                        edgeConflictAvoidanceTable.insert({agentId, pathOfAgent[t-1], pathOfAgent[t], t});
                    }
                }
            }
        }
        auto prob = std::make_shared<SingleAgentProblemWithConstraints>(problem->getGraph(), problem->getStartOf(agentId), problem->getTargetOf(agentId), problem->getObjFunction(), agentId, newFullSetOfVertexConstraints, newFullSetOfEdgeConstraints, INT_MAX, vertexConflictAvoidanceTable, edgeConflictAvoidanceTable);
        auto solution = AStar<SingleAgentProblemWithConstraints, SingleAgentSpaceTimeState>(prob, typeOfHeuristic).solve();
        if (solution->getFoundPath() and solution->isConsistent(setOfPositiveConstraints)){
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
                        for (int t = 0 ; t < solution->getPathOfAgent(agentId).size(); t++){
                            std::cout << t << " : " << solution->getPathOfAgent(agentId)[t] << std::endl;
                        }
                    } else {
                        for (int t = 0 ; t < path.size(); t++){
                            std::cout << t << " : " << path[t] << std::endl;
                        }
                    }
                }*/
                auto successorSetOfConflicts = updateSetOfConflicts(fullSolutions, node->getSetOfConflicts(), agentId, solution->getPathOfAgent(agentId));
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
        std::unordered_map<int,int> successorCosts;
        std::unordered_map<int, std::vector<int>> successorSolution;
        auto successorSetOfConflicts = node->getSetOfConflicts();
        for (auto agentK : agentsToReplan){
            vertexConflictAvoidanceTable = problem->getSetOfSoftVertexConstraints();
            edgeConflictAvoidanceTable = problem->getSetOfSoftEdgeConstraints();
            if (CAT){
                for (auto [agent, pathOfAgent] : fullSolutions){
                    if (agent != agentK){
                        for (int t = 0; t < pathOfAgent.size(); t++){
                            vertexConflictAvoidanceTable.insert({agentK, pathOfAgent[t], t});
                        }
                        for (int t = 1; t < pathOfAgent.size(); t++){
                            edgeConflictAvoidanceTable.insert({agentK, pathOfAgent[t], pathOfAgent[t-1], t});
                            edgeConflictAvoidanceTable.insert({agentK, pathOfAgent[t-1], pathOfAgent[t], t});
                        }
                    }
                }
            }
            auto prob = std::make_shared<SingleAgentProblemWithConstraints>(problem->getGraph(), problem->getStartOf(agentK), problem->getTargetOf(agentK), problem->getObjFunction(), agentK, newFullSetOfVertexConstraints, newFullSetOfEdgeConstraints, INT_MAX, vertexConflictAvoidanceTable, edgeConflictAvoidanceTable);
            auto solution = AStar<SingleAgentProblemWithConstraints, SingleAgentSpaceTimeState>(prob, typeOfHeuristic).solve();
            if (solution->getFoundPath() and solution->isConsistent(setOfPositiveConstraints)){
                successorCosts[agentK] = solution->getCost();
                successorSolution[agentK] = solution->getPathOfAgent(agentK);
                fullSolutions[agentK] = solution->getPathOfAgent(agentK);
                fullCosts[agentK] = solution->getCost();
                successorSetOfConflicts = updateSetOfConflicts(fullSolutions, successorSetOfConflicts, agentK, solution->getPathOfAgent(agentK));
            } else {
                break;
            }
        }
        if (not solution->getFoundPath()){
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
            }*/
            fringe.insert(std::make_shared<ConflictTreeNode>(successorEdgeConstraint, successorVertexConstraint, successorSolution, successorCosts, successorCost, node, successorSetOfConflicts));
        }

    }
    LOG("No path has been found.");
    return std::make_shared<Solution>();
}
