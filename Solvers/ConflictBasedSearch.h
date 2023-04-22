//
// Created by Arthur Mahy on 10/04/2023.
//

#ifndef TFE_MAPF_CONFLICTBASEDSEARCH_H
#define TFE_MAPF_CONFLICTBASEDSEARCH_H

#include "../Problems/MultiAgentProblemWithConstraints.h"
#include "AStar.h"
#include "ConflictTreeNode.h"
#include "SimpleIndependenceDetection.h"

// Conflict Based Search
// - high level of the algorithm - Conflict Tree (CT) as a best first search
//
// Only for multi agent problem
//
// typeOfHeuristic is the heuristic for the A* searches
//
// We don't take into account the setOfConstraints attributes of problem
// We don't take into account the maxCost attribute of problem
class ConflictBasedSearch {
public:
    ConflictBasedSearch(std::shared_ptr<MultiAgentProblemWithConstraints> problem, TypeOfHeuristic typeOfHeuristic);
    std::shared_ptr<Solution> solve();

protected:
    std::shared_ptr<MultiAgentProblemWithConstraints> problem;
    TypeOfHeuristic typeOfHeuristic;
    std::multiset<std::shared_ptr<ConflictTreeNode>, ConflictTreeNodeComparator> fringe; // the open list
    int numberOfVisitedNodes;

    // TODO : commentaires
    // Plans a path for each agent
    // Returns a tuple {solutions, cost, costs} where
    // - solutions is a map from the id of an agent to the path of this agent
    // - cost is the cost of all the paths
    // - costs is a map from the id an agent to the cost of the path of this agent
    std::tuple<std::unordered_map<int, std::vector<int>>,int, std::unordered_map<int, int>> planIndividualPaths();
    std::set<Conflict> calculateSetOfConflicts(std::unordered_map<int, std::vector<int>> solutions);
    std::set<Conflict> updateSetOfConflicts(std::unordered_map<int, std::vector<int>> fullSolutions, std::set<Conflict> setOfConflicts, std::unordered_map<int, std::vector<int>> successorSolution);
    std::tuple<std::set<VertexConstraint>, std::set<EdgeConstraint>, std::unordered_map<int, int>, std::unordered_map<int, vector<int>>> retrieveSetsOfConstraintsAndCostsAndSolutions(std::shared_ptr<ConflictTreeNode> node);
    std::unordered_map<int, vector<int>> retrieveSolutions(std::shared_ptr<ConflictTreeNode> node);
    std::shared_ptr<Solution> combineSolutions(std::shared_ptr<ConflictTreeNode> node);
};


#endif //TFE_MAPF_CONFLICTBASEDSEARCH_H
