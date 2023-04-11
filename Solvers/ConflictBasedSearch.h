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

    std::tuple<std::unordered_map<int, std::vector<int>>,int, std::unordered_map<int, int>> planIndividualPaths();
    std::tuple<bool, bool, int, int, int, int, int> findAConflict(std::shared_ptr<ConflictTreeNode> node);
    std::shared_ptr<Solution> combineSolutions(std::shared_ptr<ConflictTreeNode> node, int numberOfVisitedNodes);
};


#endif //TFE_MAPF_CONFLICTBASEDSEARCH_H
