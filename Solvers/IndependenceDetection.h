//
// Created by Arthur Mahy on 28/02/2023.
//

#ifndef TFE_MAPF_INDEPENDENCEDETECTION_H
#define TFE_MAPF_INDEPENDENCEDETECTION_H

#include "SimpleIndependenceDetection.h"

// Independence Detection search
// Only for multi agent problem
//
// typeOfHeuristic is the heuristic for the A* searches
//
// We don't take into account the setOfConstraints attribute of problem
// We don't take into account the maxCost attribute of problem
class IndependenceDetection : SimpleIndependenceDetection {
public:
    IndependenceDetection(std::shared_ptr<MultiAgentProblem> problem, TypeOfHeuristic typeOfHeuristic);
    std::shared_ptr<Solution> solve();
private:
    std::unordered_set<std::pair<std::shared_ptr<Group>, std::shared_ptr<Group>>, PairHasher<std::shared_ptr<Group>>, PairEquality<std::shared_ptr<Group>>> alreadyConflictedBefore;
    std::set<VertexConstraint> vertexConflictAvoidanceTable;
    std::set<EdgeConstraint> edgeConflictAvoidanceTable;

    // Find another optimal solution for groupA
    // - with the same cost as the previous one
    // - avoiding the solution of groupB with an illegal move table
    // Returns true if success (false otherwise)
    bool replanGroupAAvoidingGroupB(std::shared_ptr<Group> groupA, std::shared_ptr<Group> groupB);
};


#endif //TFE_MAPF_INDEPENDENCEDETECTION_H
