//
// Created by Arthur Mahy on 16/05/2023.
//

#ifndef TFE_MAPF_INDEPENDENCEDETECTION_H
#define TFE_MAPF_INDEPENDENCEDETECTION_H

#include "SimpleIndependenceDetection.h"
#include "AlreadyConflictedBeforeSet.h"

// Independence Detection search (with an updated set of conflicts)
// - with a Conflict Avoidance Table (CAT) when replanning
// - Enhanced version of ID (EID) : tries to resolve a conflict (by attempting to replan one group to avoid the plan of the other group) before merging the groups
//
// Only for multi agent problem
//
// typeOfHeuristic is the heuristic for the A* searches
class IndependenceDetection : SimpleIndependenceDetection {
public:
    IndependenceDetection(std::shared_ptr<MultiAgentProblemWithConstraints> problem, TypeOfHeuristic typeOfHeuristic);
    std::shared_ptr<Solution> solve();
private:
    std::unordered_set<std::set<std::shared_ptr<Group>, PointerGroupEquality>, SetOfPointersHasher, SetOfPointersEquality> alreadyConflictedBefore;

    // Find another optimal solution for groupA
    // - with the same cost as the previous one
    // - avoiding the solution of groupB with an illegal move table
    // Returns true if success (false otherwise)
    bool replanGroupAAvoidingGroupB(std::shared_ptr<Group> groupA, std::shared_ptr<Group> groupB);

    bool mergeGroupsAndPlanNewGroup(std::shared_ptr<Group> groupA, std::shared_ptr<Group> groupB);
};


#endif //TFE_MAPF_INDEPENDENCEDETECTION_H
