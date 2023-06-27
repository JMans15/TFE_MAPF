//
// Created by Arthur Mahy on 16/05/2023.
//

#ifndef TFE_MAPF_INDEPENDENCEDETECTION_H
#define TFE_MAPF_INDEPENDENCEDETECTION_H

#include "SimpleIndependenceDetection.h"
#include "AlreadyConflictedBeforeSet.h"

// Independence Detection search (with an updated set of conflicts)
// - Enhanced version of ID (EID) : tries to resolve a conflict (by attempting to replan one group to avoid the plan of the other group) before merging the groups
//
// Only for multi agent problem
//
// Only takes in account negative constraints of problem (what about startTime ?)
//
// If CAT is true, we use a Conflict Avoidance Table (CAT) when replanning to avoid planned paths (if possible with optimal cost)
// lowLevelSearch is the solver that will be used for the low-level searches
template <class MultiAgentSolver>
class IndependenceDetection : SimpleIndependenceDetection<MultiAgentSolver> {
public:
    IndependenceDetection(std::shared_ptr<MultiAgentProblem> problem, std::shared_ptr<MultiAgentSolver> lowLevelSearch, bool CAT = true);
    std::shared_ptr<Solution> solve();

    explicit IndependenceDetection(std::shared_ptr<MultiAgentSolver> lowLevelSearch, bool CAT = true);
    std::shared_ptr<Solution> solve(std::shared_ptr<MultiAgentProblem> problem);
private:
    std::unordered_set<std::set<std::shared_ptr<Group>, PointerGroupEquality>, SetOfPointersHasher, SetOfPointersEquality> alreadyConflictedBefore;

    // Find another optimal solution for groupA
    // - with the same cost as the previous one
    // - avoiding the solution of groupB with an illegal move table
    // Returns true if success (false otherwise)
    bool replanGroupAAvoidingGroupB(const std::shared_ptr<Group>& groupA, const std::shared_ptr<Group>& groupB);

    using SimpleIndependenceDetection<MultiAgentSolver>::problem;
    using SimpleIndependenceDetection<MultiAgentSolver>::lowLevelSearch;
    using SimpleIndependenceDetection<MultiAgentSolver>::CAT;
    using SimpleIndependenceDetection<MultiAgentSolver>::numberOfResolvedConflicts;
    using SimpleIndependenceDetection<MultiAgentSolver>::vertexConflictAvoidanceTable;
    using SimpleIndependenceDetection<MultiAgentSolver>::edgeConflictAvoidanceTable;
    using SimpleIndependenceDetection<MultiAgentSolver>::setOfConflicts;
    using SimpleIndependenceDetection<MultiAgentSolver>::groups;
    using SimpleIndependenceDetection<MultiAgentSolver>::planSingletonGroups;
    using SimpleIndependenceDetection<MultiAgentSolver>::calculateSetOfConflicts;
    using SimpleIndependenceDetection<MultiAgentSolver>::mergeGroupsAndPlanNewGroup;
    using SimpleIndependenceDetection<MultiAgentSolver>::combineSolutions;
};


#endif //TFE_MAPF_INDEPENDENCEDETECTION_H
