//
// Created by Arthur Mahy on 15/05/2023.
//

#ifndef TFE_MAPF_SIMPLEINDEPENDENCEDETECTION_H
#define TFE_MAPF_SIMPLEINDEPENDENCEDETECTION_H

#include "../../Problems/MultiAgentProblemWithConstraints.h"
#include "../AStar/AStar.h"
#include "Group.h"
#include "GroupConflict.h"

// Simple Independence Detection search (with an updated set of conflicts)
// Only for multi agent problem
//
// typeOfHeuristic is the heuristic for the A* searches
// If CAT is true, we use a Conflict Avoidance Table (CAT) when replanning to avoid planned paths (if possible with optimal cost)
class SimpleIndependenceDetection {
public:
    SimpleIndependenceDetection(std::shared_ptr<MultiAgentProblemWithConstraints> problem, TypeOfHeuristic typeOfHeuristic, bool CAT = false);

    virtual std::shared_ptr<Solution> solve();

protected:
    std::shared_ptr<MultiAgentProblemWithConstraints> problem;
    TypeOfHeuristic typeOfHeuristic;
    std::unordered_set<std::shared_ptr<Group>, GroupHasher, GroupEquality> groups;
    std::set<GroupConflict> setOfConflicts;
    SoftVertexConstraintsMultiSet vertexConflictAvoidanceTable = problem->getSetOfSoftVertexConstraints();
    SoftEdgeConstraintsMultiSet edgeConflictAvoidanceTable = problem->getSetOfSoftEdgeConstraints();
    bool CAT;

    // Plans a path for each singleton group
    // Returns true if it found a solution for each agent (false otherwise)
    bool planSingletonGroups();

    // Calculates the set of conflicts between the paths of the current groups
    void calculateSetOfConflicts();

    // Merges groupA and groupB, plan a path for the new group and update the set of conflicts
    // Returns true if it found a valid solution for the new group (false otherwise)
    virtual bool mergeGroupsAndPlanNewGroup(std::shared_ptr<Group> groupA, std::shared_ptr<Group> groupB);

    std::shared_ptr<Solution> combineSolutions();
};


#endif //TFE_MAPF_SIMPLEINDEPENDENCEDETECTION_H
