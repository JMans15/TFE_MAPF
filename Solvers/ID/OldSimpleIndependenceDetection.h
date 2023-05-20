//
// Created by Arthur Mahy on 27/03/2023.
//

#ifndef TFE_MAPF_OLDSIMPLEINDEPENDENCEDETECTION_H
#define TFE_MAPF_OLDSIMPLEINDEPENDENCEDETECTION_H

#include "../../Problems/MultiAgentProblemWithConstraints.h"
#include "../AStar/AStar.h"
#include "Group.h"

// Simple Independence Detection search
// Only for multi agent problem
//
// typeOfHeuristic is the heuristic for the A* searches
class OldSimpleIndependenceDetection {
public:
    OldSimpleIndependenceDetection(std::shared_ptr<MultiAgentProblemWithConstraints> problem, TypeOfHeuristic typeOfHeuristic);

    virtual std::shared_ptr<Solution> solve();

protected:
    std::shared_ptr<MultiAgentProblemWithConstraints> problem;
    TypeOfHeuristic typeOfHeuristic;
    std::unordered_set<std::shared_ptr<Group>, GroupHasher, GroupEquality> groups;

    // Plans a path for each singleton group
    // Returns true if it found a solution for each agent (false otherwise)
    bool planSingletonGroups();

    // Returns (true, a, b) if there's a conflict between the paths of group a and group b
    // and (false, _, _) otherwise
    std::tuple<bool, std::shared_ptr<Group>, std::shared_ptr<Group>> findAConflict();

    // Merges groupA and groupB and plan a path for the new group
    // Returns true if it found a valid solution for the new group (false otherwise)
    virtual bool mergeGroupsAndPlanNewGroup(std::shared_ptr<Group> groupA, std::shared_ptr<Group> groupB);

    std::shared_ptr<Solution> combineSolutions();
};


#endif //TFE_MAPF_OLDSIMPLEINDEPENDENCEDETECTION_H
