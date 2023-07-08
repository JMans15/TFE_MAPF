//
// Created by Arthur Mahy on 15/05/2023.
//

#ifndef TFE_MAPF_SIMPLEINDEPENDENCEDETECTION_H
#define TFE_MAPF_SIMPLEINDEPENDENCEDETECTION_H

#include "../../Problems/MultiAgentProblem.h"
#include "../../Problems/SingleAgentProblem.h"
#include "../AStar/GeneralAStar.h"
#include "../CBS/ConflictBasedSearch.h"
#include "Group.h"
#include "GroupConflict.h"

// Simple Independence Detection search (with an updated set of conflicts)
// Only for multi agent problem
//
// Only takes in account negative constraints of problem (what about startTime
// ?)
//
// If CAT is true, we use a Conflict Avoidance Table (CAT) when replanning to
// avoid planned paths (if possible with optimal cost) lowLevelSearch is the
// solver that will be used for the low-level searches
template <class MultiAgentSolver> class SimpleIndependenceDetection {
public:
  SimpleIndependenceDetection(std::shared_ptr<MultiAgentProblem> problem,
                              std::shared_ptr<MultiAgentSolver> lowLevelSearch,
                              bool CAT = false);
  virtual std::shared_ptr<Solution> solve();

  explicit SimpleIndependenceDetection(
      std::shared_ptr<MultiAgentSolver> lowLevelSearch, bool CAT = false);
  virtual std::shared_ptr<Solution>
  solve(std::shared_ptr<MultiAgentProblem> problem);

protected:
  std::shared_ptr<MultiAgentProblem> problem;
  std::unordered_set<std::shared_ptr<Group>, GroupHasher, GroupEquality> groups;
  std::set<GroupConflict> setOfConflicts;
  SoftVertexConstraintsMultiSet vertexConflictAvoidanceTable =
      problem->getSetOfSoftVertexConstraints();
  SoftEdgeConstraintsMultiSet edgeConflictAvoidanceTable =
      problem->getSetOfSoftEdgeConstraints();
  bool CAT;
  std::shared_ptr<MultiAgentSolver> lowLevelSearch;
  int numberOfResolvedConflicts;
  int sizeOfLargerGroup;

  // Plans a path for each singleton group
  // Returns true if it found a solution for each agent (false otherwise)
  bool planSingletonGroups();

  // Calculates the set of conflicts between the paths of the current groups
  void calculateSetOfConflicts();

  // Merges groupA and groupB, plan a path for the new group and update the set
  // of conflicts Returns true if it found a valid solution for the new group
  // (false otherwise)
  bool mergeGroupsAndPlanNewGroup(std::shared_ptr<Group> groupA,
                                  std::shared_ptr<Group> groupB);

  std::shared_ptr<Solution> combineSolutions();
};

#endif // TFE_MAPF_SIMPLEINDEPENDENCEDETECTION_H
