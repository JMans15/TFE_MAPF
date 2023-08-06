//! Simple Independent Detection search

#ifndef TFE_MAPF_SIMPLEINDEPENDENCEDETECTION_H
#define TFE_MAPF_SIMPLEINDEPENDENCEDETECTION_H

#include "../../Problems/MultiAgentProblem.h"
#include "../../Problems/SingleAgentProblem.h"
#include "../AStar/GeneralAStar.h"
#include "../CBS/ConflictBasedSearch.h"
#include "Group.h"
#include "GroupConflict.h"

/**
 * Simple Independence Detection (also called SID)
 *
 * Only takes in account negative constraints of problem
 *
 * Variable low-level search algorithm
 * Toggelable CAT enhancement
 */
template <class MultiAgentSolver> class SimpleIndependenceDetection {
public:
  /**
   * Constructor for the SID search with defined problem
   * @param [in] problem The problem to solve
   * @param [in] CAT Whether or not to use a Conflict Avoidance Table (CAT) when
   * replanning to avoid planned paths
   * @param [in] lowLevelSearch Low level search solver to be used
   */
  SimpleIndependenceDetection(std::shared_ptr<MultiAgentProblem> problem,
                              std::shared_ptr<MultiAgentSolver> lowLevelSearch,
                              bool CAT = false);
  /**
   * Starts the algorithm and eventually returns found solution
   * @return The found solution
   */
  virtual std::shared_ptr<Solution> solve();

  /**
   * An alternative constructor with no bound problem, the latter being
   * specified in the solve(std::shared_ptr<MultiAgentProblem> problem)
   */
  explicit SimpleIndependenceDetection(
      std::shared_ptr<MultiAgentSolver> lowLevelSearch, bool CAT = false);
  /** Starts the algorithm for a given problem and returns
   *  the solution
   *  @param [in] problem The problem to be solved
   *  @return The found solution
   */
  virtual std::shared_ptr<Solution>
  solve(std::shared_ptr<MultiAgentProblem> problem);

protected:
  std::shared_ptr<MultiAgentProblem>
      problem; /**< Multi agent problem to be solved */
  std::unordered_set<std::shared_ptr<Group>, GroupHasher, GroupEquality>
      groups;                             /**< Set of travel groups */
  std::set<GroupConflict> setOfConflicts; /**< Set of conflicts */
  SoftVertexConstraintsMultiSet vertexConflictAvoidanceTable =
      problem->getSetOfSoftVertexConstraints(); /**< Set of soft vertex
                                                   constraints */
  SoftEdgeConstraintsMultiSet edgeConflictAvoidanceTable =
      problem
          ->getSetOfSoftEdgeConstraints(); /**< Set of soft edge constraints */
  bool CAT; /**< Whether to use CAT enhancement */
  std::shared_ptr<MultiAgentSolver>
      lowLevelSearch; /**< Algorithm used for the low-level searches */
  int numberOfResolvedConflicts; /**< Number of resolved conflicts so far */
  int sizeOfLargerGroup;         /**< Size of the current largest group */
  /**
   * Plans a path for each singleton group
   * @return true if it found a solution for each agent, false otherwise
   */
  bool planSingletonGroups();

  //! Calculates the set of conflicts between the paths of the current groups
  void calculateSetOfConflicts();

  //! Merges groupA and groupB, plan a path for the new group and update the set
  //! of conflicts
  //! @param [in] groupA first group to merge into the new group
  //! @param [in] groupB second group to merge into the new group
  //! @return true if it found a valid solution for the new group, false
  //! otherwise
  bool mergeGroupsAndPlanNewGroup(std::shared_ptr<Group> groupA,
                                  std::shared_ptr<Group> groupB);

  /**
   * Goes through solutions of each group and combines them to yield the overall
   * solution
   * @return The whole problem's solution
   */
  std::shared_ptr<Solution> combineSolutions();
};

#endif // TFE_MAPF_SIMPLEINDEPENDENCEDETECTION_H
