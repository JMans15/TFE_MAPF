//! Independence Detection search

#ifndef TFE_MAPF_INDEPENDENCEDETECTION_H
#define TFE_MAPF_INDEPENDENCEDETECTION_H

#include "AlreadyConflictedBeforeSet.h"
#include "SimpleIndependenceDetection.h"
/**
 * Enhanced version of ID (also called EID) : tries to resolve a conflict (by
 * attempting to replan one group to avoid the plan of the other group) before
 * merging the groups
 *
 * Only takes in account negative constraints of problem
 *
 * Variable low-level search algorithm
 * Toggelable CAT enhancement
 */
template <class MultiAgentSolver>
class IndependenceDetection : SimpleIndependenceDetection<MultiAgentSolver> {
public:
  /**
   * Constructor for the ID search with defined problem
   * @param [in] problem The problem to solve
   * @param [in] CAT Whether or not to use a Conflict Avoidance Table (CAT) when
   * replanning to avoid planned paths
   * @param [in] lowLevelSearch Low level search solver to be used
   */
  IndependenceDetection(std::shared_ptr<MultiAgentProblem> problem,
                        std::shared_ptr<MultiAgentSolver> lowLevelSearch,
                        bool CAT = true);
  /**
   * Starts the algorithm and eventually returns found solution
   * @return The found solution
   */
  std::shared_ptr<Solution> solve();

  /**
   * An alternative constructor with no bound problem, the latter being
   * specified in the solve(std::shared_ptr<MultiAgentProblem> problem)
   */
  explicit IndependenceDetection(
      std::shared_ptr<MultiAgentSolver> lowLevelSearch, bool CAT = true);

  /** Starts the algorithm for a given problem and returns
   *  the solution
   *  @param [in] problem The problem to be solved
   *  @return The found solution
   */
  std::shared_ptr<Solution> solve(std::shared_ptr<MultiAgentProblem> problem);

private:
  std::unordered_set<std::set<std::shared_ptr<Group>, PointerGroupEquality>,
                     SetOfPointersHasher, SetOfPointersEquality>
      alreadyConflictedBefore; /**< A custom ordered set containing the previous
                                  conflicts */

  // Find another optimal solution for groupA
  // - with the same cost as the previous one
  // - avoiding the solution of groupB with an illegal move table
  // Returns true if success (false otherwise)
  bool replanGroupAAvoidingGroupB(
      const std::shared_ptr<Group> &groupA,
      const std::shared_ptr<Group>
          &groupB); /**<
                     * Searches for another optimal solution for groupA
                     * - with the same cost as the previous one
                     * - avoiding the solution of groupB with an illegal move
                     * table
                     * @param [in] groupA The group for which to search for an
                     * alternative optimal solution
                     * @param [in] groupB The group that groupA has now to avoid
                     * @return true if success, false otherwise
                     */

  using SimpleIndependenceDetection<MultiAgentSolver>::problem;
  using SimpleIndependenceDetection<MultiAgentSolver>::lowLevelSearch;
  using SimpleIndependenceDetection<MultiAgentSolver>::CAT;
  using SimpleIndependenceDetection<
      MultiAgentSolver>::numberOfResolvedConflicts;
  using SimpleIndependenceDetection<
      MultiAgentSolver>::vertexConflictAvoidanceTable;
  using SimpleIndependenceDetection<
      MultiAgentSolver>::edgeConflictAvoidanceTable;
  using SimpleIndependenceDetection<MultiAgentSolver>::setOfConflicts;
  using SimpleIndependenceDetection<MultiAgentSolver>::groups;
  using SimpleIndependenceDetection<MultiAgentSolver>::planSingletonGroups;
  using SimpleIndependenceDetection<MultiAgentSolver>::calculateSetOfConflicts;
  using SimpleIndependenceDetection<
      MultiAgentSolver>::mergeGroupsAndPlanNewGroup;
  using SimpleIndependenceDetection<MultiAgentSolver>::combineSolutions;
};

#endif // TFE_MAPF_INDEPENDENCEDETECTION_H
