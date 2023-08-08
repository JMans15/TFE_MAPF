//! Basic A* search
#ifndef TFE_MAPF_GENERALASTAR_H
#define TFE_MAPF_GENERALASTAR_H

#include "AStar.h"

#include "../../Problems/Problem.h"
#include <utility>

/**
 * Can be applied for multi agent and single agent problems
 *
 * Only takes in account negative constraints of problem
 */
class GeneralAStar {
public:
  //! Constructor for the general A* search
  //! @param [in] typeOfHeuristic TypeOfHeuristic to use
  //! @param [in] spaceTimeSearch true if space-time search, false if simple
  //! space search
  //! @param [in] operatorDecomposition true if OD is to be used
  GeneralAStar(TypeOfHeuristic typeOfHeuristic, bool spaceTimeSearch,
               bool operatorDecomposition = true)
      : typeOfHeuristic(typeOfHeuristic), fixedParameters(true),
        spaceTimeSearch(spaceTimeSearch),
        operatorDecomposition(operatorDecomposition) {}

  //! Constructor for the general A* search
  //! Uses determines whether space-time search should be used depending on the
  //! presence of external constraints, shouldn't be used by the user
  //! @param [in] typeOfHeuristic TypeOfHeuristic to use
  GeneralAStar(TypeOfHeuristic typeOfHeuristic)
      : typeOfHeuristic(typeOfHeuristic), fixedParameters(false) {
    operatorDecomposition = true;
  }

  //! Launches a multi-agent search and returns the Solution
  //! @param [in] problem Problem to solve
  std::shared_ptr<Solution>
  solve(const std::shared_ptr<MultiAgentProblem> &problem) {
    if (not fixedParameters) {
      if (problem->hasExternalConstraints()) {
        spaceTimeSearch = true;
      } else {
        spaceTimeSearch = false;
      }
    }
    if (problem->isImpossible()) {
      return std::make_shared<Solution>();
    }
    if (spaceTimeSearch) {
      if (operatorDecomposition) {
        return AStar<ODMultiAgentAStarProblemWithConstraints,
                     ODMultiAgentSpaceTimeState>(
                   std::make_shared<ODMultiAgentAStarProblemWithConstraints>(
                       problem),
                   typeOfHeuristic)
            .solve();
      } else {
        return AStar<StandardMultiAgentAStarProblemWithConstraints,
                     StandardMultiAgentSpaceTimeState>(
                   std::make_shared<
                       StandardMultiAgentAStarProblemWithConstraints>(problem),
                   typeOfHeuristic)
            .solve();
      }
    } else {
      if (operatorDecomposition) {
        return AStar<ODMultiAgentAStarProblem, ODMultiAgentState>(
                   std::make_shared<ODMultiAgentAStarProblem>(problem),
                   typeOfHeuristic)
            .solve();
      } else {
        return AStar<StandardMultiAgentAStarProblem, StandardMultiAgentState>(
                   std::make_shared<StandardMultiAgentAStarProblem>(problem),
                   typeOfHeuristic)
            .solve();
      }
    }
  }

  //! Launches a single-agent search and returns the Solution
  //! @param [in] problem Problem to solve
  std::shared_ptr<Solution>
  solve(const std::shared_ptr<SingleAgentProblem> &problem) {
    if (not fixedParameters) {
      if (problem->hasExternalConstraints()) {
        spaceTimeSearch = true;
      } else {
        spaceTimeSearch = false;
      }
    }
    if (problem->isImpossible()) {
      return std::make_shared<Solution>();
    }
    if (spaceTimeSearch) {
      return AStar<SingleAgentAStarProblemWithConstraints,
                   SingleAgentSpaceTimeState>(
                 std::make_shared<SingleAgentAStarProblemWithConstraints>(
                     problem),
                 typeOfHeuristic)
          .solve();
    } else {
      return AStar<SingleAgentAStarProblem, SingleAgentState>(
                 std::make_shared<SingleAgentAStarProblem>(problem),
                 typeOfHeuristic)
          .solve();
    }
  }

private:
  TypeOfHeuristic typeOfHeuristic; //!< Type of heuristic to be used
  bool fixedParameters; //!< false when space-time search has to be used
                        //!< automatically in presence of external constraints
  bool spaceTimeSearch; //!< SpaceTimeSearch or SpaceSearch (for single and
                        //!< multi)
  bool operatorDecomposition; //!< OperatorDecomposition or Standard (for multi)
};

#endif // TFE_MAPF_GENERALASTAR_H
