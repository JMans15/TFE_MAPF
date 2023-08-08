//
// Created by Arthur Mahy on 27/02/2023.
//

#ifndef TFE_MAPF_REVERSERESUMABLEASTAR_H
#define TFE_MAPF_REVERSERESUMABLEASTAR_H

#include "../../AStarProblems/SingleAgentAStarProblem.h"
#include "../../Heuristics/HeuristicManhattan.h"
#include "Node.h"

#include <queue>
#include <unordered_map>
#include <unordered_set>

template <class S> class Heuristic;

//! Reverse Resumable A* search
//!
//! Only for single agent problem
//!
//! Consists of a single agent search where the beginning of the search is the
//! goal state (target position) of the problem
//!
//! Possibility to continue the search (with the resume method) even when the
//! start state of the problem is goal tested
//! (https://www.davidsilver.uk/wp-content/uploads/2020/03/coop-path-AIWisdom.pdf)
//!
//! We don't take into account the maxCost attribute of problem
class ReverseResumableAStar {
public:
  //! Constructor for RRA* solver
  //! @param [in] problem Problem to solve
  explicit ReverseResumableAStar(
      const std::shared_ptr<SingleAgentAStarProblem> &problem);

  //! Resumes the search from target to start until state is met
  //! @return the optimal cost between goal state and state
  //! @param [in] state SingleAgentState to pause at
  int resume(const std::shared_ptr<SingleAgentState> &state);

  //! Returns the optimal distance (walls are taken into account) between
  //! position and target
  //! @param [in] position Start position from where to compute the cost of the
  //! path to the target
  int optimalDistance(int position);

  //! Returns a hash map of the distance for each state
  std::unordered_map<std::shared_ptr<SingleAgentState>, int,
                     StateHasher<SingleAgentState>,
                     StateEquality<SingleAgentState>>
  getDistance();

private:
  //! Problem to solve
  std::shared_ptr<SingleAgentAStarProblem> problem;
  //! Heuristic to use
  std::shared_ptr<Heuristic<SingleAgentState>> heuristic;
  //! Fringe used in A*
  std::multiset<std::shared_ptr<Node<SingleAgentState>>,
                NodeComparator<SingleAgentState>>
      fringe;
  //! Map of the distances from each state to the target
  std::unordered_map<std::shared_ptr<SingleAgentState>, int,
                     StateHasher<SingleAgentState>,
                     StateEquality<SingleAgentState>>
      distance;
  //! Closed set used in A*
  std::unordered_set<std::shared_ptr<SingleAgentState>,
                     StateHasher<SingleAgentState>,
                     StateEquality<SingleAgentState>>
      closed;
};

#endif // TFE_MAPF_REVERSERESUMABLEASTAR_H
