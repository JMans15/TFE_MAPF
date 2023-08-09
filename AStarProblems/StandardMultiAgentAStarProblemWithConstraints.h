//! MAPF problem with space-time search but no OD

#ifndef TFE_MAPF_STANDARDMULTIAGENTASTARPROBLEMWITHCONSTRAINTS_H
#define TFE_MAPF_STANDARDMULTIAGENTASTARPROBLEMWITHCONSTRAINTS_H

#include "../Problems/MultiAgentProblem.h"
#include "../States/StandardMultiAgentSpaceTimeState.h"
#include "AStarProblem.h"

//! Multi Agent Problem formulated as a search task
//! - Standard A* (>< Operator Decomposition A*)
//! - Space Time search (because of the external constraints)
class StandardMultiAgentAStarProblemWithConstraints
    : AStarProblem<StandardMultiAgentSpaceTimeState> {
public:
  //! Constructor
  //! @param [in] Base MultiAgentProblem
  StandardMultiAgentAStarProblemWithConstraints(
      std::shared_ptr<MultiAgentProblem> problem);

  std::shared_ptr<StandardMultiAgentSpaceTimeState>
  getStartState() const override;
  bool isGoalState(
      std::shared_ptr<StandardMultiAgentSpaceTimeState> state) const override;
  std::vector<
      std::tuple<std::shared_ptr<StandardMultiAgentSpaceTimeState>, int, int>>
  getSuccessors(
      std::shared_ptr<StandardMultiAgentSpaceTimeState> state) const override;
  std::unordered_map<int, std::vector<int>> getPositions(
      std::vector<std::shared_ptr<StandardMultiAgentSpaceTimeState>> states)
      const override;
  int getMaxCost() const override;
  int getStartTime() const override;

  //! Getter for base MultiAgentProblem
  std::shared_ptr<MultiAgentProblem> getProblem();

private:
  //! Base MultiAgentProblem
  std::shared_ptr<MultiAgentProblem> problem;

  //! @return true if position is not already occupied by assigned agents
  bool notAlreadyOccupiedPosition(int position, std::vector<int> &positions,
                                  int agentToAssign) const;

  //! @return true if the edge (position, positions[agentToAssign]) is not
  //! already occupied by assigned agents
  bool notAlreadyOccupiedEdge(int position, const std::vector<int> &positions,
                              int agentToAssign,
                              const std::vector<int> &prePositions) const;

  //! Recursive function used in the getSuccessors(state) method
  //! Branches on all possibles moves for agentToAssign (from 0 to
  //! numberOfAgents-1) If agentToAssign is the last agent, we add a successor
  //! to the list of successors for every possible move for agentToAssign If
  //! agentToAssign isn't the last agent, we call recursiveAssignAMoveToAnAgent
  //! for every possible move for agentToAssign
  //!
  //! positions[:agentToAssign] are the assigned positions
  //! positions[agentToAssign:] are the not yet assigned positions
  //! positions[agentToAssign] is not yet assigned but will be in this function
  //!
  //! @param [in] prePositions is the positions of the agents in state
  //! t is the timestep in state (when we add a successor to the list of
  //! successors, its timestep is t+1) cost and violations are the cost and the
  //! number of violated soft constraints by assigning the agents from 0 to
  //! agentToAssign cannotMove[i] is true if agent i is at its target position
  //! and cannot move anymore (for the SumOfCosts objective function)
  void recursiveAssignAMoveToAnAgent(
      int agentToAssign,
      std::vector<std::tuple<std::shared_ptr<StandardMultiAgentSpaceTimeState>,
                             int, int>> *successors,
      int cost, std::vector<int> positions,
      const std::vector<int> &prePositions, int t, int violations,
      std::vector<u_int8_t> cannotMove = std::vector<u_int8_t>()) const;
};

#endif // TFE_MAPF_STANDARDMULTIAGENTASTARPROBLEMWITHCONSTRAINTS_H
