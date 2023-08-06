//
// Created by Arthur Mahy on 11/06/2023.
//

#ifndef TFE_MAPF_STANDARDMULTIAGENTASTARPROBLEM_H
#define TFE_MAPF_STANDARDMULTIAGENTASTARPROBLEM_H

#include "../Problems/MultiAgentProblem.h"
#include "../States/StandardMultiAgentState.h"
#include "AStarProblem.h"

// Multi Agent Problem formulated as a search task
// - Standard A* (>< Operator Decomposition A*)
// - Space search
class StandardMultiAgentAStarProblem : AStarProblem<StandardMultiAgentState> {
public:
  StandardMultiAgentAStarProblem(std::shared_ptr<MultiAgentProblem> problem);

  std::shared_ptr<StandardMultiAgentState> getStartState() const override;
  bool
  isGoalState(std::shared_ptr<StandardMultiAgentState> state) const override;
  std::vector<std::tuple<std::shared_ptr<StandardMultiAgentState>, int, int>>
  getSuccessors(std::shared_ptr<StandardMultiAgentState> state) const override;
  std::unordered_map<int, std::vector<int>>
  getPositions(std::vector<std::shared_ptr<StandardMultiAgentState>> states)
      const override;
  int getMaxCost() const override;
  int getStartTime() const override;

  std::shared_ptr<MultiAgentProblem> getProblem();

private:
  std::shared_ptr<MultiAgentProblem> problem;

  // Returns true if position is not already occupied by assigned agents
  bool notAlreadyOccupiedPosition(int position, std::vector<int> &positions,
                                  int agentToAssign) const;

  // Returns true if the edge (position, positions[agentToAssign]) is not
  // already occupied by assigned agents
  bool notAlreadyOccupiedEdge(int position, const std::vector<int> &positions,
                              int agentToAssign,
                              const std::vector<int> &prePositions) const;

  // Recursive function used in the getSuccessors(state) method
  // Branches on all possibles moves for agentToAssign (from 0 to
  // numberOfAgents-1) If agentToAssign is the last agent, we add a successor to
  // the list of successors for every possible move for agentToAssign If
  // agentToAssign isn't the last agent, we call recursiveAssignAMoveToAnAgent
  // for every possible move for agentToAssign
  //
  // positions[:agentToAssign] are the assigned positions
  // positions[agentToAssign:] are the not yet assigned positions
  // positions[agentToAssign] is not yet assigned but will be in this function
  //
  // prePositions is the positions of the agents in state
  // cost is the cost of assigning the agents from 0 to agentToAssign
  // cannotMove[i] is true if agent i is at its target position and cannot move
  // anymore (for the SumOfCosts objective function)
  void recursiveAssignAMoveToAnAgent(
      int agentToAssign,
      std::vector<std::tuple<std::shared_ptr<StandardMultiAgentState>, int,
                             int>> *successors,
      int cost, std::vector<int> positions,
      const std::vector<int> &prePositions,
      std::vector<u_int8_t> cannotMove = std::vector<u_int8_t>()) const;
};

#endif // TFE_MAPF_STANDARDMULTIAGENTASTARPROBLEM_H
