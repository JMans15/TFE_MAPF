//! MAPF problem with space-time search and OD

#ifndef TFE_MAPF_ODMULTIAGENTASTARPROBLEMWITHCONSTRAINTS_H
#define TFE_MAPF_ODMULTIAGENTASTARPROBLEMWITHCONSTRAINTS_H

#include "../Problems/MultiAgentProblem.h"
#include "../States/ODMultiAgentSpaceTimeState.h"
#include "AStarProblem.h"

//! Multi Agent Problem formulated as a search task
//! - Operator Decomposition (>< Standard A*)
//! - Space Time search (because of the external constraints)
class ODMultiAgentAStarProblemWithConstraints
    : AStarProblem<ODMultiAgentSpaceTimeState> {
public:
  //! Constructor
  //! @param [in] problem MultiAgentProblem to solve
  ODMultiAgentAStarProblemWithConstraints(
      std::shared_ptr<MultiAgentProblem> problem);

  std::shared_ptr<ODMultiAgentSpaceTimeState> getStartState() const override;
  bool
  isGoalState(std::shared_ptr<ODMultiAgentSpaceTimeState> state) const override;
  std::vector<std::tuple<std::shared_ptr<ODMultiAgentSpaceTimeState>, int, int>>
  getSuccessors(
      std::shared_ptr<ODMultiAgentSpaceTimeState> state) const override;
  std::unordered_map<int, std::vector<int>>
  getPositions(std::vector<std::shared_ptr<ODMultiAgentSpaceTimeState>> states)
      const override;
  int getMaxCost() const override;
  int getStartTime() const override;

  //! Get the base problem
  std::shared_ptr<MultiAgentProblem> getProblem();

private:
  //! Base problem
  std::shared_ptr<MultiAgentProblem> problem;

  //! @return true if position is not already occupied by assigned agents
  bool notAlreadyOccupiedPosition(int position, std::vector<int> &positions,
                                  int agentToAssign) const;

  //! @return true if the edge (position, positions[agentToAssign]) is not
  //! already occupied by assigned agents
  bool notAlreadyOccupiedEdge(int position, const std::vector<int> &positions,
                              int agentToAssign,
                              const std::vector<int> &prePositions) const;
};

#endif // TFE_MAPF_ODMULTIAGENTASTARPROBLEMWITHCONSTRAINTS_H
