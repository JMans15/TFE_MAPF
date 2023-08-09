//! MAPF problem with space search only and OD

#ifndef TFE_MAPF_ODMULTIAGENTASTARPROBLEM_H
#define TFE_MAPF_ODMULTIAGENTASTARPROBLEM_H

#include "../Problems/MultiAgentProblem.h"
#include "../States/ODMultiAgentState.h"
#include "AStarProblem.h"

//! Multi Agent Problem formulated as a search task
//! - Operator Decomposition (>< Standard A*)
//! - Space search
class ODMultiAgentAStarProblem : AStarProblem<ODMultiAgentState> {
public:
  //! Constructor
  //! @param [in] MultiAgentProblem to solve
  ODMultiAgentAStarProblem(std::shared_ptr<MultiAgentProblem> problem);

  std::shared_ptr<ODMultiAgentState> getStartState() const override;
  bool isGoalState(std::shared_ptr<ODMultiAgentState> state) const override;
  std::vector<std::tuple<std::shared_ptr<ODMultiAgentState>, int, int>>
  getSuccessors(std::shared_ptr<ODMultiAgentState> state) const override;
  std::unordered_map<int, std::vector<int>> getPositions(
      std::vector<std::shared_ptr<ODMultiAgentState>> states) const override;
  int getMaxCost() const override;
  int getStartTime() const override;

  //! Getter for the MultiAgentProblem
  std::shared_ptr<MultiAgentProblem> getProblem();

private:
  //! Base MultiAgentProblem
  std::shared_ptr<MultiAgentProblem> problem;

  //! @return true if position is not already occupied by assigned agents
  bool notAlreadyOccupiedPosition(int position, std::vector<int> &positions,
                                  int agentToAssign) const;

  //! @return true if the edge (position, positions[agentToAssign]) is not
  // already occupied by assigned agents
  bool notAlreadyOccupiedEdge(int position, const std::vector<int> &positions,
                              int agentToAssign,
                              const std::vector<int> &prePositions) const;
};

#endif // TFE_MAPF_ODMULTIAGENTASTARPROBLEM_H
