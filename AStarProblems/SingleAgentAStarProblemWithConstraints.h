//
// Created by Arthur Mahy on 11/06/2023.
//

#ifndef TFE_MAPF_SINGLEAGENTASTARPROBLEMWITHCONSTRAINTS_H
#define TFE_MAPF_SINGLEAGENTASTARPROBLEMWITHCONSTRAINTS_H

#include "../Problems/SingleAgentProblem.h"
#include "../States/SingleAgentSpaceTimeState.h"
#include "AStarProblem.h"

// Single Agent Problem formulated as a search task
// - Space Time search (because of the external constraints)
class SingleAgentAStarProblemWithConstraints
    : public AStarProblem<SingleAgentSpaceTimeState> {
public:
  SingleAgentAStarProblemWithConstraints(
      std::shared_ptr<SingleAgentProblem> problem);

  std::shared_ptr<SingleAgentSpaceTimeState> getStartState() const override;
  bool
  isGoalState(std::shared_ptr<SingleAgentSpaceTimeState> state) const override;
  std::vector<std::tuple<std::shared_ptr<SingleAgentSpaceTimeState>, int, int>>
  getSuccessors(
      std::shared_ptr<SingleAgentSpaceTimeState> state) const override;
  std::unordered_map<int, std::vector<int>>
  getPositions(std::vector<std::shared_ptr<SingleAgentSpaceTimeState>> states)
      const override;
  int getMaxCost() const override;
  int getStartTime() const override;

  std::shared_ptr<SingleAgentProblem> getProblem();

private:
  std::shared_ptr<SingleAgentProblem> problem;
};

#endif // TFE_MAPF_SINGLEAGENTASTARPROBLEMWITHCONSTRAINTS_H
