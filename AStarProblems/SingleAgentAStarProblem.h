//
// Created by Arthur Mahy on 16/01/2023.
//

#ifndef TFE_MAPF_SINGLEAGENTASTARPROBLEM_H
#define TFE_MAPF_SINGLEAGENTASTARPROBLEM_H

#include "../Problems/SingleAgentProblem.h"
#include "../States/SingleAgentState.h"
#include "AStarProblem.h"

class SingleAgentAStarProblem : public AStarProblem<SingleAgentState> {
public:
  SingleAgentAStarProblem(const std::shared_ptr<SingleAgentProblem> &problem);

  std::shared_ptr<SingleAgentState> getStartState() const override;
  std::shared_ptr<SingleAgentState> getGoalState() const;
  bool isGoalState(std::shared_ptr<SingleAgentState> state) const override;
  std::vector<std::tuple<std::shared_ptr<SingleAgentState>, int, int>>
  getSuccessors(std::shared_ptr<SingleAgentState> state) const override;
  std::unordered_map<int, std::vector<int>> getPositions(
      std::vector<std::shared_ptr<SingleAgentState>> states) const override;
  int getMaxCost() const override;
  int getStartTime() const override;

  std::shared_ptr<SingleAgentProblem> getProblem();

private:
  std::shared_ptr<SingleAgentProblem> problem;
};

#endif // TFE_MAPF_SINGLEAGENTASTARPROBLEM_H
