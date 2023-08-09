//! SAPF problem with space search only

#ifndef TFE_MAPF_SINGLEAGENTASTARPROBLEM_H
#define TFE_MAPF_SINGLEAGENTASTARPROBLEM_H

#include "../Problems/SingleAgentProblem.h"
#include "../States/SingleAgentState.h"
#include "AStarProblem.h"

//! Single Agent Problem formulated as a search task
//! - Space search
class SingleAgentAStarProblem : public AStarProblem<SingleAgentState> {
public:
  //! Constructor
  //! @param [in] base SingleAgentProblem
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

  //! Get the base SingleAgentProblem
  std::shared_ptr<SingleAgentProblem> getProblem();

private:
  //! Base SingleAgentProblem
  std::shared_ptr<SingleAgentProblem> problem;
};

#endif // TFE_MAPF_SINGLEAGENTASTARPROBLEM_H
