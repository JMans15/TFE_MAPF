//
// Created by Arthur Mahy on 13/02/2023.
//

#ifndef TFE_MAPF_SINGLEAGENTSPACETIMEPROBLEM_H
#define TFE_MAPF_SINGLEAGENTSPACETIMEPROBLEM_H

#include "Problem.h"
#include "../States/SingleAgentSpaceTimeState.h"

#include <set>

class SingleAgentSpaceTimeProblem : public Problem<SingleAgentSpaceTimeState> {
public:
    SingleAgentSpaceTimeProblem(std::shared_ptr<Graph> graph, int start, int target, ObjectiveFunction objective,
                                int agentId = 0, const std::set<Constraint> &setOfConstraints = std::set<Constraint>());

    std::shared_ptr<SingleAgentSpaceTimeState> getStartState() const override;
    bool isGoalState(std::shared_ptr<SingleAgentSpaceTimeState> state) const override;
    std::vector<std::pair<std::shared_ptr<SingleAgentSpaceTimeState>, int>> getSuccessors(std::shared_ptr<SingleAgentSpaceTimeState> state) const override;
    std::unordered_map<int, std::vector<int>> getPositions(std::vector<std::shared_ptr<SingleAgentSpaceTimeState>> states) const override;
    std::vector<int> getAgentIds() const override;

    const int getStart() const;
    const int getTarget() const;
    ObjectiveFunction getObjFunction();

    bool notInForbiddenPositions(int position, int time) const;

private:
    // start position of the agent
    int start;

    // target position of the agent
    int target;

    int agentId;

    // The objective function to minimize : Fuel or Makespan
    // - Fuel : Total amount of distance traveled by the agent (costWait = 0)
    // - Makespan : Total time for the agent to reach its goal (costWait = 1)
    ObjectiveFunction objective;

    // list of constraints like (a, p, t) meaning agent a can't be at position p at time t
    std::set<Constraint> setOfConstraints;
};


#endif //TFE_MAPF_SINGLEAGENTSPACETIMEPROBLEM_H
