//
// Created by Arthur Mahy on 16/01/2023.
//

#ifndef TFE_MAPF_SINGLEAGENTPROBLEM_H
#define TFE_MAPF_SINGLEAGENTPROBLEM_H

#include "Problem.h"
#include "SingleAgentState.h"

class SingleAgentProblem : public Problem<SingleAgentState> {
public:
    SingleAgentProblem(std::shared_ptr<Graph> graph, int start, int target);

    std::shared_ptr<SingleAgentState> getStartState() const override;
    std::shared_ptr<SingleAgentState> getGoalState() const;
    bool isGoalState(std::shared_ptr<SingleAgentState> state) const override;
    std::vector<std::pair<std::shared_ptr<SingleAgentState>, int>> getSuccessors(std::shared_ptr<SingleAgentState> state) const override;
    std::vector<std::vector<int>> getPositions(std::vector<std::shared_ptr<SingleAgentState>> states) const override;

    const int getStart() const;
    const int getTarget() const;

protected:
    // start position of the agent
    int start;

    // target position of the agent
    int target;
};

#endif //TFE_MAPF_SINGLEAGENTPROBLEM_H
