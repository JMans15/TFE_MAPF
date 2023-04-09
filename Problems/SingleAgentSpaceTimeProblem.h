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
                                int agentId = 0, const std::set<VertexConstraint> &setOfVertexConstraints = std::set<VertexConstraint>(),
                                const std::set<EdgeConstraint> &setOfEdgeConstraints = std::set<EdgeConstraint>(), int maxCost = INT_MAX);

    std::shared_ptr<SingleAgentSpaceTimeState> getStartState() const override;
    bool isGoalState(std::shared_ptr<SingleAgentSpaceTimeState> state) const override;
    std::vector<std::pair<std::shared_ptr<SingleAgentSpaceTimeState>, int>> getSuccessors(std::shared_ptr<SingleAgentSpaceTimeState> state) const override;
    std::unordered_map<int, std::vector<int>> getPositions(std::vector<std::shared_ptr<SingleAgentSpaceTimeState>> states) const override;
    std::vector<int> getAgentIds() const override;

    const int getStart() const;
    const int getTarget() const;
    ObjectiveFunction getObjFunction();

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

    // set of vertex constraints like (a, p, t) meaning agent a can't be at position p at time t
    std::set<VertexConstraint> setOfVertexConstraints;
    // set of edge constraints
    std::set<EdgeConstraint> setOfEdgeConstraints;

    // Returns true if the agent is allowed to go from position to newPosition between time-1 and time (according to the set of constraints of the problem)
    bool okForConstraints(int position, int newPosition, int time) const;
};


#endif //TFE_MAPF_SINGLEAGENTSPACETIMEPROBLEM_H
