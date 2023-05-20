//
// Created by Arthur Mahy on 13/02/2023.
//

#ifndef TFE_MAPF_SINGLEAGENTPROBLEMWITHCONSTRAINTS_H
#define TFE_MAPF_SINGLEAGENTPROBLEMWITHCONSTRAINTS_H

#include "Problem.h"
#include "../States/SingleAgentSpaceTimeState.h"

#include <set>

class SingleAgentProblemWithConstraints : public Problem<SingleAgentSpaceTimeState> {
public:
    SingleAgentProblemWithConstraints(std::shared_ptr<Graph> graph, int start, int target, ObjectiveFunction objective,
                                      int agentId = 0, const std::set<VertexConstraint> &setOfHardVertexConstraints = std::set<VertexConstraint>(),
                                      const std::set<EdgeConstraint> &setOfHardEdgeConstraints = std::set<EdgeConstraint>(), int maxCost = INT_MAX,
                                      const std::set<VertexConstraint> &setOfSoftVertexConstraints = std::set<VertexConstraint>(),
                                      const std::set<EdgeConstraint> &setOfSoftEdgeConstraints = std::set<EdgeConstraint>());

    std::shared_ptr<SingleAgentSpaceTimeState> getStartState() const override;
    bool isGoalState(std::shared_ptr<SingleAgentSpaceTimeState> state) const override;
    std::vector<std::tuple<std::shared_ptr<SingleAgentSpaceTimeState>, int, int>> getSuccessors(std::shared_ptr<SingleAgentSpaceTimeState> state) const override;
    std::unordered_map<int, std::vector<int>> getPositions(std::vector<std::shared_ptr<SingleAgentSpaceTimeState>> states) const override;
    std::vector<int> getAgentIds() const override;
    bool isImpossible() const override;

    const int getStart() const;
    const int getTarget() const;
    ObjectiveFunction getObjFunction();

private:
    // start position of the agent
    int start;

    // target position of the agent
    int target;

    int agentId;

    // true if the start or the target position is unreachable
    bool impossible;

    // The objective function to minimize : Fuel or Makespan
    // - Fuel : Total amount of distance traveled by the agent (costWait = 0)
    // - Makespan or SumOfCosts : Total time for the agent to reach its goal (costWait = 1)
    ObjectiveFunction objective;

    // set of vertex constraints like (a, p, t) meaning agent a can't be at position p at time t
    std::set<VertexConstraint> setOfHardVertexConstraints;
    // set of edge constraints
    std::set<EdgeConstraint> setOfHardEdgeConstraints;

    std::set<VertexConstraint> setOfSoftVertexConstraints;
    std::set<EdgeConstraint> setOfSoftEdgeConstraints;

    // Returns true if the agent is allowed to go from position to newPosition between time-1 and time
    // (according to the hard vertex constraints and the hard edge constraints of the problem)
    bool okForConstraints(int position, int newPosition, int time) const;

    // Returns true if the agent is allowed to be at newPosition at time
    // (according to the hard vertex constraints of the problem)
    bool okForConstraints(int newPosition, int time) const;

    // The number of violated soft constraints if agent go from position to newPosition between time-1 and time
    // (according to the soft vertex constraints and the soft edge constraints of the problem)
    int numberOfViolations(int position, int newPosition, int time) const;

    // The number of violated soft constraints if agent is at newPosition at time
    // (according to the soft vertex constraints of the problem)
    int numberOfViolations(int newPosition, int time) const;
};


#endif //TFE_MAPF_SINGLEAGENTPROBLEMWITHCONSTRAINTS_H
