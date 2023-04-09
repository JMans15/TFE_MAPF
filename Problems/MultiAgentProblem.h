//
// Created by Arthur Mahy on 04/01/2023.
//

#ifndef TFE_MAPF_MULTIAGENTPROBLEM_H
#define TFE_MAPF_MULTIAGENTPROBLEM_H

#include "Problem.h"
#include "../States/MultiAgentState.h"

#include <set>

class MultiAgentProblem : public Problem<MultiAgentState> {
public:
    MultiAgentProblem(std::shared_ptr<Graph> graph, std::vector<int> starts, std::vector<int> targets,
                      ObjectiveFunction objective = Fuel, const std::vector<int>& agentIds = std::vector<int>(),
                      const std::set<VertexConstraint> &setOfVertexConstraints = std::set<VertexConstraint>(),
                      const std::set<EdgeConstraint> &setOfEdgeConstraints = std::set<EdgeConstraint>(), int maxCost = INT_MAX);

    std::shared_ptr<MultiAgentState> getStartState() const override;
    bool isGoalState(std::shared_ptr<MultiAgentState> state) const override;
    std::vector<std::pair<std::shared_ptr<MultiAgentState>, int>> getSuccessors(std::shared_ptr<MultiAgentState> state) const override;
    std::unordered_map<int, std::vector<int>> getPositions(std::vector<std::shared_ptr<MultiAgentState>> states) const override;
    std::vector<int> getAgentIds() const override;

    const std::vector<int>& getStarts() const;
    const std::vector<int>& getTargets() const;
    ObjectiveFunction getObjFunction();
    std::set<VertexConstraint> getSetOfVertexConstraints() const;
    std::set<EdgeConstraint> getSetOfEdgeConstraints() const;

    int getStartOf(int id);
    int getTargetOf(int id);

private:
    // starts is a list of length numberOfAgents with the start position of each agent
    std::vector<int> starts;

    // target is a list of length numberOfAgents with the target position of each agent
    std::vector<int> targets;

    std::vector<int> agentIds; // Index to id
    std::unordered_map<int, int> idToIndex; // Id to index

    // The objective function to minimize : Fuel or Makespan or SumOfCosts
    // - Fuel : Total amount of distance traveled by all agents
    // - Makespan : Total time for the last agent to reach its goal
    // - SumOfCosts : The sum of the time steps required for every agent to reach its goal
    ObjectiveFunction objective;

    // set of vertex constraints like (a, p, t) meaning agent a can't be at position p at time t
    std::set<VertexConstraint> setOfVertexConstraints;
    // set of edge constraints
    std::set<EdgeConstraint> setOfEdgeConstraints;

    // Returns true if agent is allowed to go from position to newPosition between time-1 and time (according to the set of constraints of the problem)
    bool okForConstraints(int agent, int position, int newPosition, int time) const;

};


#endif //TFE_MAPF_MULTIAGENTPROBLEM_H
