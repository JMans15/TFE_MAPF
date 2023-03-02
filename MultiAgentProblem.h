//
// Created by Arthur Mahy on 04/01/2023.
//

#ifndef TFE_MAPF_MULTIAGENTPROBLEM_H
#define TFE_MAPF_MULTIAGENTPROBLEM_H

#include "Problem.h"
#include "MultiAgentState.h"

#include <set>

class MultiAgentProblem : public Problem<MultiAgentState> {
public:
    MultiAgentProblem(std::shared_ptr<Graph> graph, std::vector<int> starts, std::vector<int> targets,
                      ObjectiveFunction objective = Fuel, const std::set<Constraint> &setOfConstraints = std::set<Constraint>());

    std::shared_ptr<MultiAgentState> getStartState() const override;
    bool isGoalState(std::shared_ptr<MultiAgentState> state) const override;
    std::vector<std::pair<std::shared_ptr<MultiAgentState>, int>> getSuccessors(std::shared_ptr<MultiAgentState> state) const override;
    std::vector<std::vector<int>> getPositions(std::vector<std::shared_ptr<MultiAgentState>> states) const override;

    const std::vector<int>& getStarts() const;
    const std::vector<int>& getTargets() const;
    ObjectiveFunction getObjFunction();
    const std::set<Constraint>& getSetOfConstraints() const;

private:
    // starts is a list of length numberOfAgents with the start position of each agent
    std::vector<int> starts;

    // target is a list of length numberOfAgents with the target position of each agent
    std::vector<int> targets;

    // The objective function to minimize : Fuel or Makespan or SumOfCosts
    // - Fuel : Total amount of distance traveled by all agents
    // - Makespan : Total time for the last agent to reach its goal
    // - SumOfCosts : The sum of the time steps required for every agent to reach its goal
    ObjectiveFunction objective;

    // list of constraints like (a, p, t) meaning agent a can't be at position p at time t
    std::set<Constraint> setOfConstraints;

    bool notInForbiddenPositions(int position, int agent, int time) const;

};


#endif //TFE_MAPF_MULTIAGENTPROBLEM_H
