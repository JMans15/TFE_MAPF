//
// Created by Arthur Mahy on 04/01/2023.
//

#ifndef TFE_MAPF_MULTIAGENTPROBLEM_H
#define TFE_MAPF_MULTIAGENTPROBLEM_H
#include "Problem.h"
#include "MultiAgentState.h"
#include "set"
typedef tuple<int, int, int> Constraint; // agent, position, time

class MultiAgentProblem : public Problem{
public:
    MultiAgentProblem(Graph m_graph, vector<int> m_starts, vector<int> m_targets,
                      ObjectiveFunction m_obj_function = Fuel,
                      const set<Constraint> &m_setOfConstraints = set<Constraint>(), int verbose = 1);
    State* getStartState();
    bool isGoalState(State* state);

    // Extends state thanks to Operator Decomposition
    vector<Double> getSuccessors(State* state);

    vector<int> getStarts();
    vector<int> getTargets();
    ObjectiveFunction getObjFunction();

private:
    // starts is a list of length numberOfAgents with the start position of each agent
    vector<int> starts;

    // target is a list of length numberOfAgents with the target position of each agent
    vector<int> targets;

    // The objective function to minimize : Fuel or Makespan or SumOfCosts
    // - Fuel : Total amount of distance traveled by all agents
    // - Makespan : Total time for the last agent to reach its goal
    // - SumOfCosts : The sum of the time steps required for every agent to reach its goal
    ObjectiveFunction obj_function;

    // list of constraints like (a, p, t) meaning agent a can't be at position p at time t
    set<Constraint> setOfConstraints;

    // setOfConstraintsMap[a][t] is the list of positions where agent a can't be at time t
    map<int, map<int, set<int>>> setOfConstraintsMap;

};


#endif //TFE_MAPF_MULTIAGENTPROBLEM_H
