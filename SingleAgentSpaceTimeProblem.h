//
// Created by Arthur Mahy on 13/02/2023.
//

#ifndef TFE_MAPF_SINGLEAGENTSPACETIMEPROBLEM_H
#define TFE_MAPF_SINGLEAGENTSPACETIMEPROBLEM_H

#include "Problem.h"
#include "SingleAgentSpaceTimeState.h"
typedef tuple<int, int, int> Constraint; // agent, position, time

class SingleAgentSpaceTimeProblem : public Problem{
public:
    SingleAgentSpaceTimeProblem(Graph m_graph, int m_start, int m_target, ObjectiveFunction m_obj_function,
                                const vector<Constraint> &m_setOfConstraints = vector<Constraint>(),
                                        int m_numberOfTheAgent = 0, int verbose = 1);
    State* getStartState();
    bool isGoalState(State* state);
    vector<Double> getSuccessors(State* state);
    vector<int> getStarts();
    vector<int> getTargets();
    ObjectiveFunction getObjFunction();

private:
    // start position of the agent
    int start;

    // target position of the agent
    int target;

    // the number of the agent of this problem
    int numberOfTheAgent;

    // The objective function to minimize : Fuel or Makespan
    // - Fuel : Total amount of distance traveled by the agent (costWait = 0)
    // - Makespan : Total time for the agent to reach its goal (costWait = 1)
    ObjectiveFunction obj_function;

    // list of constraints like (a, p, t) meaning agent a can't be at position p at time t
    vector<Constraint> setOfConstraints;

    // setOfConstraintsMap[t] is the list of positions where this agent can't be at time t
    map<int, vector<int>> setOfConstraintsMap;
};


#endif //TFE_MAPF_SINGLEAGENTSPACETIMEPROBLEM_H
