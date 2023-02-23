//
// Created by Arthur Mahy on 16/01/2023.
//

#ifndef TFE_MAPF_SINGLEAGENTPROBLEM_H
#define TFE_MAPF_SINGLEAGENTPROBLEM_H

#include "Problem.h"
#include "SingleAgentState.h"

class SingleAgentProblem : public Problem{
public:
    SingleAgentProblem(Graph m_graph, int m_start, int m_target);
    shared_ptr<State> getStartState();
    bool isGoalState(shared_ptr<State> state);
    vector<Double> getSuccessors(shared_ptr<State> state);
    vector<int> getStarts();
    vector<int> getTargets();

private:
    // start position of the agent
    int start;

    // target position of the agent
    int target;
};


#endif //TFE_MAPF_SINGLEAGENTPROBLEM_H
