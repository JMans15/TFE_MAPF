//
// Created by Arthur Mahy on 09/11/2022.
//

#ifndef TFE_MAPF_PROBLEM_H
#define TFE_MAPF_PROBLEM_H

#include "State.h"
#include <tuple>
#include <vector>
#include <iostream>
#include "Graph.h"
using namespace std;
typedef tuple<State, string, int> Triple;

class Problem {
public:

    Problem(Graph m_graph, vector<int> m_starts, vector<int> m_targets);
    State getStartState() const; // Returns the start state for the search problem.
    bool isGoalState(State state) const; // Returns True if and only if the state is a valid goal state.
    vector<Triple> getSuccessors(State state) const;
    /* For a given state, this should return a list of triples, (successor,
        action, stepCost), where 'successor' is a successor to the current
        state, 'action' is the action required to get there, and 'stepCost' is
        the incremental cost of expanding to that successor. */
    Graph getGraph() const;
    vector<int> getTargets() const;
    // void getCostOfActions(actions) const;
    /* This method returns the total cost of a particular sequence of actions.
        The sequence must be composed of legal moves.*/

private:
    Graph graph;
    vector<int> starts;
    vector<int> targets;
    int numberOfAgents;
};


#endif //TFE_MAPF_PROBLEM_H
