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
#include "Solution.h"
#include "Node.h"
using namespace std;
typedef tuple<State, string, int> Triple;

class Problem {
public:

    Problem(Graph m_graph, vector<int> m_starts, vector<int> m_targets, string m_obj_function);
    State getStartState() const; // Returns the start state for the search problem.
    bool isGoalState(State state) const; // Returns True if and only if the state is a valid goal state.
    vector<Triple> getSuccessors(State state) const;
    /* For a given state, this should return a list of triples, (successor,
        action, stepCost), where 'successor' is a successor to the current
        state, 'action' is the action required to get there, and 'stepCost' is
        the incremental cost of expanding to that successor. */
    Graph getGraph() const;
    vector<int> getTargets() const;
    string getObjFunction() const;
    static int SICheuristic(State state, const Problem& problem);
    static int MICheuristic(State state, const Problem& problem);
    Solution retrieveSolution(int numberOfVisitedStates, Node node) const ;

private:
    Graph graph;
    vector<int> starts;
    vector<int> targets;
    int numberOfAgents;
    string obj_function;
};


#endif //TFE_MAPF_PROBLEM_H
