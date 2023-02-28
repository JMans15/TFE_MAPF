//
// Created by Arthur Mahy on 09/11/2022.
//

#ifndef TFE_MAPF_PROBLEM_H
#define TFE_MAPF_PROBLEM_H

#include "State.h"
#include "Graph.h"
#include <tuple>
#include <vector>
#include <iostream>
#include <memory>
#include "map"
using namespace std;
typedef tuple<shared_ptr<State>, int> Double;
enum ObjectiveFunction {
    Fuel, Makespan, SumOfCosts
};

class Problem {
public:

    // Constructor
    Problem(Graph m_graph);
    ~Problem();
    // Returns the start state for the search problem
    virtual shared_ptr<State> getStartState() = 0;

    // Returns True if the state is a valid goal state
    virtual bool isGoalState(shared_ptr<State> state) = 0;

    // For a given state, getSuccessors returns a list of doubles (successor, stepcost)
    // where successor is a successor state to the current state
    // stepcost is the cost to go from state to successor
    virtual vector<Double> getSuccessors(shared_ptr<State> state) = 0;

    Graph getGraph() const;
    virtual vector<int> getStarts() = 0;
    virtual vector<int> getTargets() = 0;
    int getNumberOfAgents() const;

protected:

    // Graph with the possible positions and transitions for the agents
    Graph graph;

    int numberOfAgents;

};


#endif //TFE_MAPF_PROBLEM_H
