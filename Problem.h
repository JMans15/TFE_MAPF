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
#include "map"
using namespace std;
typedef tuple<State, string, int> Triple;
typedef tuple<int, int, int> Constraint; // agent, position, time

class Problem {
public:
    // Constructor
    Problem(Graph m_graph, vector<int> m_starts, vector<int> m_targets,
            string m_obj_function="Fuel", const vector<Constraint>& m_setOfConstraints = vector<Constraint>());
    // Returns the start state for the search problem
    State getStartState() const;
    // Returns True if the state is a valid goal state
    bool isGoalState(State state) const;
    // For a given state, getSuccessors returns a list of triples (successor, action, stepcost)
    // where successor is a successor state to the current state
    // action is a string describing the action to from state to successor
    // stepcost is the cost to go from state to successor
    // Extends state thanks to Operator Decomposition
    vector<Triple> getSuccessors(State state) const;
    // Sum of Individual Costs heuristic (for SumOfCosts and Fuel objective functions)
    static int SICheuristic(State state, const Problem& problem);
    // Maximum Individual Cost heuristic (for Makespan objective function)
    static int MICheuristic(State state, const Problem& problem);
    // retrieveSolution is needed at the end of an A* search to retrieve the solution path from node
    Solution retrieveSolution(int numberOfVisitedStates, Node node) const ;

    Graph getGraph() const;
    vector<int> getTargets() const;
    string getObjFunction() const;

private:
    // Graph with the possible positions and transitions for the agents
    Graph graph;
    // starts is a list of length numberOfAgents with the start position of each agent
    vector<int> starts;
    // target is a list of length numberOfAgents with the target position of each agent
    vector<int> targets;
    int numberOfAgents;
    // The objective function to minimize : "Fuel" or "Makespan" or "SumOfCosts"
    // - Fuel : Total amount of distance traveled by all agents
    // - Makespan : Total time for the last agent to reach its goal
    // - SumOfCosts : The sum of the time steps required for every agent to reach its goal (doesn't work right)
    string obj_function;
    // list of constraints like (a, p, t) meaning agent a can't be at position p at time t
    vector<Constraint> setOfConstraints;
    // setOfConstraintsMap[a][t] is the list of positions where agent a can't be at time t
    map<int, map<int, vector<int>>> setOfConstraintsMap;
};


#endif //TFE_MAPF_PROBLEM_H