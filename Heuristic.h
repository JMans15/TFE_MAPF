//
// Created by Arthur Mahy on 14/01/2023.
//

#ifndef TFE_MAPF_HEURISTIC_H
#define TFE_MAPF_HEURISTIC_H
#include "State.h"
#include "MultiAgentState.h"
#include "SingleAgentState.h"
#include "Graph.h"
enum TypeOfHeuristic {
    SIC, MIC, Manhattan, MIOC, SIOC
};

class Heuristic {
public:
    virtual int heuristicFunction(State* state) = 0;
};

// Manhattan distance heuristic
// - for single agent problem
class Manhattanheuristic : public Heuristic {
public :
    Manhattanheuristic(int m_target, int m_width);
    int heuristicFunction(State* state);
private:
    int target;
    int width;
};

// Sum of Individual Costs heuristic
// where cost is the Manhattan distance
// - for SumOfCosts and Fuel objective functions
// - for multi agent problem
class SICheuristic : public Heuristic {
public :
    SICheuristic(vector<int> m_targets, int m_width);
    int heuristicFunction(State* state);
private:
    vector<int> targets;
    int width;
};

// Maximum Individual Cost heuristic
// where cost is the Manhattan distance
// - for Makespan objective function
// - for multi agent problem
class MICheuristic : public Heuristic {
public :
    MICheuristic(vector<int> m_targets, int m_width);
    int heuristicFunction(State* state);
private:
    vector<int> targets;
    int width;
};

// Sum of Individual Optimal Costs heuristic
// where cost is the optimal distance computed by a single agent A* (ignoring other agents)
// - for SumOfCosts and Fuel objective functions
// - for multi agent problem
class SIOCheuristic : public Heuristic {
public :
    SIOCheuristic(vector<int> m_targets, Graph m_graph);
    int heuristicFunction(State* state);
private:
    vector<int> targets;
    Graph graph;
};

// Maximum Individual Optimal Cost heuristic
// where cost is the optimal distance computed by a single agent A* (ignoring other agents)
// - for Makespan objective function
// - for multi agent problem
class MIOCheuristic : public Heuristic {
public :
    MIOCheuristic(vector<int> m_targets, Graph m_graph);
    int heuristicFunction(State* state);
private:
    vector<int> targets;
    Graph graph;
};


#endif //TFE_MAPF_HEURISTIC_H
