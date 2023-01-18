//
// Created by Arthur Mahy on 14/01/2023.
//

#ifndef TFE_MAPF_HEURISTIC_H
#define TFE_MAPF_HEURISTIC_H
#include "State.h"
#include "MultiAgentState.h"
#include "SingleAgentState.h"
enum TypeOfHeuristic {
    SIC, MIC, Manhattan
};

class Heuristic {
public:
    virtual int heuristicFunction(State* state) = 0;
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


#endif //TFE_MAPF_HEURISTIC_H
