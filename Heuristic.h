//
// Created by Arthur Mahy on 14/01/2023.
//

#ifndef TFE_MAPF_HEURISTIC_H
#define TFE_MAPF_HEURISTIC_H
#include "State.h"
enum TypeOfHeuristic {
    SIC, MIC
};

class Heuristic {
public:
    virtual int heuristicFunction(State state) = 0;
};

// Sum of Individual Costs heuristic (for SumOfCosts and Fuel objective functions)
class SICheuristic : public Heuristic {
public :
    SICheuristic(vector<int> m_targets, int m_width);
    int heuristicFunction(State state);
private:
    vector<int> targets;
    int width;
};

// Maximum Individual Cost heuristic (for Makespan objective function)
class MICheuristic : public Heuristic {
public :
    MICheuristic(vector<int> m_targets, int m_width);
    int heuristicFunction(State state);
private:
    vector<int> targets;
    int width;
};


#endif //TFE_MAPF_HEURISTIC_H
