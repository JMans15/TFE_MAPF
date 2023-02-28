//
// Created by Arthur Mahy on 14/01/2023.
//

#ifndef TFE_MAPF_HEURISTIC_H
#define TFE_MAPF_HEURISTIC_H
#include <memory>
#include "State.h"
#include "ReverseResumableAStar.h"
#include "Graph.h"
enum TypeOfHeuristic {
    SIC, MIC, Manhattan, MIOC, SIOC, OptimalDistance
};

class Heuristic {
public:
    virtual int heuristicFunction(shared_ptr<State> state) = 0;
    virtual ~Heuristic() = default;
};

// Manhattan distance heuristic
// - for single agent problem
class Manhattanheuristic : public Heuristic {
public :
    Manhattanheuristic(int m_target, int m_width);
private:
    int target;
    int width;

    int heuristicFunction(shared_ptr<State> state) override;
};

class ReverseResumableAStar;

// Optimal distance heuristic
// - for single agent problem
// - only interesting for Space Time A*
// A reverse resumable A* search will be run in addition to the search which this heuristic is used for.
class OptimalDistanceheuristic : public Heuristic {
public :
    OptimalDistanceheuristic(int m_start, int m_target, const Graph& m_graph);
    int heuristicFunction(shared_ptr<State> state);
private:
    ReverseResumableAStar* RRAStarSearch;
};

// Sum of Individual Costs heuristic
// where cost is the Manhattan distance
// - for SumOfCosts and Fuel objective functions
// - for multi agent problem
class SICheuristic : public Heuristic {
public :
    SICheuristic(vector<int> m_targets, int m_width);
    int heuristicFunction(shared_ptr<State> state);
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
    int heuristicFunction(shared_ptr<State> state);
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
    SIOCheuristic(vector<int> m_targets, const Graph& m_graph);
    int heuristicFunction(shared_ptr<State> state);
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
    MIOCheuristic(vector<int> m_targets, const Graph& m_graph);
    int heuristicFunction(shared_ptr<State> state);
private:
    vector<int> targets;
    Graph graph;
};


#endif //TFE_MAPF_HEURISTIC_H
