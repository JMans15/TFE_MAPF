//
// Created by Arthur Mahy on 09/11/2022.
//

#ifndef TFE_MAPF_PROBLEM_H
#define TFE_MAPF_PROBLEM_H

#include "../GraphParser/Graph.h"

#include <iostream>
#include <memory>

#define DEBUG

#ifdef DEBUG
#define LOG(str) std::cout << str << std::endl;
#else
#define LOG(str)
#endif

enum ObjectiveFunction {
    Fuel, Makespan, SumOfCosts
};

typedef struct Constraint {
    int agent;
    int position;
    int time;

    bool operator<(const struct Constraint &other) const {
        if (agent == other.agent) {
            if (position == other.position) {
                return time < other.time;
            }
            return position < other.position;
        }
        return agent < other.agent;
    }
} Constraint;

template <class S>
class Problem {
public:

    Problem(std::shared_ptr<Graph> graph, int numberOfAgents) : graph(graph), numberOfAgents(numberOfAgents) {}
    ~Problem() {}

    std::shared_ptr<Graph> getGraph() const {
        return graph;
    }

    inline int getNumberOfAgents() const {
        return numberOfAgents;
    }

    // Returns the start state for the search problem
    virtual std::shared_ptr<S> getStartState() const = 0;

    // Returns True if the state is a valid goal state
    virtual bool isGoalState(std::shared_ptr<S> state) const = 0;

    // For a given state, getSuccessors returns a list of pairs (successor, stepcost)
    // where successor is a successor state to the current state
    // stepcost is the cost to go from state to successor
    virtual std::vector<std::pair<std::shared_ptr<S>, int>> getSuccessors(std::shared_ptr<S> state) const = 0;

    virtual std::vector<std::vector<int>> getPositions(std::vector<std::shared_ptr<S>> states) const = 0;

    virtual std::vector<int> getAgentIds() const = 0;

protected:

    // Graph with the possible positions and transitions for the agents
    std::shared_ptr<Graph> graph;

    int numberOfAgents;

};

#endif //TFE_MAPF_PROBLEM_H
