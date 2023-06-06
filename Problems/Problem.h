//
// Created by Arthur Mahy on 09/11/2022.
//

#ifndef TFE_MAPF_PROBLEM_H
#define TFE_MAPF_PROBLEM_H

#include "../GraphParser/Graph.h"
#include "../Constraints/ConstraintsSet.h"

#include <iostream>
#include <memory>
#include <utility>
#include "unordered_map"

//#define DEBUG

#ifdef DEBUG
#define LOG(str) std::cout << str << std::endl;
#else
#define LOG(str)
#endif

enum ObjectiveFunction {
    Fuel, Makespan, SumOfCosts
};

template <class S>
class Problem {
public:

    Problem(std::shared_ptr<Graph> graph, int numberOfAgents, int maxCost, int startTime) : graph(std::move(graph)), numberOfAgents(numberOfAgents), maxCost(maxCost), startTime(startTime) {}
    ~Problem() = default;

    std::shared_ptr<Graph> getGraph() const {
        return graph;
    }

    inline int getNumberOfAgents() const {
        return numberOfAgents;
    }

    inline int getMaxCost() const {
        return maxCost;
    }

    inline int getStartTime() const {
        return startTime;
    }

    // Returns the start state for the search problem
    virtual std::shared_ptr<S> getStartState() const = 0;

    // Returns true if the state is a valid goal state
    virtual bool isGoalState(std::shared_ptr<S> state) const = 0;

    // For a given state, getSuccessors returns a list of pairs (successor, stepcost, numberOfViolations)
    // where successor is a successor state to the current state
    // stepcost is the cost to go from state to successor
    // numberOfViolations is the number of soft constraints that have been violated to go from state to successor
    virtual std::vector<std::tuple<std::shared_ptr<S>, int, int>> getSuccessors(std::shared_ptr<S> state) const = 0;

    virtual std::unordered_map<int, std::vector<int>> getPositions(std::vector<std::shared_ptr<S>> states) const = 0;

    virtual std::vector<int> getAgentIds() const = 0;
    virtual bool isImpossible() const = 0;

protected:

    // Graph with the possible positions and transitions for the agents
    std::shared_ptr<Graph> graph;

    int numberOfAgents;

    // The solution of this problem must have a cost inferior or equal to maxCost
    int maxCost;

    int startTime;

};

#endif //TFE_MAPF_PROBLEM_H
