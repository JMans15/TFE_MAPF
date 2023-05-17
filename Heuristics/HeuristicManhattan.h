//
// Created by Arthur Mahy on 14/01/2023.
//

#ifndef TFE_MAPF_HEURISTICMANHATTAN_H
#define TFE_MAPF_HEURISTICMANHATTAN_H

#include "../GraphParser/Graph.h"
#include "../Problems/MultiAgentProblemWithConstraints.h"
#include "../States/MultiAgentState.h"
#include "../Solvers/AStar/ReverseResumableAStar.h"
#include "../Problems/SingleAgentProblem.h"
#include "../States/SingleAgentState.h"
#include "../Problems/SingleAgentProblemWithConstraints.h"
#include "../States/SingleAgentSpaceTimeState.h"
#include "../States/State.h"

#include <memory>

enum TypeOfHeuristic {
    Manhattan, OptimalDistance
};

template <class S>
class Heuristic {
public:
    virtual int heuristicFunction(std::shared_ptr<S> state) = 0;
    virtual ~Heuristic() = default;
};

// Returns the Manhattan distance between position a and position b
// ignoring walls and other agents
inline int manhattanDistance(int a, int b, int width) {
    int ax, ay, bx, by;
    ax = (int) a % width; ay = a / width;
    bx = (int) b % width; by = b / width;
    return abs(ax-bx) + abs(ay-by);
}

// Manhattan distance heuristic (ignoring walls)
// - for single agent problem
template<typename S, typename std::enable_if<std::is_base_of<SingleAgentState, S>::value or std::is_base_of<SingleAgentSpaceTimeState, S>::value>::type* = nullptr>
class ManhattanHeuristic : public Heuristic<S> {
public:
    ManhattanHeuristic(int target, int width) : target(target), width(width) {}

    int heuristicFunction(std::shared_ptr<S> state) override {
        return manhattanDistance(state->getPosition(), target, width);
    }
private:
    int target;
    int width;
};

// Sum of Individual Costs heuristic
// where cost is the Manhattan distance (ignoring walls and other agents)
// - for SumOfCosts and Fuel objective functions
// - for multi agent problem
template<typename S, typename std::enable_if<std::is_base_of<MultiAgentState, S>::value>::type* = nullptr>
class SICheuristic : public Heuristic<S> {
public:
    SICheuristic(std::vector<int> targets, int width) : targets(targets), width(width) {}

    int heuristicFunction(std::shared_ptr<S> state) override {
        int sum = 0;
        auto positions = state->getPositions();
        for (int i = 0; i < positions.size(); i++){
            sum += manhattanDistance(positions[i], targets[i], width);
        }
        return sum;
    }
private:
    std::vector<int> targets;
    int width;
};

// Maximum Individual Cost heuristic
// where cost is the Manhattan distance (ignoring walls and other agents)
// - for Makespan objective function
// - for multi agent problem
template<typename S, typename std::enable_if<std::is_base_of<MultiAgentState, S>::value>::type* = nullptr>
class MICheuristic : public Heuristic<S> {
public:
    MICheuristic(std::vector<int> targets, int width) : targets(targets), width(width) {}

    int heuristicFunction(std::shared_ptr<S> state) {
        int maxDistance = 0;
        auto positions = state->getPositions();
        for (int i = 0; i < state->getAgentToAssign(); i++){
            maxDistance = std::max(maxDistance, manhattanDistance(positions[i], targets[i], width));
        }
        for (int i = state->getAgentToAssign(); i < positions.size(); i++){
            maxDistance = std::max(maxDistance, manhattanDistance(positions[i], targets[i], width) - 1);
        }
        return maxDistance;
    }
private:
    std::vector<int> targets;
    int width;
};

#endif //TFE_MAPF_HEURISTICMANHATTAN_H
