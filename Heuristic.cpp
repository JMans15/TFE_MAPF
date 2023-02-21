//
// Created by Arthur Mahy on 14/01/2023.
//

#include "Heuristic.h"
#include "SingleAgentProblem.h"
#include "library.h"

#include <utility>
#include <algorithm>

// Returns the Manhattan distance between position a and position b
int distance(int a, int b, int width) {
    int ax, ay, bx, by;
    ax = (int) a / width; ay = a % width;
    bx = (int) b / width; by = b % width;
    return abs(ax-bx) + abs(ay-by);
}

// Returns the optimal distance between position a and position b
int optimalDistance(int a, int b, Graph g) {
    SingleAgentProblem problem = SingleAgentProblem(g, a, b, 0);
    Solution solution = aStarSearch(&problem, Manhattan, 0);
    return solution.getCost();
}

Manhattanheuristic::Manhattanheuristic(int m_target, int m_width) {
    target = m_target;
    width = m_width;
}

int Manhattanheuristic::heuristicFunction(State* state) {
    auto* SAstate = dynamic_cast<SingleAgentState *>(state);
    return distance(SAstate->getPosition(), target, width);
}

SICheuristic::SICheuristic(vector<int> m_targets, int m_width) {
    targets = std::move(m_targets);
    width = m_width;
}

int SICheuristic::heuristicFunction(State* state) {
    auto* MAstate = dynamic_cast<MultiAgentState *>(state); // NEEDED ??
    int sum = 0;
    vector<int> positions = MAstate->getPositions();
    for (int i = 0; i < positions.size(); i++){
        sum += distance(positions[i], targets[i], width);
    }
    return sum;
}

MICheuristic::MICheuristic(vector<int> m_targets, int m_width) {
    targets = std::move(m_targets);
    width = m_width;
}

int MICheuristic::heuristicFunction(State* state) {
    auto* MAstate = dynamic_cast<MultiAgentState *>(state);
    int Max = 0;
    vector<int> positions = MAstate->getPositions();
    for (int i = 0; i < MAstate->getAgentToAssign(); i++){
        Max = max(Max, distance(positions[i], targets[i], width));
    }
    for (int i = MAstate->getAgentToAssign(); i < positions.size(); i++){
        Max = max(Max, distance(positions[i], targets[i], width)-1);
    }
    return Max;
}

SIOCheuristic::SIOCheuristic(vector<int> m_targets, Graph m_graph) : graph(m_graph) {
    targets = std::move(m_targets);
}

int SIOCheuristic::heuristicFunction(State *state) {
    auto* MAstate = dynamic_cast<MultiAgentState *>(state); // NEEDED ??
    int sum = 0;
    vector<int> positions = MAstate->getPositions();
    for (int i = 0; i < positions.size(); i++){
        sum += optimalDistance(positions[i], targets[i], graph);
    }
    return sum;
}

MIOCheuristic::MIOCheuristic(vector<int> m_targets, Graph m_graph) : graph(m_graph) {
    targets = std::move(m_targets);
}

int MIOCheuristic::heuristicFunction(State *state) {
    auto* MAstate = dynamic_cast<MultiAgentState *>(state);
    int Max = 0;
    vector<int> positions = MAstate->getPositions();
    for (int i = 0; i < MAstate->getAgentToAssign(); i++){
        Max = max(Max, optimalDistance(positions[i], targets[i], graph));
    }
    for (int i = MAstate->getAgentToAssign(); i < positions.size(); i++){
        Max = max(Max, optimalDistance(positions[i], targets[i], graph)-1);
    }
    return Max;
}
