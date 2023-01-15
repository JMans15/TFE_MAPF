//
// Created by Arthur Mahy on 14/01/2023.
//

#include "Heuristic.h"

#include <utility>

// Returns the Manhattan distance between position a and position b
int distance(int a, int b, int width) {
    int ax, ay, bx, by;
    ax = (int) a / width; ay = a % width;
    bx = (int) b / width; by = b % width;
    return abs(ax-bx) + abs(ay-by);
}

SICheuristic::SICheuristic(vector<int> m_targets, int m_width) {
    targets = std::move(m_targets);
    width = m_width;
}

int SICheuristic::heuristicFunction(State state) {
    int sum = 0;
    vector<int> positions = state.getPositions();
    for (int i = 0; i < positions.size(); i++){
        sum += distance(positions[i], targets[i], width);
    }
    return sum;
}

MICheuristic::MICheuristic(vector<int> m_targets, int m_width) {
    targets = std::move(m_targets);
    width = m_width;
}

int MICheuristic::heuristicFunction(State state) {
    int Max = 0;
    vector<int> positions = state.getPositions();
    for (int i = 0; i < state.getAgentToAssign(); i++){
        Max = max(Max, distance(positions[i], targets[i], width));
    }
    for (int i = state.getAgentToAssign(); i < positions.size(); i++){
        Max = max(Max, distance(positions[i], targets[i], width)-1);
    }
    return Max;
}
