//
// Created by Arthur Mahy on 14/01/2023.
//

#include "Heuristic.h"
#include "SingleAgentProblem.h"
#include "AStar.h"
#include "MultiAgentState.h"
#include "SingleAgentState.h"

#include <utility>
#include <algorithm>

// Returns the Manhattan distance between position a and position b
// ignoring walls and other agents
int distance(int a, int b, int width) {
    int ax, ay, bx, by;
    ax = (int) a % width; ay = a / width;
    bx = (int) b % width; by = b / width;
    return abs(ax-bx) + abs(ay-by);
}

Manhattanheuristic::Manhattanheuristic(int m_target, int m_width) {
    target = m_target;
    width = m_width;
}

int Manhattanheuristic::heuristicFunction(shared_ptr<State> state) {
    auto SAstate = dynamic_pointer_cast<SingleAgentState>(state);
    return distance(SAstate->getPosition(), target, width);
}

OptimalDistanceheuristic::OptimalDistanceheuristic(int m_start, int m_target, const Graph& m_graph) {
    auto* prob = new SingleAgentProblem(m_graph, m_start, m_target);
    RRAStarSearch = new ReverseResumableAStar(prob);
}

int OptimalDistanceheuristic::heuristicFunction(shared_ptr<State> state) {
    auto SAstate = dynamic_pointer_cast<SingleAgentState>(state);
    return RRAStarSearch->OptimalDistance(SAstate->getPosition());
}

SICheuristic::SICheuristic(vector<int> m_targets, int m_width) {
    targets = std::move(m_targets);
    width = m_width;
}

int SICheuristic::heuristicFunction(shared_ptr<State> state) {
    auto MAstate = dynamic_pointer_cast<MultiAgentState>(state); // NEEDED ??
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

int MICheuristic::heuristicFunction(shared_ptr<State> state) {
    auto MAstate = dynamic_pointer_cast<MultiAgentState>(state);
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

SIOCheuristic::SIOCheuristic(vector<int> m_starts, vector<int> m_targets, const Graph &m_graph) {
    for (int a = 0; a < m_starts.size(); a++){
        auto* prob = new SingleAgentProblem(m_graph, m_starts[a], m_targets[a]);
        auto* RRAStarSearch = new ReverseResumableAStar(prob);
        RRAStarSearches.emplace_back(RRAStarSearch);
    }
}

int SIOCheuristic::heuristicFunction(shared_ptr<State> state) {
    auto MAstate = dynamic_pointer_cast<MultiAgentState>(state);
    int sum = 0;
    vector<int> positions = MAstate->getPositions();
    for (int i = 0; i < positions.size(); i++){
        sum += RRAStarSearches[i]->OptimalDistance(positions[i]);
    }
    return sum;
}

MIOCheuristic::MIOCheuristic(vector<int> m_starts, vector<int> m_targets, const Graph &m_graph) {
    for (int a = 0; a < m_starts.size(); a++){
        auto* prob = new SingleAgentProblem(m_graph, m_starts[a], m_targets[a]);
        auto* RRAStarSearch = new ReverseResumableAStar(prob);
        RRAStarSearches.emplace_back(RRAStarSearch);
    }
}

int MIOCheuristic::heuristicFunction(shared_ptr<State> state) {
    auto MAstate = dynamic_pointer_cast<MultiAgentState>(state);
    int Max = 0;
    vector<int> positions = MAstate->getPositions();
    for (int i = 0; i < MAstate->getAgentToAssign(); i++){
        Max = max(Max, RRAStarSearches[i]->OptimalDistance(positions[i]));
    }
    for (int i = MAstate->getAgentToAssign(); i < positions.size(); i++){
        Max = max(Max, RRAStarSearches[i]->OptimalDistance(positions[i])-1);
    }
    return Max;
}
