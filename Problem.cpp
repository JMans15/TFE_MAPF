//
// Created by Arthur Mahy on 09/11/2022.
//

#include "Problem.h"

#include <utility>

Problem::Problem(Graph m_graph, vector<int> m_starts, vector<int> m_targets) : graph(m_graph) {
    starts = std::move(m_starts);
    targets = std::move(m_targets);
    numberOfAgents = starts.size();
    cout << "==== Problem ====" << endl;
    cout << "Number of agents : " << numberOfAgents << endl;
    cout << "Starts :" << endl;
    for (int i = 0; i < numberOfAgents; i++){
        cout << " - Agent " << i << " : " << starts[i] << endl;
    }
    cout << "Targets :" << endl;
    for (int i = 0; i < numberOfAgents; i++){
        cout << " - Agent " << i << " : " << targets[i] << endl;
    }
    cout << "=================" << endl;
}

State Problem::getStartState() const {
    return {starts,0,0};
}

bool Problem::isGoalState(State state) const {
    return state.getPositions()==targets;
}

bool notIn(int j, const vector<int>& positions){
    for (int i : positions){
        if (i==j){
            return false;
        }
    }
    return true;
}

// Extend a state thanks to Operator Decomposition
vector<Triple> Problem::getSuccessors(State state) const {
    vector<Triple> successors;
    vector<int> positions = state.getPositions();
    int agentToAssign = state.getAgentToAssign();
    int t = state.getTimestep();
    int nextAgentToAssign;
    int nextT;
    if (agentToAssign==numberOfAgents-1){
        nextAgentToAssign = 0;
        nextT = t+1;
    } else {
        nextAgentToAssign = agentToAssign+1;
        nextT = t;
    }
    for (int j : graph.getneighbors(positions[agentToAssign])){
        vector<int> newpositions(positions);
        newpositions[agentToAssign] = j;
        if (notIn(j, positions)){
            string action = "At time "+ to_string(t)+" agent "+to_string(agentToAssign)+" goes from position "+to_string(positions[agentToAssign])+" to position "+to_string(j);
            successors.emplace_back(State(newpositions, nextT, nextAgentToAssign), action, 1);
        }
    }
    string action = "At time "+ to_string(t)+" agent "+to_string(agentToAssign)+" waits at position "+to_string(positions[agentToAssign]);
    successors.emplace_back(State(positions, nextT, nextAgentToAssign), action, 0);
    return successors;
}

Graph Problem::getGraph() const {
    return graph;
}

vector<int> Problem::getTargets() const {
    return targets;
}
