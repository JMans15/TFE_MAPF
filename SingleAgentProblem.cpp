//
// Created by Arthur Mahy on 16/01/2023.
//

#include "SingleAgentProblem.h"

SingleAgentProblem::SingleAgentProblem(Graph m_graph, int m_start, int m_target) : Problem(m_graph) {
    start = m_start;
    target = m_target;
    numberOfAgents = 1;
    cout << "==== Single Agent Problem ====" << endl;
    cout << "Start position of each agent : " << start << endl;
    cout << "Target position of each agent : " << target << endl;
    cout << "=================" << endl;
}

State* SingleAgentProblem::getStartState() {
    auto* pointer = new SingleAgentState(start,0);
    return pointer;
}

bool SingleAgentProblem::isGoalState(State* state) {
    auto* SAstate = dynamic_cast<SingleAgentState *>(state);
    return SAstate->getPosition()==target;
}

vector<Double> SingleAgentProblem::getSuccessors(State* state) {
    vector<Double> successors;
    auto* SAstate = dynamic_cast<SingleAgentState *>(state);
    int position = SAstate->getPosition();
    int t = SAstate->getTimestep();
    int nextT = t + 1;
    int costMovement = 1;

    // Move
    for (int newposition : graph.getNeighbors(position)){
        auto* pointer = new SingleAgentState(newposition, nextT);
        successors.emplace_back(pointer, costMovement);
    }

    return successors;
}

vector<int> SingleAgentProblem::getStarts() {
    vector<int> tab;
    tab.push_back(start);
    return tab;
}

vector<int> SingleAgentProblem::getTargets() {
    vector<int> tab;
    tab.push_back(target);
    return tab;
}
