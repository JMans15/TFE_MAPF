//
// Created by Arthur Mahy on 16/01/2023.
//

#include "SingleAgentProblem.h"

//#define DEBUG

#ifdef DEBUG
#define LOG(str) cout << str << endl;
#else
#define LOG(str)
#endif

SingleAgentProblem::SingleAgentProblem(Graph m_graph, int m_start, int m_target) : Problem(m_graph) {
    start = m_start;
    target = m_target;
    numberOfAgents = 1;
    LOG("==== Single Agent Problem ====");
    LOG("Start position of each agent : " << start);
    LOG("Target position of each agent : " << target);
    LOG(" ");
}

shared_ptr<State> SingleAgentProblem::getStartState() {
    auto pointer = make_shared<SingleAgentState>(start,0);
    return pointer;
}

bool SingleAgentProblem::isGoalState(shared_ptr<State> state) {
    auto SAstate = dynamic_pointer_cast<SingleAgentState>(state);
    return SAstate->getPosition()==target;
}

vector<Double> SingleAgentProblem::getSuccessors(shared_ptr<State> state) {
    vector<Double> successors;
    auto SAstate = dynamic_pointer_cast<SingleAgentState>(state);
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
