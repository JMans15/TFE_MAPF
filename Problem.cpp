//
// Created by Arthur Mahy on 09/11/2022.
//

#include "Problem.h"

#include <utility>

Problem::Problem(Graph m_graph, vector<int> m_starts, vector<int> m_targets, string m_obj_function) : graph(m_graph) {
    starts = std::move(m_starts);
    targets = std::move(m_targets);
    numberOfAgents = starts.size();
    cout << "==== Problem ====" << endl;
    cout << "Number of agents : " << numberOfAgents << endl;
    obj_function = std::move(m_obj_function);
    if (obj_function!="SumOfCosts" and obj_function!="Makespan" and obj_function!="Fuel"){
        cout << "The input for the objective function is not correct." << endl;
        cout << "So, the default objective function will be applied." << endl;
        obj_function = "Fuel";
    }
    cout << "Objective function : " << obj_function << endl;
    cout << "Start position of each agent :" << endl;
    for (int i = 0; i < numberOfAgents; i++){
        cout << " - Agent " << i << " : " << starts[i] << endl;
        if (graph.getneighbors(starts[i]).empty()){
            cout << "   The start position of agent "<< i << " is unreachable." << endl;
        }
    }
    cout << "Target position of each agent :" << endl;
    for (int i = 0; i < numberOfAgents; i++){
        cout << " - Agent " << i << " : " << targets[i] << endl;
        if (graph.getneighbors(targets[i]).empty()){
            cout << "   The target position of agent "<< i << " is unreachable." << endl;
        }
    }
    cout << "=================" << endl;
}

State Problem::getStartState() const {
    return {starts,0,0};
}

bool Problem::isGoalState(State state) const {
    return state.getPositions()==targets;
}

// Returns true if position is not already occupied by assigned agents
bool notAlreadyOccupied(int position, vector<int> positions, int agentToAssign){
    for (int i = 0; i < agentToAssign; i++){
        if (positions[i]==position){
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
    int costMovement;
    int costWait;
    if (obj_function=="Fuel"){
        costMovement = 1;
        costWait = 0;
    } else if (obj_function=="Makespan"){
        if (agentToAssign==0){
            costMovement = 1;
            costWait = 1;
        } else {
            costMovement = 0;
            costWait = 0;
        }
    } else { // obj_function=="SumOfCosts"
        costMovement = 1;
        costWait = 1; // here we should put costWait = O when the agent is at his target position and won't move anymore
    }
    if (agentToAssign==0){
        nextT = t+1;
    } else {
        nextT = t;
    }
    if (agentToAssign==numberOfAgents-1){
        nextAgentToAssign = 0;
    } else {
        nextAgentToAssign = agentToAssign+1;
    }
    for (int j : graph.getneighbors(positions[agentToAssign])){
        vector<int> newpositions(positions);
        newpositions[agentToAssign] = j;
        if (notAlreadyOccupied(j, positions, agentToAssign)){
            string action = "Between time "+ to_string(nextT-1)+" and time "+to_string(nextT)+", agent "+to_string(agentToAssign)+" goes from position "+to_string(positions[agentToAssign])+" to position "+to_string(j);
            successors.emplace_back(State(newpositions, nextT, nextAgentToAssign), action, costMovement);
        }
    }
    if (notAlreadyOccupied(positions[agentToAssign], positions, agentToAssign)){
        string action = "Between time "+ to_string(nextT-1)+" and time "+to_string(nextT)+", agent "+to_string(agentToAssign)+" waits at position "+to_string(positions[agentToAssign]);
        successors.emplace_back(State(positions, nextT, nextAgentToAssign), action, costWait);
    }
    return successors;
}

Graph Problem::getGraph() const {
    return graph;
}

vector<int> Problem::getTargets() const {
    return targets;
}

string Problem::getObjFunction() const {
    return obj_function;
}

int distance(int a, int b, int width) {
    int ax, ay, bx, by;
    ax = (int) a / width; ay = a % width;
    bx = (int) b / width; by = b % width;
    return abs(ax-bx) + abs(ay-by);
}

// Sum of Individual Costs heuristic (for SumOfCosts and Fuel objective functions)
int Problem::SICheuristic(State state, const Problem& problem){
    int sum = 0;
    vector<int> positions = state.getPositions();
    for (int i = 0; i < positions.size(); i++){
        sum += distance(positions[i], problem.getTargets()[i], problem.getGraph().getWidth());
    }
    return sum;
}

// Maximum Individual Cost heuristic (for Makespan objective function)
int Problem::MICheuristic(State state, const Problem& problem){
    int Max = 0;
    vector<int> positions = state.getPositions();
    for (int i = 0; i < state.getAgentToAssign(); i++){
        Max = max(Max, distance(positions[i], problem.getTargets()[i], problem.getGraph().getWidth()));
    }
    for (int i = state.getAgentToAssign(); i < positions.size(); i++){
        Max = max(Max, distance(positions[i], problem.getTargets()[i], problem.getGraph().getWidth())-1);
    }
    return Max;
}
