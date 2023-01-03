//
// Created by Arthur Mahy on 09/11/2022.
//

#include "Problem.h"

// #include <bits/stdc++.h>
#include <utility>

Problem::Problem(Graph m_graph, vector<int> m_starts, vector<int> m_targets, string m_obj_function, const vector<Constraint>& m_setOfConstraints) : graph(m_graph) {
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
        if (graph.getNeighbors(starts[i]).empty()){
            cout << "   The start position of agent "<< i << " is unreachable." << endl;
        }
    }
    cout << "Target position of each agent :" << endl;
    for (int i = 0; i < numberOfAgents; i++){
        cout << " - Agent " << i << " : " << targets[i] << endl;
        if (graph.getNeighbors(targets[i]).empty()){
            cout << "   The target position of agent "<< i << " is unreachable." << endl;
        }
    }
    setOfConstraints = m_setOfConstraints;
    if (not setOfConstraints.empty()){
        cout << "The problem has the following constraints :" << endl;
        for (int i = 0; i < setOfConstraints.size(); i++){
            Constraint constraint = setOfConstraints[i];
            int agent = get<0>(constraint);
            int position = get<1>(constraint);
            int time = get<2>(constraint);
            setOfConstraintsMap[agent][time].push_back(position);
            cout << "   (" << agent << ", " << position << ", " << time << ")" << endl;
        }
    }
    cout << "=================" << endl;
}

State Problem::getStartState() const {
    return {starts,0,0,true,starts};
}

bool Problem::isGoalState(State state) const {
    return state.getPositions()==targets;
}

// Returns true if position is not already occupied by assigned agents
bool notAlreadyOccupiedPosition(int position, vector<int> positions, int agentToAssign){
    for (int i = 0; i < agentToAssign; i++){
        if (positions[i]==position){
            return false;
        }
    }
    return true;
}

// Returns true if the edge (position, positions[agentToAssign]) is not already occupied by assigned agents
bool notAlreadyOccupiedEdge(int position, vector<int> positions, int agentToAssign, vector<int> prePositions){
    for (int i = 0; i < agentToAssign; i++){
        if (prePositions[i]==position and positions[i]==positions[agentToAssign]){
            return false;
        }
    }
    return true;
}

// Returns true if agent is allowed to be at position at time (according to the set of constraints of the problem)
bool NotInForbiddenPositions(int position, int agent, int time, map<int, map<int, vector<int>>> setOfConstraintsMap){
    if (setOfConstraintsMap.count(agent)){
        if (setOfConstraintsMap[agent].count(time)){
            vector<int> v = setOfConstraintsMap[agent][time];
            if (std::find(v.begin(), v.end(), position) != v.end()){
                return false;
            } else {
                return true;
            }
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
    bool isStandard;
    if (obj_function=="Fuel"){
        costMovement = 1;
        costWait = 0;
    } else if (obj_function=="Makespan"){
        if (agentToAssign==0){
            // we are assigning a position to the first agent,
            // we know that we will need another timestep
            costMovement = 1;
            costWait = 1;
        } else {
            costMovement = 0;
            costWait = 0;
        }
    } else { // obj_function=="SumOfCosts"
        costMovement = 1;
    }
    if (agentToAssign==0){
        nextT = t+1;
    } else {
        nextT = t;
    }
    if (agentToAssign==numberOfAgents-1){
        // we are assigning a position to the last agent,
        // the next state will be standard
        nextAgentToAssign = 0;
        isStandard = true;
    } else {
        nextAgentToAssign = agentToAssign+1;
        isStandard = false;
    }

    if (obj_function!="SumOfCosts"){

        // Move
        for (int j : graph.getNeighbors(positions[agentToAssign])){
            vector<int> newpositions(positions);
            newpositions[agentToAssign] = j;
            if (notAlreadyOccupiedPosition(j, positions, agentToAssign) && notAlreadyOccupiedEdge(j, positions, agentToAssign, state.getPrePositions()) && NotInForbiddenPositions(j, agentToAssign, nextT, setOfConstraintsMap)){
                string action = "Between time "+ to_string(nextT-1)+" and time "+to_string(nextT)+", agent "+to_string(agentToAssign)+" goes from position "+to_string(positions[agentToAssign])+" to position "+to_string(j);
                successors.emplace_back(State(newpositions, nextT, nextAgentToAssign, isStandard, positions), action, costMovement);
            }
        }

        // Wait
        if (notAlreadyOccupiedPosition(positions[agentToAssign], positions, agentToAssign) && NotInForbiddenPositions(positions[agentToAssign], agentToAssign, nextT, setOfConstraintsMap)){
            string action = "Between time "+ to_string(nextT-1)+" and time "+to_string(nextT)+", agent "+to_string(agentToAssign)+" waits at position "+to_string(positions[agentToAssign]);
            successors.emplace_back(State(positions, nextT, nextAgentToAssign, isStandard, positions), action, costWait);
        }

    } else { // obj_function=="SumOfCosts"
        vector<int> cannotMove = state.getCannotMove();
        if (state.canMove(agentToAssign)){ // agentToAssign is allowed to move

            // Move
            for (int j : graph.getNeighbors(positions[agentToAssign])){
                vector<int> newpositions(positions);
                newpositions[agentToAssign] = j;
                if (notAlreadyOccupiedPosition(j, positions, agentToAssign) && notAlreadyOccupiedEdge(j, positions, agentToAssign, state.getPrePositions()) && NotInForbiddenPositions(j, agentToAssign, nextT, setOfConstraintsMap)){
                    string action = "Between time "+ to_string(nextT-1)+" and time "+to_string(nextT)+", agent "+to_string(agentToAssign)+" goes from position "+to_string(positions[agentToAssign])+" to position "+to_string(j);
                    successors.emplace_back(State(newpositions, nextT, nextAgentToAssign, isStandard, positions, cannotMove), action, costMovement);
                }
            }

            // Wait
            if (positions[agentToAssign]!=targets[agentToAssign]){ // agentToAssign not at his target position
                costWait = 1;
                if (notAlreadyOccupiedPosition(positions[agentToAssign], positions, agentToAssign) && NotInForbiddenPositions(positions[agentToAssign], agentToAssign, nextT, setOfConstraintsMap)){
                    string action = "Between time "+ to_string(nextT-1)+" and time "+to_string(nextT)+", agent "+to_string(agentToAssign)+" waits at position "+to_string(positions[agentToAssign]);
                    successors.emplace_back(State(positions, nextT, nextAgentToAssign, isStandard, positions, cannotMove), action, costWait);
                }
            } else { // agentToAssign is at his target position
                // agentToAssign can still move in the future
                costWait = 1;
                if (notAlreadyOccupiedPosition(positions[agentToAssign], positions, agentToAssign) && NotInForbiddenPositions(positions[agentToAssign], agentToAssign, nextT, setOfConstraintsMap)){
                    string action = "Between time "+ to_string(nextT-1)+" and time "+to_string(nextT)+", agent "+to_string(agentToAssign)+" waits at position "+to_string(positions[agentToAssign]);
                    successors.emplace_back(State(positions, nextT, nextAgentToAssign, isStandard, positions, cannotMove), action, costWait);
                }

                // we are forcing agentToAssign to not move in the future
                costWait = 0;
                cannotMove.push_back(agentToAssign);
                if (notAlreadyOccupiedPosition(positions[agentToAssign], positions, agentToAssign) && NotInForbiddenPositions(positions[agentToAssign], agentToAssign, nextT, setOfConstraintsMap)){
                    string action = "Between time "+ to_string(nextT-1)+" and time "+to_string(nextT)+", agent "+to_string(agentToAssign)+" waits at position "+to_string(positions[agentToAssign]);
                    successors.emplace_back(State(positions, nextT, nextAgentToAssign, isStandard, positions, cannotMove), action, costWait);
                }
            }


        } else { // agentToAssign is not allowed to move
            costWait = 0;
            if (notAlreadyOccupiedPosition(positions[agentToAssign], positions, agentToAssign) && NotInForbiddenPositions(positions[agentToAssign], agentToAssign, nextT, setOfConstraintsMap)){
                string action = "Between time "+ to_string(nextT-1)+" and time "+to_string(nextT)+", agent "+to_string(agentToAssign)+" waits at position "+to_string(positions[agentToAssign]);
                successors.emplace_back(State(positions, nextT, nextAgentToAssign, isStandard, positions, cannotMove), action, costWait);
            }
        }
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


// Returns the Manhattan distance between position a and position b
int distance(int a, int b, int width) {
    int ax, ay, bx, by;
    ax = (int) a / width; ay = a % width;
    bx = (int) b / width; by = b % width;
    return abs(ax-bx) + abs(ay-by);
}

int Problem::SICheuristic(State state, const Problem& problem){
    int sum = 0;
    vector<int> positions = state.getPositions();
    for (int i = 0; i < positions.size(); i++){
        sum += distance(positions[i], problem.getTargets()[i], problem.getGraph().getWidth());
    }
    return sum;
}

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

Solution Problem::retrieveSolution(int numberOfVisitedStates, Node node) const {
    vector<vector<int>> positionsAtTime;
    vector<string> stringPath;
    int cost = node.getGn();
    Node* currentnode = &node;
    int numberOfTimesteps = node.getState().getTimestep();
    int oldT = numberOfTimesteps+1;
    while (currentnode->getParent() != nullptr){
        stringPath.push_back(currentnode->getAction());
        if (oldT!=currentnode->getState().getTimestep()){
            positionsAtTime.push_back(currentnode->getState().getPositions());
        }
        oldT = currentnode->getState().getTimestep();
        currentnode = currentnode->getParent();
    }
    positionsAtTime.push_back(starts);
    reverse(positionsAtTime.begin(), positionsAtTime.end());
    reverse(stringPath.begin(), stringPath.end());
    return {stringPath, cost, obj_function, numberOfVisitedStates, numberOfTimesteps+1, positionsAtTime};
}
