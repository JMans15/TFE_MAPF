//
// Created by Arthur Mahy on 04/01/2023.
//

#include "MultiAgentProblem.h"

MultiAgentProblem::MultiAgentProblem(Graph m_graph, vector<int> m_starts, vector<int> m_targets,
                                     ObjectiveFunction m_obj_function,
                                     const vector<Constraint> &m_setOfConstraints) : Problem(m_graph) {
    starts = std::move(m_starts);
    targets = std::move(m_targets);
    numberOfAgents = starts.size();
    cout << "==== Multi Agent Problem ====" << endl;
    cout << "Number of agents : " << numberOfAgents << endl;
    obj_function = m_obj_function;
    if (obj_function!=SumOfCosts and obj_function!=Makespan and obj_function!=Fuel){
        cout << "The input for the objective function is not correct." << endl;
        cout << "So, the default objective function will be applied." << endl;
        obj_function = Fuel;
    }
    if (obj_function==SumOfCosts){
        cout << "Objective function : SumOfCosts" << endl;
    } else if (obj_function==Makespan){
        cout << "Objective function : Makespan" << endl;
    } else {
        cout << "Objective function : Fuel" << endl;
    }
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

State* MultiAgentProblem::getStartState() {
    auto* pointer = new MultiAgentState(starts,0,0,true,starts);
    return pointer;
}

bool MultiAgentProblem::isGoalState(State* state) {
    return state->getPositions()==targets;
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

vector<Double> MultiAgentProblem::getSuccessors(State* state) {
    vector<Double> successors;
    auto* MAstate = dynamic_cast<MultiAgentState *>(state);
    vector<int> positions = MAstate->getPositions();
    int agentToAssign = MAstate->getAgentToAssign();
    int t = MAstate->getTimestep();
    int nextAgentToAssign;
    int nextT;
    int costMovement;
    int costWait;
    bool isStandard;
    if (obj_function==Fuel){
        costMovement = 1;
        costWait = 0;
    } else if (obj_function==Makespan){
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

    if (obj_function!=SumOfCosts){

        // Move
        for (int j : graph.getNeighbors(positions[agentToAssign])){
            vector<int> newpositions(positions);
            newpositions[agentToAssign] = j;
            if (notAlreadyOccupiedPosition(j, positions, agentToAssign) && notAlreadyOccupiedEdge(j, positions, agentToAssign, MAstate->getPrePositions()) && NotInForbiddenPositions(j, agentToAssign, nextT, setOfConstraintsMap)){
                auto* pointer = new MultiAgentState(newpositions, nextT, nextAgentToAssign, isStandard, positions);
                successors.emplace_back(pointer, costMovement);
            }
        }

        // Wait
        if (notAlreadyOccupiedPosition(positions[agentToAssign], positions, agentToAssign) && NotInForbiddenPositions(positions[agentToAssign], agentToAssign, nextT, setOfConstraintsMap)){
            auto* pointer = new MultiAgentState(positions, nextT, nextAgentToAssign, isStandard, positions);
            successors.emplace_back(pointer, costWait);
        }

    } else { // obj_function=="SumOfCosts"
        vector<int> cannotMove = MAstate->getCannotMove();
        if (MAstate->canMove(agentToAssign)){ // agentToAssign is allowed to move

            // Move
            for (int j : graph.getNeighbors(positions[agentToAssign])){
                vector<int> newpositions(positions);
                newpositions[agentToAssign] = j;
                if (notAlreadyOccupiedPosition(j, positions, agentToAssign) && notAlreadyOccupiedEdge(j, positions, agentToAssign, MAstate->getPrePositions()) && NotInForbiddenPositions(j, agentToAssign, nextT, setOfConstraintsMap)){
                    auto* pointer = new MultiAgentState(newpositions, nextT, nextAgentToAssign, isStandard, positions, cannotMove);
                    successors.emplace_back(pointer, costMovement);
                }
            }

            // Wait
            if (positions[agentToAssign]!=targets[agentToAssign]){ // agentToAssign not at his target position
                costWait = 1;
                if (notAlreadyOccupiedPosition(positions[agentToAssign], positions, agentToAssign) && NotInForbiddenPositions(positions[agentToAssign], agentToAssign, nextT, setOfConstraintsMap)){
                    auto* pointer = new MultiAgentState(positions, nextT, nextAgentToAssign, isStandard, positions, cannotMove);
                    successors.emplace_back(pointer, costWait);
                }
            } else { // agentToAssign is at his target position
                // agentToAssign can still move in the future
                costWait = 1;
                if (notAlreadyOccupiedPosition(positions[agentToAssign], positions, agentToAssign) && NotInForbiddenPositions(positions[agentToAssign], agentToAssign, nextT, setOfConstraintsMap)){
                    auto* pointer = new MultiAgentState(positions, nextT, nextAgentToAssign, isStandard, positions, cannotMove);
                    successors.emplace_back(pointer, costWait);
                }

                // we are forcing agentToAssign to not move in the future
                costWait = 0;
                cannotMove.push_back(agentToAssign);
                if (notAlreadyOccupiedPosition(positions[agentToAssign], positions, agentToAssign) && NotInForbiddenPositions(positions[agentToAssign], agentToAssign, nextT, setOfConstraintsMap)){
                    auto* pointer = new MultiAgentState(positions, nextT, nextAgentToAssign, isStandard, positions, cannotMove);
                    successors.emplace_back(pointer, costWait);
                }
            }


        } else { // agentToAssign is not allowed to move
            costWait = 0;
            if (notAlreadyOccupiedPosition(positions[agentToAssign], positions, agentToAssign) && NotInForbiddenPositions(positions[agentToAssign], agentToAssign, nextT, setOfConstraintsMap)){
                auto* pointer = new MultiAgentState(positions, nextT, nextAgentToAssign, isStandard, positions, cannotMove);
                successors.emplace_back(pointer, costWait);
            }
        }
    }
    return successors;
}

vector<int> MultiAgentProblem::getStarts() {
    return starts;
}

vector<int> MultiAgentProblem::getTargets() {
    return targets;
}

ObjectiveFunction MultiAgentProblem::getObjFunction() {
    return obj_function;
}
