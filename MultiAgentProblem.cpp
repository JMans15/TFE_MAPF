//
// Created by Arthur Mahy on 04/01/2023.
//

#include "MultiAgentProblem.h"
#include <algorithm>

//#define DEBUG

#ifdef DEBUG
#define LOG(str) cout << str << endl;
#else
#define LOG(str)
#endif

MultiAgentProblem::MultiAgentProblem(Graph m_graph, vector<int> m_starts, vector<int> m_targets,
                                     ObjectiveFunction m_obj_function,
                                     const set<Constraint> &m_setOfConstraints) : Problem(m_graph) {
    starts = std::move(m_starts);
    targets = std::move(m_targets);
    numberOfAgents = starts.size();
    LOG("==== Multi Agent Problem ====");
    LOG("Number of agents : " << numberOfAgents)
    obj_function = m_obj_function;
    if (obj_function!=SumOfCosts and obj_function!=Makespan and obj_function!=Fuel){
        LOG("The input for the objective function is not correct.");
        LOG("So, the default objective function will be applied.");
        obj_function = Fuel;
    }
    if (obj_function==SumOfCosts){
        LOG("Objective function : SumOfCosts");
    } else if (obj_function==Makespan){
        LOG("Objective function : Makespan");
    } else {
        LOG("Objective function : Fuel");
    }
    LOG("Start position of each agent :");
    for (int i = 0; i < numberOfAgents; i++){
        LOG(" - Agent " << i << " : " << starts[i]);
        if (graph.getNeighbors(starts[i]).empty()){
            LOG("   The start position of agent "<< i << " is unreachable.");
        }
    }
    LOG("Target position of each agent :");
    for (int i = 0; i < numberOfAgents; i++){
        LOG(" - Agent " << i << " : " << targets[i]);
        if (graph.getNeighbors(targets[i]).empty()){
            LOG("   The target position of agent "<< i << " is unreachable.");
        }
    }
    setOfConstraints = m_setOfConstraints;
    if (not setOfConstraints.empty()){
        LOG("The problem has the following constraints :");
        for (Constraint constraint : setOfConstraints){
            int agent = get<0>(constraint);
            int position = get<1>(constraint);
            int time = get<2>(constraint);
            setOfConstraintsMap[agent][time].insert(position);
            LOG("   (" << agent << ", " << position << ", " << time << ")");
        }
    }
    LOG(" ");
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
bool notInForbiddenPositions(int position, int agent, int time, map<int, map<int, set<int>>> setOfConstraintsMap){
    if (setOfConstraintsMap.count(agent)){
        if (setOfConstraintsMap[agent].count(time)){
            set<int> v = setOfConstraintsMap[agent][time];
            if (v.count(position)){
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
            if (notAlreadyOccupiedPosition(j, positions, agentToAssign) && notAlreadyOccupiedEdge(j, positions, agentToAssign, MAstate->getPrePositions()) && notInForbiddenPositions(j, agentToAssign, nextT, setOfConstraintsMap)){
                auto* pointer = new MultiAgentState(newpositions, nextT, nextAgentToAssign, isStandard, positions);
                successors.emplace_back(pointer, costMovement);
            }
        }

        // Wait
        if (notAlreadyOccupiedPosition(positions[agentToAssign], positions, agentToAssign) && notInForbiddenPositions(positions[agentToAssign], agentToAssign, nextT, setOfConstraintsMap)){
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
                if (notAlreadyOccupiedPosition(j, positions, agentToAssign) && notAlreadyOccupiedEdge(j, positions, agentToAssign, MAstate->getPrePositions()) && notInForbiddenPositions(j, agentToAssign, nextT, setOfConstraintsMap)){
                    auto* pointer = new MultiAgentState(newpositions, nextT, nextAgentToAssign, isStandard, positions, cannotMove);
                    successors.emplace_back(pointer, costMovement);
                }
            }

            // Wait
            if (positions[agentToAssign]!=targets[agentToAssign]){ // agentToAssign not at his target position
                costWait = 1;
                if (notAlreadyOccupiedPosition(positions[agentToAssign], positions, agentToAssign) && notInForbiddenPositions(positions[agentToAssign], agentToAssign, nextT, setOfConstraintsMap)){
                    auto* pointer = new MultiAgentState(positions, nextT, nextAgentToAssign, isStandard, positions, cannotMove);
                    successors.emplace_back(pointer, costWait);
                }
            } else { // agentToAssign is at his target position
                // agentToAssign can still move in the future
                costWait = 1;
                if (notAlreadyOccupiedPosition(positions[agentToAssign], positions, agentToAssign) && notInForbiddenPositions(positions[agentToAssign], agentToAssign, nextT, setOfConstraintsMap)){
                    auto* pointer = new MultiAgentState(positions, nextT, nextAgentToAssign, isStandard, positions, cannotMove);
                    successors.emplace_back(pointer, costWait);
                }

                // we are forcing agentToAssign to not move in the future
                costWait = 0;
                cannotMove.push_back(agentToAssign);
                if (notAlreadyOccupiedPosition(positions[agentToAssign], positions, agentToAssign) && notInForbiddenPositions(positions[agentToAssign], agentToAssign, nextT, setOfConstraintsMap)){
                    auto* pointer = new MultiAgentState(positions, nextT, nextAgentToAssign, isStandard, positions, cannotMove);
                    successors.emplace_back(pointer, costWait);
                }
            }


        } else { // agentToAssign is not allowed to move
            costWait = 0;
            if (notAlreadyOccupiedPosition(positions[agentToAssign], positions, agentToAssign) && notInForbiddenPositions(positions[agentToAssign], agentToAssign, nextT, setOfConstraintsMap)){
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
