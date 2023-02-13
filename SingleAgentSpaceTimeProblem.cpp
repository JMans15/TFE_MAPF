//
// Created by Arthur Mahy on 13/02/2023.
//

#include "SingleAgentSpaceTimeProblem.h"

#define LOG(str) if (verbose) {cout << str << endl;}

SingleAgentSpaceTimeProblem::SingleAgentSpaceTimeProblem(Graph m_graph, int m_start, int m_target,
                                                         ObjectiveFunction m_obj_function,
                                                         const vector<Constraint> &m_setOfConstraints,
                                                         int m_numberOfTheAgent, int verbose) : Problem(m_graph){
    start = m_start;
    target = m_target;
    numberOfAgents = 1;
    numberOfTheAgent = m_numberOfTheAgent;
    LOG("==== Single Agent Space Time Problem ====");
    LOG("Number of the agent : " << numberOfTheAgent);
    obj_function = m_obj_function;
    if (obj_function!=Makespan and obj_function!=Fuel){
        LOG("The input for the objective function is not correct.");
        LOG("So, the default objective function will be applied.");
        obj_function = Fuel;
    }
    if (obj_function==Makespan){
        LOG("Objective function : Makespan (costWait = 1)");
    } else {
        LOG("Objective function : Fuel (costWait = 0)");
    }
    LOG("Start position of each agent : " << start);
    LOG("Target position of each agent : " << target);
    setOfConstraints = m_setOfConstraints;
    if (not setOfConstraints.empty()){
        LOG("The problem has the following constraints :");
        for (int i = 0; i < setOfConstraints.size(); i++){
            Constraint constraint = setOfConstraints[i];
            int agent = get<0>(constraint);
            int position = get<1>(constraint);
            int time = get<2>(constraint);
            if (agent==numberOfTheAgent){
                setOfConstraintsMap[time].push_back(position);
            }
            LOG("   (" << agent << ", " << position << ", " << time << ")");
        }
    }
    LOG("=================");

}

State *SingleAgentSpaceTimeProblem::getStartState() {
    auto* pointer = new SingleAgentSpaceTimeState(start,0);
    return pointer;
}

bool SingleAgentSpaceTimeProblem::isGoalState(State *state) {
    auto* SAstate = dynamic_cast<SingleAgentSpaceTimeState *>(state);
    return SAstate->getPosition()==target;
}

// Returns true if the agent is allowed to be at position at time (according to the set of constraints of the problem)
bool notInForbiddenPositions(int position, int time, map<int, vector<int>> setOfConstraintsMap){
    if (setOfConstraintsMap.count(time)){
        vector<int> v = setOfConstraintsMap[time];
        if (std::find(v.begin(), v.end(), position) != v.end()){
            return false;
        } else {
            return true;
        }
    }
    return true;
}

vector<Double> SingleAgentSpaceTimeProblem::getSuccessors(State *state) {
    vector<Double> successors;
    auto* SAstate = dynamic_cast<SingleAgentSpaceTimeState *>(state);
    int position = SAstate->getPosition();
    int t = SAstate->getTimestep();
    int nextT = t+1;
    int costMovement;
    int costWait;
    if (obj_function==Fuel){
        costMovement = 1;
        costWait = 0;
    } else { // obj_function==Makespan
        costMovement = 1;
        costWait = 1;
    }

    // Move
    for (int newposition : graph.getNeighbors(position)){
        if (notInForbiddenPositions(newposition, nextT, setOfConstraintsMap)){
            auto* pointer = new SingleAgentSpaceTimeState(newposition, nextT);
            successors.emplace_back(pointer, costMovement);
        }
    }

    // Wait
    if (notInForbiddenPositions(position, nextT, setOfConstraintsMap)){
        auto* pointer = new SingleAgentSpaceTimeState(position, nextT);
        successors.emplace_back(pointer, costWait);
    }

    return successors;
}

vector<int> SingleAgentSpaceTimeProblem::getStarts() {
    vector<int> tab;
    tab.push_back(start);
    return tab;
}

vector<int> SingleAgentSpaceTimeProblem::getTargets() {
    vector<int> tab;
    tab.push_back(target);
    return tab;
}

ObjectiveFunction SingleAgentSpaceTimeProblem::getObjFunction() {
    return obj_function;
}
