//
// Created by Arthur Mahy on 13/02/2023.
//

#include "SingleAgentSpaceTimeProblem.h"

//#define DEBUG

#ifdef DEBUG
#define LOG(str) cout << str << endl;
#else
#define LOG(str)
#endif

SingleAgentSpaceTimeProblem::SingleAgentSpaceTimeProblem(Graph m_graph, int m_start, int m_target,
                                                         ObjectiveFunction m_obj_function,
                                                         const set<PositionTimeConstraint> &m_setOfConstraints,
                                                         int m_numberOfTheAgent) : Problem(m_graph){
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
        for (PositionTimeConstraint constraint : setOfConstraints){
            int position = get<0>(constraint);
            int time = get<1>(constraint);
            setOfConstraintsMap[time].insert(position);
            LOG("   (" << numberOfTheAgent << ", " << position << ", " << time << ")");
        }
    }
    LOG(" ");

}

shared_ptr<State> SingleAgentSpaceTimeProblem::getStartState() {
    auto pointer = make_shared<SingleAgentSpaceTimeState>(start,0);
    return pointer;
}

bool SingleAgentSpaceTimeProblem::isGoalState(shared_ptr<State> state) {
    auto SAstate = dynamic_pointer_cast<SingleAgentSpaceTimeState>(state);
    return SAstate->getPosition()==target;
}

// Returns true if the agent is allowed to be at position at time (according to the set of constraints of the problem)
bool notInForbiddenPositions(int position, int time, map<int, set<int>> setOfConstraintsMap){
    if (setOfConstraintsMap.count(time)){
        set<int> v = setOfConstraintsMap[time];
        if (v.count(position)){
            return false;
        } else {
            return true;
        }
    }
    return true;
}

vector<Double> SingleAgentSpaceTimeProblem::getSuccessors(shared_ptr<State> state) {
    vector<Double> successors;
    auto SAstate = dynamic_pointer_cast<SingleAgentSpaceTimeState>(state);
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
