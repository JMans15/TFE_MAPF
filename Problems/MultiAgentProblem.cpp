//
// Created by Arthur Mahy on 04/01/2023.
//

#include "MultiAgentProblem.h"

#include <algorithm>

MultiAgentProblem::MultiAgentProblem(std::shared_ptr<Graph> graph, std::vector<int> starts, std::vector<int> targets,
                                     ObjectiveFunction objective, const std::vector<int>& m_agentIds, const std::set<Constraint> &setOfConstraints)
    : Problem(graph, starts.size())
    , starts(starts)
    , targets(targets)
    , agentIds(m_agentIds)
    , objective(objective)
    , setOfConstraints(setOfConstraints)
{
    LOG("==== Multi Agent Problem ====");
    LOG("Number of agents : " << numberOfAgents)
    
    if (objective != SumOfCosts && objective != Makespan && objective != Fuel) {
        LOG("The input for the objective function is not correct.");
        LOG("So, the default objective function will be applied.");
        objective = Fuel;
    }
    if (objective == SumOfCosts){
        LOG("Objective function : SumOfCosts");
    } else if (objective == Makespan){
        LOG("Objective function : Makespan");
    } else {
        LOG("Objective function : Fuel");
    }
    if (m_agentIds.empty()){
        for (int a = 0; a < numberOfAgents; a++){
            agentIds.emplace_back(a);
        }
    }
    for (int i = 0; i < numberOfAgents; i++){
        idToIndex[agentIds[i]] = i;
    }
    LOG("Start position of each agent :");
    for (int i = 0; i < numberOfAgents; i++) {
        LOG(" - Agent " << agentIds[i] << " : " << starts[i]);
        if (graph->getNeighbors(starts[i]).empty()) {
            LOG("   The start position of agent "<< agentIds[i] << " is unreachable.");
        }
    }
    LOG("Target position of each agent :");
    for (int i = 0; i < numberOfAgents; i++) {
        LOG(" - Agent " << agentIds[i] << " : " << targets[i]);
        if (graph->getNeighbors(targets[i]).empty()) {
            LOG("   The target position of agent "<< agentIds[i] << " is unreachable.");
        }
    }
    if (!setOfConstraints.empty()) {
        LOG("The problem has the following constraints :");
        for (Constraint constraint : setOfConstraints) {
            LOG("   (" << constraint.agent << ", " << constraint.position << ", " << constraint.time << ")");
        }
    }
    LOG(" ");
}

std::shared_ptr<MultiAgentState> MultiAgentProblem::getStartState() const {
    return std::make_shared<MultiAgentState>(starts, starts, 0 , 0, true);
}

bool MultiAgentProblem::isGoalState(std::shared_ptr<MultiAgentState> state) const {
    auto positions = state->getPrePositions();
    for (int i = 0; i < numberOfAgents; i++) {
        if (positions[i] != targets[i]) {
            return false;
        }
    }
    return true;
}

// Returns true if position is not already occupied by assigned agents
bool notAlreadyOccupiedPosition(int position, std::vector<int> &positions, int agentToAssign) {
    for (int i = 0; i < agentToAssign; i++){
        if (positions[i] == position){
            return false;
        }
    }
    return true;
}

// Returns true if the edge (position, positions[agentToAssign]) is not already occupied by assigned agents
bool notAlreadyOccupiedEdge(int position, const std::vector<int> &positions, int agentToAssign, const std::vector<int> &prePositions) {
    for (int i = 0; i < agentToAssign; i++) {
        if (prePositions[i] == position && positions[i] == positions[agentToAssign]){
            return false;
        }
    }
    return true;
}

// Returns true if agent is allowed to be at position at time (according to the set of constraints of the problem)
bool MultiAgentProblem::notInForbiddenPositions(int position, int agent, int time) const {
    return setOfConstraints.find({agentIds[agent], position, time}) == setOfConstraints.end();
}

std::vector<std::pair<std::shared_ptr<MultiAgentState>, int>> MultiAgentProblem::getSuccessors(std::shared_ptr<MultiAgentState> state) const {
    std::vector<std::pair<std::shared_ptr<MultiAgentState>, int>> successors;
    auto positions = state->getPositions();
    int agentToAssign = state->getAgentToAssign();
    int t = state->getTimestep();
    int nextAgentToAssign;
    int nextT;
    int costMovement;
    int costWait;
    bool isStandard;
    if (objective == Fuel){
        costMovement = 1;
        costWait = 0;
    } else if (objective == Makespan) {
        if (agentToAssign == 0){
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
    if (agentToAssign == 0) {
        nextT = t+1;
    } else {
        nextT = t;
    }
    if (agentToAssign == numberOfAgents-1) {
        // we are assigning a position to the last agent,
        // the next state will be standard
        nextAgentToAssign = 0;
        isStandard = true;
    } else {
        nextAgentToAssign = agentToAssign+1;
        isStandard = false;
    }

    if (objective != SumOfCosts){

        // Move
        for (int j : graph->getNeighbors(positions[agentToAssign])) {
            vector<int> newpositions(positions);
            newpositions[agentToAssign] = j;
            if (notAlreadyOccupiedPosition(j, positions, agentToAssign) && notAlreadyOccupiedEdge(j, positions, agentToAssign, state->getPrePositions()) && notInForbiddenPositions(j, agentToAssign, nextT)) {
                auto successor = std::make_shared<MultiAgentState>(newpositions, positions, nextT, nextAgentToAssign, isStandard);
                successors.emplace_back(successor, costMovement);
            }
        }

        // Wait
        if (notAlreadyOccupiedPosition(positions[agentToAssign], positions, agentToAssign) && notInForbiddenPositions(positions[agentToAssign], agentToAssign, nextT)) {
            auto successor = std::make_shared<MultiAgentState>(positions, positions, nextT, nextAgentToAssign, isStandard);
            successors.emplace_back(successor, costWait);
        }

    } else { // obj_function=="SumOfCosts"
        auto cannotMove = state->getCannotMove();
        if (state->canMove(agentToAssign)){ // agentToAssign is allowed to move

            // Move
            for (int j : graph->getNeighbors(positions[agentToAssign])) {
                vector<int> newpositions(positions);
                newpositions[agentToAssign] = j;
                if (notAlreadyOccupiedPosition(j, positions, agentToAssign) && notAlreadyOccupiedEdge(j, positions, agentToAssign, state->getPrePositions()) && notInForbiddenPositions(j, agentToAssign, nextT)) {
                    auto successor = std::make_shared<MultiAgentState>(newpositions, positions, nextT, nextAgentToAssign, isStandard, cannotMove);
                    successors.emplace_back(successor, costMovement);
                }
            }

            // Wait
            if (positions[agentToAssign]!=targets[agentToAssign]) { // agentToAssign not at his target position
                costWait = 1;
                if (notAlreadyOccupiedPosition(positions[agentToAssign], positions, agentToAssign) && notInForbiddenPositions(positions[agentToAssign], agentToAssign, nextT)) {
                    auto successor = std::make_shared<MultiAgentState>(positions, positions, nextT, nextAgentToAssign, isStandard, cannotMove);
                    successors.emplace_back(successor, costWait);
                }
            } else { // agentToAssign is at his target position
                // agentToAssign can still move in the future
                costWait = 1;
                if (notAlreadyOccupiedPosition(positions[agentToAssign], positions, agentToAssign) && notInForbiddenPositions(positions[agentToAssign], agentToAssign, nextT)) {
                    auto successor = std::make_shared<MultiAgentState>(positions, positions, nextT, nextAgentToAssign, isStandard, cannotMove);
                    successors.emplace_back(successor, costWait);
                }

                // we are forcing agentToAssign to not move in the future
                costWait = 0;
                cannotMove.push_back(agentToAssign);
                if (notAlreadyOccupiedPosition(positions[agentToAssign], positions, agentToAssign) && notInForbiddenPositions(positions[agentToAssign], agentToAssign, nextT)) {
                    auto successor = std::make_shared<MultiAgentState>(positions, positions, nextT, nextAgentToAssign, isStandard, cannotMove);
                    successors.emplace_back(successor, costWait);
                }
            }


        } else { // agentToAssign is not allowed to move
            costWait = 0;
            if (notAlreadyOccupiedPosition(positions[agentToAssign], positions, agentToAssign) && notInForbiddenPositions(positions[agentToAssign], agentToAssign, nextT)) {
                auto successor = std::make_shared<MultiAgentState>(positions, positions, nextT, nextAgentToAssign, isStandard, cannotMove);
                successors.emplace_back(successor, costWait);
            }
        }
    }
    return successors;
}

std::unordered_map<int, std::vector<int>> MultiAgentProblem::getPositions(std::vector<std::shared_ptr<MultiAgentState>> states) const {
    std::unordered_map<int, std::vector<int>> positions;

    for (const auto& state : states) {
        if (state->isStandard()){
            for (int agent = 0; agent < numberOfAgents; agent++){
                positions[agentIds[agent]].push_back(state->getPositions()[agent]);
            }
        }
    }

    return positions;
}


const std::vector<int>& MultiAgentProblem::getStarts() const {
    return starts;
}

const std::vector<int>& MultiAgentProblem::getTargets() const {
    return targets;
}

ObjectiveFunction MultiAgentProblem::getObjFunction() {
    return objective;
}

const std::set<Constraint>& MultiAgentProblem::getSetOfConstraints() const {
    return setOfConstraints;
}

std::vector<int> MultiAgentProblem::getAgentIds() const {
    return agentIds;
}

int MultiAgentProblem::getStartOf(int id) {
    return starts[idToIndex[id]];
}

int MultiAgentProblem::getTargetOf(int id) {
    return targets[idToIndex[id]];
}

