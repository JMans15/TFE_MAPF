//
// Created by Arthur Mahy on 04/01/2023.
//

#include "MultiAgentProblemWithConstraints.h"

#include <algorithm>

MultiAgentProblemWithConstraints::MultiAgentProblemWithConstraints(std::shared_ptr<Graph> graph, std::vector<int> starts, std::vector<int> targets,
                                                                   ObjectiveFunction objective, const std::vector<int>& m_agentIds,
                                                                   const std::set<VertexConstraint> &setOfHardVertexConstraints,
                                                                   const std::set<EdgeConstraint> &setOfHardEdgeConstraints, int maxCost,
                                                                   const std::set<VertexConstraint> &setOfSoftVertexConstraints,
                                                                   const std::set<EdgeConstraint> &setOfSoftEdgeConstraints)
    : Problem(graph, starts.size(), maxCost)
    , starts(starts)
    , targets(targets)
    , agentIds(m_agentIds)
    , objective(objective)
    , setOfHardVertexConstraints(setOfHardVertexConstraints)
    , setOfHardEdgeConstraints(setOfHardEdgeConstraints)
    , setOfSoftVertexConstraints(setOfSoftVertexConstraints)
    , setOfSoftEdgeConstraints(setOfSoftEdgeConstraints)
{
    LOG("==== Multi Agent Problem With Constraints ====");
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
    impossible = false;
    LOG("Start position of each agent :");
    for (int i = 0; i < numberOfAgents; i++) {
        LOG(" - Agent " << agentIds[i] << " : " << starts[i]);
        if (graph->getNeighbors(starts[i]).empty()) {
            LOG("   The start position of agent "<< agentIds[i] << " is unreachable.");
            impossible = true;
        }
    }
    LOG("Target position of each agent :");
    for (int i = 0; i < numberOfAgents; i++) {
        LOG(" - Agent " << agentIds[i] << " : " << targets[i]);
        if (graph->getNeighbors(targets[i]).empty()) {
            LOG("   The target position of agent "<< agentIds[i] << " is unreachable.");
            impossible = true;
        }
    }
    for (int start : starts){
        if (std::count(starts.begin(), starts.end(), start)>1){
            LOG("   2 or more agents have the same start position.");
            impossible = true;
            break;
        }
    }
    for (int target : targets){
        if (std::count(targets.begin(), targets.end(), target)>1){
            LOG("   2 or more agents have the same target position.");
            impossible = true;
            break;
        }
    }
    if (!setOfHardVertexConstraints.empty()){
        LOG("The problem has the following hard vertex constraints :");
        for (const auto& constraint : setOfHardVertexConstraints){
            LOG("   Agent " << constraint.getAgent() << " cannot be at position " << constraint.getPosition() << " at time " << constraint.getTime() << ".");
        }
    }
    if (!setOfHardEdgeConstraints.empty()){
        LOG("The problem has the following hard edge constraints :");
        for (const auto& constraint : setOfHardEdgeConstraints){
            LOG("   Agent " << constraint.getAgent() << "cannot go from position " << constraint.getPosition1() << " to position " << constraint.getPosition2() << " between " << constraint.getTime()-1 << " and time "<< constraint.getTime() << ".");
        }
    }
    if (!setOfSoftVertexConstraints.empty()){
        LOG("The problem has the following soft vertex constraints :");
        for (const auto& constraint : setOfSoftVertexConstraints){
            LOG("   Agent " << constraint.getAgent() << " cannot be at position " << constraint.getPosition() << " at time " << constraint.getTime() << ".");
        }
    }
    if (!setOfSoftEdgeConstraints.empty()){
        LOG("The problem has the following soft edge constraints :");
        for (const auto& constraint : setOfSoftEdgeConstraints){
            LOG("   Agent " << constraint.getAgent() << "cannot go from position " << constraint.getPosition1() << " to position " << constraint.getPosition2() << " between " << constraint.getTime()-1 << " and time "<< constraint.getTime() << ".");
        }
    }
    if (maxCost!=INT_MAX){
        LOG("The solution of this problem must have a cost inferior or equal to " << maxCost);
    }
    LOG(" ");
}

std::shared_ptr<MultiAgentState> MultiAgentProblemWithConstraints::getStartState() const {
    return std::make_shared<MultiAgentState>(starts, starts, 0 , 0, true);
}

bool MultiAgentProblemWithConstraints::isGoalState(std::shared_ptr<MultiAgentState> state) const {
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

bool MultiAgentProblemWithConstraints::okForConstraints(int agent, int position, int newPosition, int time) const {
    if (setOfHardVertexConstraints.find({agentIds[agent], newPosition, time}) == setOfHardVertexConstraints.end()){
        return setOfHardEdgeConstraints.find({agentIds[agent], position, newPosition, time}) == setOfHardEdgeConstraints.end();
    }
    return false;
}

bool MultiAgentProblemWithConstraints::okForConstraints(int agent, int newPosition, int time) const {
    return setOfHardVertexConstraints.find({agentIds[agent], newPosition, time}) == setOfHardVertexConstraints.end();
}

int MultiAgentProblemWithConstraints::numberOfViolations(int agent, int position, int newPosition, int time) const {
    int count = 0;
    if (setOfSoftVertexConstraints.find({agentIds[agent], newPosition, time}) != setOfSoftVertexConstraints.end()){
        count += 1;
    }
    if (setOfSoftEdgeConstraints.find({agentIds[agent], position, newPosition, time}) != setOfSoftEdgeConstraints.end()){
        count += 1;
    }
    return count;
}

int MultiAgentProblemWithConstraints::numberOfViolations(int agent, int newPosition, int time) const {
    if (setOfSoftVertexConstraints.find({agentIds[agent], newPosition, time}) == setOfSoftVertexConstraints.end()){
        return 0;
    }
    return 1;
}

std::vector<std::tuple<std::shared_ptr<MultiAgentState>, int, int>> MultiAgentProblemWithConstraints::getSuccessors(std::shared_ptr<MultiAgentState> state) const {
    std::vector<std::tuple<std::shared_ptr<MultiAgentState>, int, int>> successors;
    auto positions = state->getPositions();
    auto prePositions = state->getPrePositions();
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
            if (notAlreadyOccupiedPosition(j, positions, agentToAssign) && notAlreadyOccupiedEdge(j, positions, agentToAssign, prePositions) && okForConstraints(agentToAssign, positions[agentToAssign], j, nextT)) {
                auto successor = std::make_shared<MultiAgentState>(newpositions, prePositions, nextT, nextAgentToAssign, isStandard);
                successors.emplace_back(successor, costMovement, numberOfViolations(agentToAssign, positions[agentToAssign], j, nextT));
            }
        }

        // Wait
        if (notAlreadyOccupiedPosition(positions[agentToAssign], positions, agentToAssign) && okForConstraints(agentToAssign, positions[agentToAssign], nextT)) {
            auto successor = std::make_shared<MultiAgentState>(positions, prePositions, nextT, nextAgentToAssign, isStandard);
            successors.emplace_back(successor, costWait, numberOfViolations(agentToAssign, positions[agentToAssign], nextT));
        }

    } else { // obj_function=="SumOfCosts"
        auto cannotMove = state->getCannotMove();
        if (state->canMove(agentToAssign)){ // agentToAssign is allowed to move

            // Move
            for (int j : graph->getNeighbors(positions[agentToAssign])) {
                vector<int> newpositions(positions);
                newpositions[agentToAssign] = j;
                if (notAlreadyOccupiedPosition(j, positions, agentToAssign) && notAlreadyOccupiedEdge(j, positions, agentToAssign, prePositions) && okForConstraints(agentToAssign, positions[agentToAssign], j, nextT)) {
                    auto successor = std::make_shared<MultiAgentState>(newpositions, prePositions, nextT, nextAgentToAssign, isStandard, cannotMove);
                    successors.emplace_back(successor, costMovement, numberOfViolations(agentToAssign, positions[agentToAssign], j, nextT));
                }
            }

            // Wait
            if (positions[agentToAssign]!=targets[agentToAssign]) { // agentToAssign not at his target position
                costWait = 1;
                if (notAlreadyOccupiedPosition(positions[agentToAssign], positions, agentToAssign) && okForConstraints(agentToAssign, positions[agentToAssign], nextT)) {
                    auto successor = std::make_shared<MultiAgentState>(positions, prePositions, nextT, nextAgentToAssign, isStandard, cannotMove);
                    successors.emplace_back(successor, costWait, numberOfViolations(agentToAssign, positions[agentToAssign], nextT));
                }
            } else { // agentToAssign is at his target position
                // agentToAssign can still move in the future
                costWait = 1;
                if (notAlreadyOccupiedPosition(positions[agentToAssign], positions, agentToAssign) && okForConstraints(agentToAssign, positions[agentToAssign], nextT)) {
                    auto successor = std::make_shared<MultiAgentState>(positions, prePositions, nextT, nextAgentToAssign, isStandard, cannotMove);
                    successors.emplace_back(successor, costWait, numberOfViolations(agentToAssign, positions[agentToAssign], nextT));
                }

                // we are forcing agentToAssign to not move in the future
                costWait = 0;
                cannotMove.push_back(agentToAssign);
                if (notAlreadyOccupiedPosition(positions[agentToAssign], positions, agentToAssign) && okForConstraints(agentToAssign, positions[agentToAssign], nextT)) {
                    auto successor = std::make_shared<MultiAgentState>(positions, prePositions, nextT, nextAgentToAssign, isStandard, cannotMove);
                    successors.emplace_back(successor, costWait, numberOfViolations(agentToAssign, positions[agentToAssign], nextT));
                }
            }


        } else { // agentToAssign is not allowed to move
            costWait = 0;
            if (notAlreadyOccupiedPosition(positions[agentToAssign], positions, agentToAssign) && okForConstraints(agentToAssign, positions[agentToAssign], nextT)) {
                auto successor = std::make_shared<MultiAgentState>(positions, prePositions, nextT, nextAgentToAssign, isStandard, cannotMove);
                successors.emplace_back(successor, costWait, numberOfViolations(agentToAssign, positions[agentToAssign], nextT));
            }
        }
    }
    return successors;
}

std::unordered_map<int, std::vector<int>> MultiAgentProblemWithConstraints::getPositions(std::vector<std::shared_ptr<MultiAgentState>> states) const {
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


const std::vector<int>& MultiAgentProblemWithConstraints::getStarts() const {
    return starts;
}

const std::vector<int>& MultiAgentProblemWithConstraints::getTargets() const {
    return targets;
}

ObjectiveFunction MultiAgentProblemWithConstraints::getObjFunction() {
    return objective;
}

std::set<VertexConstraint> MultiAgentProblemWithConstraints::getSetOfHardVertexConstraints() const {
    return setOfHardVertexConstraints;
}

std::set<EdgeConstraint> MultiAgentProblemWithConstraints::getSetOfHardEdgeConstraints() const {
    return setOfHardEdgeConstraints;
}

std::vector<int> MultiAgentProblemWithConstraints::getAgentIds() const {
    return agentIds;
}

int MultiAgentProblemWithConstraints::getStartOf(int id) {
    return starts[idToIndex[id]];
}

int MultiAgentProblemWithConstraints::getTargetOf(int id) {
    return targets[idToIndex[id]];
}

bool MultiAgentProblemWithConstraints::isImpossible() const {
    return impossible;
}

