//
// Created by Arthur Mahy on 11/06/2023.
//

#include "ODMultiAgentAStarProblem.h"

ODMultiAgentAStarProblem::ODMultiAgentAStarProblem(std::shared_ptr<MultiAgentProblem>  problem)
        : AStarProblem<ODMultiAgentState>()
        , problem(std::move(problem))
{}

std::shared_ptr<ODMultiAgentState> ODMultiAgentAStarProblem::getStartState() const {
    return std::make_shared<ODMultiAgentState>(problem->getStarts(), problem->getStarts(), 0, true);
}

bool ODMultiAgentAStarProblem::isGoalState(std::shared_ptr<ODMultiAgentState> state) const {
    auto positions = state->getPrePositions();
    for (int i = 0; i < problem->getNumberOfAgents(); i++) {
        if (positions[i] != problem->getTargets()[i]) {
            return false;
        }
    }
    return true;
}

// Returns true if position is not already occupied by assigned agents
bool ODMultiAgentAStarProblem::notAlreadyOccupiedPosition(int position, std::vector<int> &positions, int agentToAssign) const {
    for (int i = 0; i < agentToAssign; i++){
        if (positions[i] == position){
            return false;
        }
    }
    return true;
}

// Returns true if the edge (position, positions[agentToAssign]) is not already occupied by assigned agents
bool ODMultiAgentAStarProblem::notAlreadyOccupiedEdge(int position, const std::vector<int> &positions, int agentToAssign, const std::vector<int> &prePositions) const{
    for (int i = 0; i < agentToAssign; i++) {
        if (prePositions[i] == position && positions[i] == positions[agentToAssign]){
            return false;
        }
    }
    return true;
}

std::vector<std::tuple<std::shared_ptr<ODMultiAgentState>, int, int>> ODMultiAgentAStarProblem::getSuccessors(std::shared_ptr<ODMultiAgentState> state) const {
    std::vector<std::tuple<std::shared_ptr<ODMultiAgentState>, int, int>> successors;
    auto positions = state->getPositions();
    auto prePositions = state->getPrePositions();
    int agentToAssign = state->getAgentToAssign();
    int nextAgentToAssign;
    int costMovement;
    int costWait;
    bool isStandard;
    if (problem->getObjFunction() == Fuel){
        costMovement = 1;
        costWait = 0;
    } else if (problem->getObjFunction() == Makespan) {
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
    if (agentToAssign == problem->getNumberOfAgents()-1) {
        // we are assigning a position to the last agent,
        // the next state will be standard
        nextAgentToAssign = 0;
        isStandard = true;
    } else {
        nextAgentToAssign = agentToAssign+1;
        isStandard = false;
    }

    if (problem->getObjFunction() != SumOfCosts){

        // Move
        for (int j : problem->getGraph()->getNeighbors(positions[agentToAssign])) {
            vector<int> newpositions(positions);
            newpositions[agentToAssign] = j;
            if (notAlreadyOccupiedPosition(j, positions, agentToAssign) && notAlreadyOccupiedEdge(j, positions, agentToAssign, prePositions)) {
                auto successor = std::make_shared<ODMultiAgentState>(newpositions, prePositions, nextAgentToAssign, isStandard);
                successors.emplace_back(successor, costMovement, 0);
            }
        }

        // Wait
        if (notAlreadyOccupiedPosition(positions[agentToAssign], positions, agentToAssign)) {
            auto successor = std::make_shared<ODMultiAgentState>(positions, prePositions, nextAgentToAssign, isStandard);
            successors.emplace_back(successor, costWait, 0);
        }

    } else { // obj_function=="SumOfCosts"
        auto cannotMove = state->getCannotMove();
        if (state->canMove(agentToAssign)){ // agentToAssign is allowed to move

            // Move
            for (int j : problem->getGraph()->getNeighbors(positions[agentToAssign])) {
                vector<int> newpositions(positions);
                newpositions[agentToAssign] = j;
                if (notAlreadyOccupiedPosition(j, positions, agentToAssign) && notAlreadyOccupiedEdge(j, positions, agentToAssign, prePositions)) {
                    auto successor = std::make_shared<ODMultiAgentState>(newpositions, prePositions, nextAgentToAssign, isStandard, cannotMove);
                    successors.emplace_back(successor, costMovement, 0);
                }
            }

            // Wait
            if (positions[agentToAssign]!=problem->getTargets()[agentToAssign]) { // agentToAssign not at his target position
                costWait = 1;
                if (notAlreadyOccupiedPosition(positions[agentToAssign], positions, agentToAssign)) {
                    auto successor = std::make_shared<ODMultiAgentState>(positions, prePositions, nextAgentToAssign, isStandard, cannotMove);
                    successors.emplace_back(successor, costWait, 0);
                }
            } else { // agentToAssign is at his target position
                // agentToAssign can still move in the future
                costWait = 1;
                if (notAlreadyOccupiedPosition(positions[agentToAssign], positions, agentToAssign)) {
                    auto successor = std::make_shared<ODMultiAgentState>(positions, prePositions, nextAgentToAssign, isStandard, cannotMove);
                    successors.emplace_back(successor, costWait, 0);
                }

                // we are forcing agentToAssign to not move in the future
                costWait = 0;
                cannotMove[agentToAssign] = true;
                if (notAlreadyOccupiedPosition(positions[agentToAssign], positions, agentToAssign)) {
                    auto successor = std::make_shared<ODMultiAgentState>(positions, prePositions, nextAgentToAssign, isStandard, cannotMove);
                    successors.emplace_back(successor, costWait, 0);
                }
            }


        } else { // agentToAssign is not allowed to move
            costWait = 0;
            if (notAlreadyOccupiedPosition(positions[agentToAssign], positions, agentToAssign)) {
                auto successor = std::make_shared<ODMultiAgentState>(positions, prePositions, nextAgentToAssign, isStandard, cannotMove);
                successors.emplace_back(successor, costWait, 0);
            }
        }
    }
    return successors;
}

std::unordered_map<int, std::vector<int>> ODMultiAgentAStarProblem::getPositions(std::vector<std::shared_ptr<ODMultiAgentState>> states) const {
    std::unordered_map<int, std::vector<int>> positions;

    for (const auto& state : states) {
        if (state->isStandard()){
            for (int agent = 0; agent < problem->getNumberOfAgents(); agent++){
                positions[problem->getAgentIds()[agent]].push_back(state->getPositions()[agent]);
            }
        }
    }

    return positions;
}

int ODMultiAgentAStarProblem::getMaxCost() const {
    return problem->getMaxCost();
}

int ODMultiAgentAStarProblem::getStartTime() const {
    return problem->getStartTime();
}

std::shared_ptr<MultiAgentProblem> ODMultiAgentAStarProblem::getProblem() {
    return problem;
}
