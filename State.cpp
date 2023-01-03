//
// Created by Arthur Mahy on 09/11/2022.
//

#include "State.h"
#include <utility>

State::State(vector<int> m_positions, int m_timestep, int m_agentToAssign, bool m_standard, vector<int> m_prePositions, const vector<int>& m_cannotMove) {
    positions = std::move(m_positions);
    timestep = m_timestep;
    agentToAssign = m_agentToAssign;
    standard = m_standard;
    if (standard){
        prePositions = positions;
    } else {
        prePositions = std::move(m_prePositions);
    }
    cannotMove = m_cannotMove;
}

vector<int> State::getPositions() {
    return positions;
}

vector<int> State::getPrePositions() {
    return prePositions;
}

int State::getTimestep() const {
    return timestep;
}

int State::getAgentToAssign() const {
    return agentToAssign;
}

bool State::isStandard() const {
    return standard;
}

void State::makeStandard() {
    standard = true;
}

vector<int> State::getCannotMove() {
    return cannotMove;
}

bool State::canMove(int agent) {
    return not (std::find(cannotMove.begin(), cannotMove.end(), agent) != cannotMove.end());
}
