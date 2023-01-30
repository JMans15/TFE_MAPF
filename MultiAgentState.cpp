//
// Created by Arthur Mahy on 18/01/2023.
//

#include "MultiAgentState.h"
#include <algorithm>

MultiAgentState::MultiAgentState(vector<int> m_positions, int m_timestep, int m_agentToAssign, bool m_standard, vector<int> m_prePositions, const vector<int>& m_cannotMove) : State(m_timestep) {
    positions = std::move(m_positions);
    agentToAssign = m_agentToAssign;
    standard = m_standard;
    if (standard){
        prePositions = positions;
    } else {
        prePositions = std::move(m_prePositions);
    }
    cannotMove = m_cannotMove;
}

vector<int> MultiAgentState::getPositions() {
    return positions;
}

vector<int> MultiAgentState::getPrePositions() {
    return prePositions;
}

int MultiAgentState::getAgentToAssign() const {
    return agentToAssign;
}

bool MultiAgentState::isStandard() const {
    return standard;
}

void MultiAgentState::makeStandard() {
    standard = true;
}

vector<int> MultiAgentState::getCannotMove() {
    return cannotMove;
}

bool MultiAgentState::canMove(int agent) {
    return not (std::find(cannotMove.begin(), cannotMove.end(), agent) != cannotMove.end());
}