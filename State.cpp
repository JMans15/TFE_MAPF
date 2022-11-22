//
// Created by Arthur Mahy on 09/11/2022.
//

#include "State.h"

#include <utility>

State::State(vector<int> m_positions, int m_timestep, int m_agentToAssign) {
    positions = std::move(m_positions);
    timestep = m_timestep;
    agentToAssign = m_agentToAssign;
}

vector<int> State::getPositions() {
    return positions;
}

int State::getTimestep() const {
    return timestep;
}

int State::getAgentToAssign() const {
    return agentToAssign;
}
