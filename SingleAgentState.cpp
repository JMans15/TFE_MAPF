//
// Created by Arthur Mahy on 18/01/2023.
//

#include "SingleAgentState.h"

SingleAgentState::SingleAgentState(int m_position, int m_timestep) : State(m_timestep){
    position = m_position;
}

vector<int> SingleAgentState::getPositions() {
    vector<int> tab;
    tab.push_back(position);
    return tab;
}

int SingleAgentState::getPosition() const {
    return position;
}