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

bool SingleAgentState::operator==(const State& other) const {
    auto o = dynamic_cast<const SingleAgentState*>(&other);
    if (position!=o->position){
        return false;
    }
    /*if (timestep!=other.timestep){ // just needed because of the constraints (a, p, t)
        return false;
    }*/
    return true;
}
