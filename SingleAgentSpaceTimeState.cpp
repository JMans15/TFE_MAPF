//
// Created by Arthur Mahy on 13/02/2023.
//

#include "SingleAgentSpaceTimeState.h"

SingleAgentSpaceTimeState::SingleAgentSpaceTimeState(int m_position, int m_timestep) : SingleAgentState(m_position, m_timestep){
}

bool SingleAgentSpaceTimeState::operator==(const State &other) const {
    auto o = dynamic_cast<const SingleAgentSpaceTimeState*>(&other);
    if (position!=o->position){
        return false;
    }
    if (timestep!=o->timestep){ // just needed because of the constraints (a, p, t)
        return false;
    }
    return true;
}

size_t SingleAgentSpaceTimeState::hash() const {
    size_t result = 0;
    boost::hash_combine(result, position);
    boost::hash_combine(result, timestep);
    return result;
}
