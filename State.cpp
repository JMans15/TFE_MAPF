//
// Created by Arthur Mahy on 09/11/2022.
//

#include "State.h"

State::State(int m_timestep) {
    timestep = m_timestep;
}

int State::getTimestep() const {
    return timestep;
}

