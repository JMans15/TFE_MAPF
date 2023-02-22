//
// Created by Arthur Mahy on 18/01/2023.
//

#include "MultiAgentState.h"
#include <algorithm>
#include <boost/functional/hash.hpp>

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

bool MultiAgentState::operator==(const State& other) const {
    auto o = dynamic_cast<const MultiAgentState*>(&other);
    if (agentToAssign!=o->agentToAssign){
        return false;
    }
    for (int i = 0; i < positions.size(); i++){
        if (positions[i]!=o->positions[i]){
            return false;
        }
    }
    /*if (timestep!=other.timestep){ // just needed because of the constraints (a, p, t)
        return false;
    }*/
    return true;
}

size_t MultiAgentState::hash() const {
    size_t result = 0;
    for (const auto& val : positions) {
        boost::hash_combine(result, val);
    }
    boost::hash_combine(result, agentToAssign);
    return result;
}
