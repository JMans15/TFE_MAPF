//
// Created by Arthur Mahy on 08/06/2023.
//

#include "ODMultiAgentState.h"

#include <boost/functional/hash.hpp>
#include <utility>

ODMultiAgentState::ODMultiAgentState(const std::vector<int> &positions, std::vector<int> prePositions_,
                                                     int agentToAssign, bool standard,
                                                     const std::vector<u_int8_t>& cannotMove_)
        : positions(positions)
        , agentToAssign(agentToAssign)
        , standard(standard)
        , cannotMove(cannotMove_)
{
    if (standard) {
        prePositions = positions;
    } else {
        prePositions = std::move(prePositions_);
    }
    if (cannotMove_.empty()){
        cannotMove = std::vector<u_int8_t>(positions.size(), false);
    }
}

std::vector<int> ODMultiAgentState::getPositions() const {
    return positions;
}

std::vector<int> ODMultiAgentState::getPrePositions() const {
    return prePositions;
}

int ODMultiAgentState::getAgentToAssign() const {
    return agentToAssign;
}

bool ODMultiAgentState::isStandard() const {
    return standard;
}

std::vector<u_int8_t> ODMultiAgentState::getCannotMove() const {
    return cannotMove;
}

bool ODMultiAgentState::canMove(int agent) const {
    return not cannotMove[agent];
}

std::size_t ODMultiAgentState::getHash() const {
    size_t result = 0;
    for (const auto& val : prePositions) {
        boost::hash_combine(result, val);
    }
    for (const auto& val : positions) {
        boost::hash_combine(result, val);
    }
    for (const auto& val : cannotMove) {
        boost::hash_combine(result, val);
    }
    boost::hash_combine(result, agentToAssign);
    return result;
}

bool ODMultiAgentState::isEqual(const ODMultiAgentState &other) const {
    for (int i = 0; i < prePositions.size(); i++) {
        if (prePositions[i] != other.prePositions[i] || positions[i] != other.positions[i]) {
            return false;
        }
    }

    for (int i = 0; i < cannotMove.size(); i++) {
        if (cannotMove[i] != other.cannotMove[i]) {
            return false;
        }
    }

    return true;
}