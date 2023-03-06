//
// Created by Arthur Mahy on 18/01/2023.
//

#include "MultiAgentState.h"

#include <boost/functional/hash.hpp>
#include <utility>

MultiAgentState::MultiAgentState(std::vector<int> positions, std::vector<int> prePositions_, int timestep, int agentToAssign, bool standard, const std::vector<int>& cannotMove)
    : positions(positions)
    , timestep(timestep)
    , agentToAssign(agentToAssign)
    , standard(standard)
    , cannotMove(cannotMove)
{
    if (standard) {
        prePositions = positions;
    } else {
        prePositions = std::move(prePositions_);
    }
}

const std::vector<int>& MultiAgentState::getPositions() const {
    return positions;
}

const std::vector<int>& MultiAgentState::getPrePositions() const {
    return prePositions;
}

int MultiAgentState::getAgentToAssign() const {
    return agentToAssign;
}

int MultiAgentState::getTimestep() const {
    return timestep;
}

bool MultiAgentState::isStandard() const {
    return standard;
}

void MultiAgentState::makeStandard() {
    standard = true;
}

const std::vector<int>& MultiAgentState::getCannotMove() const {
    return cannotMove;
}

bool MultiAgentState::canMove(int agent) {
    return not (std::find(cannotMove.begin(), cannotMove.end(), agent) != cannotMove.end());
}

const std::size_t MultiAgentState::getHash() const {
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

const bool MultiAgentState::isEqual(const MultiAgentState &other) const {
    for (int i = 0; i < prePositions.size(); i++) {
        if (prePositions[i] != other.prePositions[i] || positions[i] != other.positions[i]) {
            return false;
        }
    }

    if (cannotMove.size() != other.cannotMove.size()) {
        return false;
    }

    for (int i = 0; i < cannotMove.size(); i++) {
        if (cannotMove[i] != other.cannotMove[i]) {
            return false;
        }
    }

    return true;
}