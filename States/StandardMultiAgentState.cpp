//
// Created by Arthur Mahy on 23/05/2023.
//

#include "StandardMultiAgentState.h"

#include <boost/functional/hash.hpp>
#include <utility>

StandardMultiAgentState::StandardMultiAgentState(std::vector<int> positions, int timestep, const std::vector<int>& cannotMove)
        : positions(positions)
        , timestep(timestep)
        , cannotMove(cannotMove)
{}

const std::vector<int>& StandardMultiAgentState::getPositions() const {
    return positions;
}

int StandardMultiAgentState::getTimestep() const {
    return timestep;
}

const std::vector<int>& StandardMultiAgentState::getCannotMove() const {
    return cannotMove;
}

bool StandardMultiAgentState::canMove(int agent) {
    return not (std::find(cannotMove.begin(), cannotMove.end(), agent) != cannotMove.end());
}

const std::size_t StandardMultiAgentState::getHash() const {
    size_t result = 0;
    for (const auto& val : positions) {
        boost::hash_combine(result, val);
    }
    for (const auto& val : cannotMove) {
        boost::hash_combine(result, val);
    }
    return result;
}

const bool StandardMultiAgentState::isEqual(const StandardMultiAgentState &other) const {
    for (int i = 0; i < positions.size(); i++) {
        if (positions[i] != other.positions[i]) {
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