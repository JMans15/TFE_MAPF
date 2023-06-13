//
// Created by Arthur Mahy on 18/01/2023.
//

#include "MultiAgentSpaceTimeState.h"

#include <boost/functional/hash.hpp>
#include <utility>

MultiAgentSpaceTimeState::MultiAgentSpaceTimeState(std::vector<int> positions, std::vector<int> prePositions_, int timestep, int agentToAssign, bool standard, const std::vector<int>& cannotMove)
    : MultiAgentState(std::move(positions), std::move(prePositions_), agentToAssign, standard, cannotMove)
    , timestep(timestep)
{}

const std::size_t MultiAgentSpaceTimeState::getHash() const {
    size_t result = 0;
    boost::hash_combine(result, timestep);
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

const bool MultiAgentSpaceTimeState::isEqual(const MultiAgentSpaceTimeState &other) const {

    if (timestep!=other.timestep){
        return false;
    }

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

int MultiAgentSpaceTimeState::getTimestep() const {
    return timestep;
}