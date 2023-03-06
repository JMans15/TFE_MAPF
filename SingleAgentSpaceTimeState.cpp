//
// Created by Arthur Mahy on 13/02/2023.
//

#include "SingleAgentSpaceTimeState.h"

#include <boost/functional/hash.hpp>

SingleAgentSpaceTimeState::SingleAgentSpaceTimeState(int position, int timestep)
    : position(position)
    , timestep(timestep)
{}

SingleAgentSpaceTimeState::~SingleAgentSpaceTimeState() {}

inline const std::size_t SingleAgentSpaceTimeState::getHash() const {
    size_t result = 0;
    boost::hash_combine(result, position);
    boost::hash_combine(result, timestep);
    return result;
}

const bool SingleAgentSpaceTimeState::isEqual(const SingleAgentSpaceTimeState &other) const{
    return position == other.position && timestep == other.timestep;
}

const int SingleAgentSpaceTimeState::getPosition() const {
    return position;
}

const int SingleAgentSpaceTimeState::getTimestep() const {
    return timestep;
}

const std::vector<int> SingleAgentSpaceTimeState::getPositions() const {
    return { position };
}
