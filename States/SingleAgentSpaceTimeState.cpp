//
// Created by Arthur Mahy on 13/02/2023.
//

#include "SingleAgentSpaceTimeState.h"

#include <boost/functional/hash.hpp>

SingleAgentSpaceTimeState::SingleAgentSpaceTimeState(int position, int timestep)
    : position(position), timestep(timestep) {}

inline std::size_t SingleAgentSpaceTimeState::getHash() const {
  size_t result = 0;
  boost::hash_combine(result, position);
  boost::hash_combine(result, timestep);
  return result;
}

bool SingleAgentSpaceTimeState::isEqual(
    const SingleAgentSpaceTimeState &other) const {
  return position == other.position && timestep == other.timestep;
}

int SingleAgentSpaceTimeState::getPosition() const { return position; }

int SingleAgentSpaceTimeState::getTimestep() const { return timestep; }
