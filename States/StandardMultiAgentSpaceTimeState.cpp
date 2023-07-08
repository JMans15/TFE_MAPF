//
// Created by Arthur Mahy on 23/05/2023.
//

#include "StandardMultiAgentSpaceTimeState.h"
#include <sys/types.h>

#include <boost/functional/hash.hpp>
#include <utility>

StandardMultiAgentSpaceTimeState::StandardMultiAgentSpaceTimeState(
    std::vector<int> positions, int timestep,
    const std::vector<u_int8_t> &cannotMove)
    : StandardMultiAgentState(std::move(positions), cannotMove),
      timestep(timestep) {}

int StandardMultiAgentSpaceTimeState::getTimestep() const { return timestep; }

std::size_t StandardMultiAgentSpaceTimeState::getHash() const {
  size_t result = 0;
  boost::hash_combine(result, timestep);
  for (const auto &val : positions) {
    boost::hash_combine(result, val);
  }
  for (const auto &val : cannotMove) {
    boost::hash_combine(result, val);
  }
  return result;
}

bool StandardMultiAgentSpaceTimeState::isEqual(
    const StandardMultiAgentSpaceTimeState &other) const {

  if (timestep != other.timestep) {
    return false;
  }

  for (int i = 0; i < positions.size(); i++) {
    if (positions[i] != other.positions[i]) {
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
