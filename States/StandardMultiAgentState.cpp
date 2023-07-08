//
// Created by Arthur Mahy on 11/06/2023.
//

#include "StandardMultiAgentState.h"

#include <boost/functional/hash.hpp>
#include <utility>

StandardMultiAgentState::StandardMultiAgentState(
    const std::vector<int> &positions, const std::vector<u_int8_t> &cannotMove_)
    : positions(positions), cannotMove(cannotMove_) {
  if (cannotMove_.empty()) {
    cannotMove = std::vector<u_int8_t>(positions.size(), false);
  }
}

std::vector<int> StandardMultiAgentState::getPositions() const {
  return positions;
}

std::vector<u_int8_t> StandardMultiAgentState::getCannotMove() const {
  return cannotMove;
}

std::size_t StandardMultiAgentState::getHash() const {
  size_t result = 0;
  for (const auto &val : positions) {
    boost::hash_combine(result, val);
  }
  for (const auto &val : cannotMove) {
    boost::hash_combine(result, val);
  }
  return result;
}

bool StandardMultiAgentState::isEqual(
    const StandardMultiAgentState &other) const {
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
