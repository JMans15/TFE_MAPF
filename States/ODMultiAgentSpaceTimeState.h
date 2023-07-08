//
// Created by Arthur Mahy on 18/01/2023.
//

#ifndef TFE_MAPF_ODMULTIAGENTSPACETIMESTATE_H
#define TFE_MAPF_ODMULTIAGENTSPACETIMESTATE_H

#include "ODMultiAgentState.h"

class ODMultiAgentSpaceTimeState : public ODMultiAgentState {
public:
  ODMultiAgentSpaceTimeState(
      const std::vector<int> &positions, std::vector<int> prePositions,
      int timestep, int agentToAssign, bool standard,
      const std::vector<u_int8_t> &cannotMove = std::vector<u_int8_t>());

  std::size_t getHash() const override;
  bool isEqual(const ODMultiAgentSpaceTimeState &other) const;

  int getTimestep() const;

private:
  // Number of timesteps since the agents started
  int timestep;
};

#endif // TFE_MAPF_ODMULTIAGENTSPACETIMESTATE_H
