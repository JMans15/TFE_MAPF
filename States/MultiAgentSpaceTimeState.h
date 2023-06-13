//
// Created by Arthur Mahy on 18/01/2023.
//

#ifndef TFE_MAPF_MULTIAGENTSPACETIMESTATE_H
#define TFE_MAPF_MULTIAGENTSPACETIMESTATE_H

#include "MultiAgentState.h"

class MultiAgentSpaceTimeState : public MultiAgentState {
public:

    MultiAgentSpaceTimeState(std::vector<int> positions, std::vector<int> prePositions, int timestep, int agentToAssign, bool standard, const std::vector<int>& cannotMove = std::vector<int>());
    
    const std::size_t getHash() const override;
    const bool isEqual(const MultiAgentSpaceTimeState &other) const;

    int getTimestep() const;

private:
    // Number of timesteps since the agents started
    int timestep;

};


#endif //TFE_MAPF_MULTIAGENTSPACETIMESTATE_H
