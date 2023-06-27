//
// Created by Arthur Mahy on 23/05/2023.
//

#ifndef TFE_MAPF_STANDARDMULTIAGENTSPACETIMESTATE_H
#define TFE_MAPF_STANDARDMULTIAGENTSPACETIMESTATE_H

#include "StandardMultiAgentState.h"

class StandardMultiAgentSpaceTimeState : public StandardMultiAgentState {
public:

    StandardMultiAgentSpaceTimeState(std::vector<int> positions, int timestep, const std::vector<u_int8_t>& cannotMove = std::vector<u_int8_t>());

    std::size_t getHash() const override;
    bool isEqual(const StandardMultiAgentSpaceTimeState &other) const;

    int getTimestep() const;

private:
    // Number of timesteps since the agents started
    int timestep;
};


#endif //TFE_MAPF_STANDARDMULTIAGENTSPACETIMESTATE_H
