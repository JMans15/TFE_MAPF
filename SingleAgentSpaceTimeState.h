//
// Created by Arthur Mahy on 13/02/2023.
//

#ifndef TFE_MAPF_SINGLEAGENTSPACETIMESTATE_H
#define TFE_MAPF_SINGLEAGENTSPACETIMESTATE_H

#include "SingleAgentState.h"

class SingleAgentSpaceTimeState : public SingleAgentState {
public:

    SingleAgentSpaceTimeState(int position, int timestep);
    ~SingleAgentSpaceTimeState();
    
    const std::size_t getHash() const override;

    const bool isEqual(const SingleAgentSpaceTimeState &other) const;

    const int getTimestep() const;

private:

    // Number of timesteps since the agent started
    int timestep;

};


#endif //TFE_MAPF_SINGLEAGENTSPACETIMESTATE_H
