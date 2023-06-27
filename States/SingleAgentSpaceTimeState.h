//
// Created by Arthur Mahy on 13/02/2023.
//

#ifndef TFE_MAPF_SINGLEAGENTSPACETIMESTATE_H
#define TFE_MAPF_SINGLEAGENTSPACETIMESTATE_H

#include "State.h"

class SingleAgentSpaceTimeState : public State {
public:

    SingleAgentSpaceTimeState(int position, int timestep);
    ~SingleAgentSpaceTimeState() = default;
    
    std::size_t getHash() const override;

    bool isEqual(const SingleAgentSpaceTimeState &other) const;

    int getPosition() const;
    int getTimestep() const;

private:

    // Position of the agent in this state
    int position;

    // Number of timesteps since the agent started
    int timestep;

};


#endif //TFE_MAPF_SINGLEAGENTSPACETIMESTATE_H
