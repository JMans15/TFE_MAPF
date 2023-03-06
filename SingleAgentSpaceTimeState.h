//
// Created by Arthur Mahy on 13/02/2023.
//

#ifndef TFE_MAPF_SINGLEAGENTSPACETIMESTATE_H
#define TFE_MAPF_SINGLEAGENTSPACETIMESTATE_H

#include "State.h"

#include <vector>

class SingleAgentSpaceTimeState : public State {
public:

    SingleAgentSpaceTimeState(int position, int timestep);
    ~SingleAgentSpaceTimeState();
    
    const std::size_t getHash() const;

    const bool isEqual(const SingleAgentSpaceTimeState &other) const;

    const int getPosition() const;
    const int getTimestep() const;

    const std::vector<int> getPositions() const;

private:

    // Position of the agent in this state
    int position;

    // Number of timesteps since the agent started
    int timestep;

};


#endif //TFE_MAPF_SINGLEAGENTSPACETIMESTATE_H
