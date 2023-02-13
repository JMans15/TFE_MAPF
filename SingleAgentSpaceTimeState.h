//
// Created by Arthur Mahy on 13/02/2023.
//

#ifndef TFE_MAPF_SINGLEAGENTSPACETIMESTATE_H
#define TFE_MAPF_SINGLEAGENTSPACETIMESTATE_H

#include "SingleAgentState.h"

class SingleAgentSpaceTimeState : public SingleAgentState {
public:

    SingleAgentSpaceTimeState(int m_position, int m_timestep);

    bool operator==(const State& other) const;

    bool operator< (const SingleAgentSpaceTimeState& other) const
    {
        if (timestep!=other.timestep){
            return timestep<other.timestep;
        }
        return position<other.position;
    }
};


#endif //TFE_MAPF_SINGLEAGENTSPACETIMESTATE_H
