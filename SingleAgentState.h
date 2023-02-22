//
// Created by Arthur Mahy on 18/01/2023.
//

#ifndef TFE_MAPF_SINGLEAGENTSTATE_H
#define TFE_MAPF_SINGLEAGENTSTATE_H
#include "State.h"

class SingleAgentState : public State {
public:

    SingleAgentState(int m_position, int m_timestep);

    bool operator==(const State& other) const;

    bool operator< (const SingleAgentState& other) const{
        return position<other.position;
    }

    vector<int> getPositions();
    int getPosition() const;
    size_t hash() const override;

protected:

    // Position of the agent in this state
    int position;

};


#endif //TFE_MAPF_SINGLEAGENTSTATE_H
