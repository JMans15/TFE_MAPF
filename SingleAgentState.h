//
// Created by Arthur Mahy on 18/01/2023.
//

#ifndef TFE_MAPF_SINGLEAGENTSTATE_H
#define TFE_MAPF_SINGLEAGENTSTATE_H
#include "State.h"

class SingleAgentState : public State {
public:

    SingleAgentState(int m_position, int m_timestep);

    bool operator== (const SingleAgentState& other) const
    {
        if (position!=other.position){
            return false;
        }
        /*if (timestep!=other.timestep){ // just needed because of the constraints (a, p, t)
            return false;
        }*/
        return true;
    }

    bool operator< (const SingleAgentState& other) const
    {
        /*if (timestep!=other.timestep){ // just needed because of the constraints (a, p, t)
            return timestep<other.timestep;
        }*/
        return position<other.position;
    }

    vector<int> getPositions();
    int getPosition() const;

private:

    // Position of the agent in this state
    int position;

};


#endif //TFE_MAPF_SINGLEAGENTSTATE_H
