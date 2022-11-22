//
// Created by Arthur Mahy on 09/11/2022.
//

#ifndef TFE_MAPF_STATE_H
#define TFE_MAPF_STATE_H

#include <utility>
#include <vector>
using namespace std;

class State {

public:

    State(vector<int> m_positions, int m_timestep, int m_agentToAssign);
    bool operator== (State other) const
    {
        for (int i = 0; i < positions.size(); i++){
            if (positions[i]!=other.positions[i]){
                return false;
            }
        }
        if (agentToAssign!=other.agentToAssign){
            return false;
        }
        return true;
    }

    bool operator< (State other) const
    {
        return not (*this==std::move(other));
    }
    vector<int> getPositions();
    int getTimestep() const;
    int getAgentToAssign() const;

private:
    vector<int> positions;
    int timestep;
    int agentToAssign;
};


#endif //TFE_MAPF_STATE_H
