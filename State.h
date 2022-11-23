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
        if (agentToAssign!=other.agentToAssign){
            return false;
        }
        for (int i = 0; i < positions.size(); i++){
            if (positions[i]!=other.positions[i]){
                return false;
            }
        }
        return true;
    }

    bool operator< (const State& other) const
    {
        if (agentToAssign!=other.agentToAssign){
            return agentToAssign<other.agentToAssign;
        }
        return positions<other.positions;
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
