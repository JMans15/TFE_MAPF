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

    State(vector<int> m_positions, int m_timestep, int m_agentToAssignbool, bool m_standard, vector<int> m_prePositions);

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
    vector<int> getPrePositions();
    int getTimestep() const;
    int getAgentToAssign() const;
    bool isStandard() const;
    void makeStandard();

private:
    vector<int> positions;
    vector<int> prePositions; // positions and prePositions are equal at standard nodes (when all agents have been assigned)
    int timestep;
    int agentToAssign;
    bool standard;
};


#endif //TFE_MAPF_STATE_H
