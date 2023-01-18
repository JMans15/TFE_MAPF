//
// Created by Arthur Mahy on 18/01/2023.
//

#ifndef TFE_MAPF_MULTIAGENTSTATE_H
#define TFE_MAPF_MULTIAGENTSTATE_H
#include "State.h"

class MultiAgentState : public State {
public:

    MultiAgentState(vector<int> m_positions, int m_timestep, int m_agentToAssign, bool m_standard, vector<int> m_prePositions, const vector<int>& m_cannotMove = vector<int>());

    bool operator== (MultiAgentState other) const
    {
        if (agentToAssign!=other.agentToAssign){
            return false;
        }
        for (int i = 0; i < positions.size(); i++){
            if (positions[i]!=other.positions[i]){
                return false;
            }
        }
        /*if (timestep!=other.timestep){ // just needed because of the constraints (a, p, t)
            return false;
        }*/
        return true;
    }

    bool operator< (const MultiAgentState& other) const
    {
        if (agentToAssign!=other.agentToAssign){
            return agentToAssign<other.agentToAssign;
        }
        /*if (timestep!=other.timestep){ // just needed because of the constraints (a, p, t)
            return timestep<other.timestep;
        }*/
        return positions<other.positions;
    }

    vector<int> getPositions();
    vector<int> getPrePositions();
    int getAgentToAssign() const;
    bool isStandard() const;
    void makeStandard();
    vector<int> getCannotMove();
    bool canMove(int agent);

private:

    // The positions of every agent in this state.
    // positions[:agentToAssign] are the assigned positions.
    // positions[agentToAssign:] are the not yet assigned positions.
    // positions[agentToAssign] is not yet assigned but will be in the successor state.
    vector<int> positions;
    int agentToAssign;

    // A state is standard or regular when all agents have been assigned.
    // When a state is not standard, it is intermediate. (Operator Decomposition)
    bool standard;

    // prePositions is the positions of the last standard state.
    // positions and prePositions are equal at standard states (when all agents have been assigned).
    // prePositions is just needed to avoid Edge Conflict (implemented in the getSuccessors function).
    vector<int> prePositions;

    // list of agents which are at their target positions
    // and cannot move anymore (for the SumOfCosts objective function)
    vector<int> cannotMove;
};


#endif //TFE_MAPF_MULTIAGENTSTATE_H
