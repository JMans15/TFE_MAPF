//
// Created by Arthur Mahy on 08/06/2023.
//

#ifndef TFE_MAPF_ODMULTIAGENTSTATE_H
#define TFE_MAPF_ODMULTIAGENTSTATE_H

#include "State.h"

class ODMultiAgentState : public State {
public:

    ODMultiAgentState(const std::vector<int>& positions, std::vector<int> prePositions, int agentToAssign, bool standard, const std::vector<u_int8_t>& cannotMove = std::vector<u_int8_t>());

    std::size_t getHash() const override;
    bool isEqual(const ODMultiAgentState &other) const;

    std::vector<int> getPositions() const;
    std::vector<int> getPrePositions() const;
    int getAgentToAssign() const;
    bool isStandard() const;
    std::vector<u_int8_t> getCannotMove() const;
    bool canMove(int agent) const;

protected:

    // The positions of every agent in this state.
    // positions[:agentToAssign] are the assigned positions.
    // positions[agentToAssign:] are the not yet assigned positions.
    // positions[agentToAssign] is not yet assigned but will be in the successor state.
    std::vector<int> positions;
    int agentToAssign;

    // A state is standard or regular when all agents have been assigned.
    // When a state is not standard, it is intermediate. (Operator Decomposition)
    bool standard;

    // prePositions is the positions of the last standard state.
    // positions and prePositions are equal at standard states (when all agents have been assigned).
    // prePositions is just needed to avoid Edge Conflict (implemented in the getSuccessors function).
    std::vector<int> prePositions;

    // vector of size numberOfAgents
    // cannotMove[i] is true if agent i is at its target position
    // and cannot move anymore (for the SumOfCosts objective function)
    std::vector<u_int8_t> cannotMove;
};


#endif //TFE_MAPF_ODMULTIAGENTSTATE_H
