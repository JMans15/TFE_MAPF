//
// Created by Arthur Mahy on 18/01/2023.
//

#ifndef TFE_MAPF_MULTIAGENTSTATE_H
#define TFE_MAPF_MULTIAGENTSTATE_H

#include "State.h"

#include <vector>

class MultiAgentState : public State {
public:

    MultiAgentState(std::vector<int> positions, std::vector<int> prePositions, int timestep, int agentToAssign, bool standard, const std::vector<int>& cannotMove = std::vector<int>());
    
    const std::size_t getHash() const override;
    const bool isEqual(const MultiAgentState &other) const;

    const std::vector<int>& getPositions() const;
    const std::vector<int>& getPrePositions() const;
    int getAgentToAssign() const;
    int getTimestep() const;
    bool isStandard() const;
    void makeStandard();
    const std::vector<int>& getCannotMove() const;
    bool canMove(int agent);

private:

    // The positions of every agent in this state.
    // positions[:agentToAssign] are the assigned positions.
    // positions[agentToAssign:] are the not yet assigned positions.
    // positions[agentToAssign] is not yet assigned but will be in the successor state.
    std::vector<int> positions;
    int agentToAssign;

    // Number of timesteps since the agents started
    int timestep;

    // A state is standard or regular when all agents have been assigned.
    // When a state is not standard, it is intermediate. (Operator Decomposition)
    bool standard;

    // prePositions is the positions of the last standard state.
    // positions and prePositions are equal at standard states (when all agents have been assigned).
    // prePositions is just needed to avoid Edge Conflict (implemented in the getSuccessors function).
    std::vector<int> prePositions;

    // list of agents which are at their target positions
    // and cannot move anymore (for the SumOfCosts objective function)
    std::vector<int> cannotMove;
};


#endif //TFE_MAPF_MULTIAGENTSTATE_H
