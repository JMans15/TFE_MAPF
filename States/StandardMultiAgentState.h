//
// Created by Arthur Mahy on 11/06/2023.
//

#ifndef TFE_MAPF_STANDARDMULTIAGENTSTATE_H
#define TFE_MAPF_STANDARDMULTIAGENTSTATE_H

#include "State.h"

class StandardMultiAgentState : public State {
public:

    StandardMultiAgentState(std::vector<int> positions, const std::vector<int>& cannotMove = std::vector<int>());

    const std::size_t getHash() const override;
    const bool isEqual(const StandardMultiAgentState &other) const;

    const std::vector<int>& getPositions() const;
    const std::vector<int>& getCannotMove() const;
    bool canMove(int agent);

protected:

    // The positions of every agent in this state.
    std::vector<int> positions;

    // list of agents which are at their target positions
    // and cannot move anymore (for the SumOfCosts objective function)
    std::vector<int> cannotMove;
};


#endif //TFE_MAPF_STANDARDMULTIAGENTSTATE_H
