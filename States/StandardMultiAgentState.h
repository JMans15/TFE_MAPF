//
// Created by Arthur Mahy on 11/06/2023.
//

#ifndef TFE_MAPF_STANDARDMULTIAGENTSTATE_H
#define TFE_MAPF_STANDARDMULTIAGENTSTATE_H

#include "State.h"

class StandardMultiAgentState : public State {
public:

    explicit StandardMultiAgentState(const std::vector<int>& positions, const std::vector<u_int8_t>& cannotMove = std::vector<u_int8_t>());

    std::size_t getHash() const override;
    bool isEqual(const StandardMultiAgentState &other) const;

    std::vector<int> getPositions() const;
    std::vector<u_int8_t> getCannotMove() const;

protected:

    // The positions of every agent in this state.
    std::vector<int> positions;

    // vector of size numberOfAgents
    // cannotMove[i] is true if agent i is at its target position
    // and cannot move anymore (for the SumOfCosts objective function)
    std::vector<u_int8_t> cannotMove;
};


#endif //TFE_MAPF_STANDARDMULTIAGENTSTATE_H
