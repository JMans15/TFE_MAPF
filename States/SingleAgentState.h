//
// Created by Arthur Mahy on 18/01/2023.
//

#ifndef TFE_MAPF_SINGLEAGENTSTATE_H
#define TFE_MAPF_SINGLEAGENTSTATE_H

#include "State.h"

class SingleAgentState : public State {
public:

    SingleAgentState(int position) : position(position) {}
    ~SingleAgentState() {}

    inline const std::size_t getHash() const {
        return position;
    }

    inline const bool isEqual(const SingleAgentState &other) const {
        return position == other.position;
    }

    inline const int getPosition() const {
        return position;
    }

    const std::vector<int> getPositions() const {
        return { position };
    }

private:

    // Position of the agent in this state
    int position;

};


#endif //TFE_MAPF_SINGLEAGENTSTATE_H
