//
// Created by Arthur Mahy on 18/01/2023.
//

#ifndef TFE_MAPF_SINGLEAGENTSTATE_H
#define TFE_MAPF_SINGLEAGENTSTATE_H

#include "State.h"

class SingleAgentState : public State {
public:

    explicit SingleAgentState(int position) : position(position) {}
    ~SingleAgentState() = default;

    inline std::size_t getHash() const override {
        return position;
    }

    inline bool isEqual(const SingleAgentState &other) const {
        return position == other.position;
    }

    inline int getPosition() const {
        return position;
    }

private:

    // Position of the agent in this state
    int position;

};


#endif //TFE_MAPF_SINGLEAGENTSTATE_H
