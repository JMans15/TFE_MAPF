//
// Created by Arthur Mahy on 09/11/2022.
//

#ifndef TFE_MAPF_STATE_H
#define TFE_MAPF_STATE_H

#include <cstddef>
#include <memory>

class State {
public:
    const virtual std::size_t getHash() const = 0;
};

template<class S>
struct StateHasher {
    std::size_t operator()(const std::shared_ptr<S> &state) const {
        return state->getHash();
    }
};

template<class S>
struct StateEquality {
    bool operator()(const std::shared_ptr<S> &a, const std::shared_ptr<S> &b) const {
        return a->isEqual(*b);
    }
};

#endif //TFE_MAPF_STATE_H
