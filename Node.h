//
// Created by Arthur Mahy on 09/11/2022.
//

#ifndef TFE_MAPF_NODE_H
#define TFE_MAPF_NODE_H

#include <memory>

template <class S>
class Node {
public:

    Node(std::shared_ptr<S> state, int cost, int heuristic, std::shared_ptr<Node<S>> parent = nullptr)
        : state(state)
        , cost(cost)
        , heuristic(heuristic)
        , parent(parent)
    {}
    ~Node() {}

    inline std::shared_ptr<S> getState() const {
        return state;
    }

    inline std::shared_ptr<Node<S>> getParent() const {
        return parent;
    }

    inline int getCost() const {
        return cost;
    }

    inline int getHeuristic() const {
        return heuristic;
    }

private:
    std::shared_ptr<S> state; // state of the node
    std::shared_ptr<Node<S>> parent; // parent node
    int cost; // path cost
    int heuristic; // heuristic value
};

template <class S>
class NodeComparator {
public:
    inline bool operator() (const std::shared_ptr<Node<S>> &a, const std::shared_ptr<Node<S>> &b) const {
        if (a->getCost() + a->getHeuristic() == b->getCost() + b->getHeuristic()) {
            if (a->getCost() == b->getCost()) {
                return a->getState()->getHash() < b->getState()->getHash();
            }
            return a->getCost() > b->getCost();
        }
        return a->getCost() + a->getHeuristic() < b->getCost() + b->getHeuristic();
    }
};

#endif //TFE_MAPF_NODE_H
