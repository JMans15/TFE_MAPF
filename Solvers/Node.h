//
// Created by Arthur Mahy on 09/11/2022.
//

#ifndef TFE_MAPF_NODE_H
#define TFE_MAPF_NODE_H

#include <memory>

template <class S>
class Node {
public:

    Node(std::shared_ptr<S> state, int cost, int heuristic, std::shared_ptr<Node<S>> parent = nullptr, int violationCount = 0)
        : state(state)
        , cost(cost)
        , heuristic(heuristic)
        , parent(parent)
        , violationCount(violationCount)
    {}
    ~Node() = default;

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

    inline int getViolationCount() const {
        return violationCount;
    }

private:
    std::shared_ptr<S> state; // state of the node
    std::shared_ptr<Node<S>> parent; // parent node
    int cost; // path cost
    int heuristic; // heuristic value
    int violationCount; // number of soft constraints that have been violated on the path leading up to this node
};

template <class S>
class NodeComparator {
public:
    inline bool operator() (const std::shared_ptr<Node<S>> &a, const std::shared_ptr<Node<S>> &b) const {
        if (a->getCost() + a->getHeuristic() == b->getCost() + b->getHeuristic()) {
            if (a->getViolationCount() == b->getViolationCount()){
                return a->getHeuristic() < b->getHeuristic();
            }
            return a->getViolationCount() < b->getViolationCount();
        }
        return a->getCost() + a->getHeuristic() < b->getCost() + b->getHeuristic();
    }
};

#endif //TFE_MAPF_NODE_H
