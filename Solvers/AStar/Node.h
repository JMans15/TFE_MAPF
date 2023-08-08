//! Node structure used in A*

#ifndef TFE_MAPF_NODE_H
#define TFE_MAPF_NODE_H

#include <memory>

template <class S> class Node {
public:
  //! Constructor for Node
  Node(std::shared_ptr<S>
           state,     //!< Which definition of State to use, see \ref States
       int cost,      //!< Cost of the path
       int heuristic, //!< Value of the heuristic for the node
       std::shared_ptr<Node<S>> parent =
           nullptr, //!< Parent Node (can be nullptr)
       int violationCount =
           0) //!< Number of violated soft constraints (tie-breaking heuristic)
      : state(state), cost(cost), heuristic(heuristic), parent(parent),
        violationCount(violationCount) {}
  ~Node() = default;

  //! Getter for state
  inline std::shared_ptr<S> getState() const { return state; }

  //! Getter for parent Node
  inline std::shared_ptr<Node<S>> getParent() const { return parent; }

  //! Getter for  cost
  inline int getCost() const { return cost; }

  //! Getter for heuristic value
  inline int getHeuristic() const { return heuristic; }

  //! Getter for number of violated contraints
  inline int getViolationCount() const { return violationCount; }

private:
  std::shared_ptr<S> state;        //!< state of the node
  int cost;                        //<! path cost
  int heuristic;                   //<! heuristic value
  std::shared_ptr<Node<S>> parent; //<! parent node
  int violationCount; //<! number of soft constraints that have been violated on
                      //<! the path leading up to this node
};

//! Node comparator used in the open list/frontier of A*
//! Comparison value defined as cost + heuristic, then number of violated soft
//! constraints, then heuristic alone
template <class S> class NodeComparator {
public:
  inline bool operator()(const std::shared_ptr<Node<S>> &a,
                         const std::shared_ptr<Node<S>> &b) const {
    if (a->getCost() + a->getHeuristic() == b->getCost() + b->getHeuristic()) {
      if (a->getViolationCount() == b->getViolationCount()) {
        return a->getHeuristic() < b->getHeuristic();
      }
      return a->getViolationCount() < b->getViolationCount();
    }
    return a->getCost() + a->getHeuristic() < b->getCost() + b->getHeuristic();
  }
};

#endif // TFE_MAPF_NODE_H
