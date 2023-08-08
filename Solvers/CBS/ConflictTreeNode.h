//! %Node for a conflict tree
#include <memory>
#include <set>
#include <unordered_map>
#include <utility>
#include <vector>

//
// Created by Arthur Mahy on 09/11/2022.
//

#ifndef TFE_MAPF_CONFLICTTREENODE_H
#define TFE_MAPF_CONFLICTTREENODE_H

#include "../../Constraints/EdgeConstraint.h"
#include "../../Constraints/VertexConstraint.h"
#include "AgentConflict.h"

class ConflictTreeNode {
public:
  //! Constructor for a ConflictTreeNode
  ConflictTreeNode(
      std::shared_ptr<EdgeConstraint>
          edgeConstraint, //!< Edge constraint added by this node
      std::shared_ptr<VertexConstraint>
          vertexConstraint, //!< Vertex constraint added by this node
      std::unordered_map<int, std::vector<int>>
          solution, //!< Solutions for each agent of this node
      std::unordered_map<int, int> costs, //!< Costs for each agent
      int cost,                           //!< Total cost of this node
      std::shared_ptr<ConflictTreeNode> parent =
          nullptr, //!< Parent node of this node
      std::set<AgentConflict> setOfConflicts =
          std::set<AgentConflict>()) //!< Set of conflicts found in this node
      : edgeConstraint(std::move(edgeConstraint)),
        vertexConstraint(std::move(vertexConstraint)),
        solution(std::move(solution)), costs(std::move(costs)), cost(cost),
        parent(std::move(parent)), setOfConflicts(std::move(setOfConflicts)) {}
  ~ConflictTreeNode() = default;

  //! Getter for parent node
  inline std::shared_ptr<ConflictTreeNode> getParent() const { return parent; }

  //! Getter for edgeConstraint
  std::shared_ptr<EdgeConstraint> getEdgeConstraint() { return edgeConstraint; }

  //! Getter for vertexConstraint
  std::shared_ptr<VertexConstraint> getVertexConstraint() {
    return vertexConstraint;
  }

  //! Getter for solution
  std::unordered_map<int, std::vector<int>> getSolution() const {
    return solution;
  }

  //! Getter for costs
  std::unordered_map<int, int> getCosts() const { return costs; }

  //! Getter for cost
  inline int getCost() const { return cost; }

  //! Get the number of conflicts
  inline int getNumberOfConflicts() const { return setOfConflicts.size(); }

  //! Getter for The set of conflicts
  std::set<AgentConflict> getSetOfConflicts() const { return setOfConflicts; }

private:
  std::shared_ptr<EdgeConstraint>
      edgeConstraint; //!< Edge constraint added in this node (can be nullptr)
  std::shared_ptr<VertexConstraint>
      vertexConstraint; //!< Vertex  constrain added in this node (can be
                        //!< nullptr)
  std::unordered_map<int, std::vector<int>>
      solution; //!< key = id of the agent, value = path of this agent
  std::unordered_map<int, int>
      costs; //!< key = id of the agent, value = cost of the path for this agent
  int cost;  //!< cost of the current solution, the f-value of the node
  std::shared_ptr<ConflictTreeNode> parent; //!< Parent node

  // solution and costs only contains the path (and its path) of the agent that
  // just has been replanned but they contain the paths of all agents in the
  // root node

  std::set<AgentConflict> setOfConflicts; //!< set of conflicts (between the
                                          //!< paths) in the current solution
};

//! Class implementing a comparator for ConflictTreeNode
class ConflictTreeNodeComparator {
public:
  //! Defines comparison between ConflictTreeNode instances as comparison
  //! between their costs, then their number of conflicts
  inline bool operator()(const std::shared_ptr<ConflictTreeNode> &a,
                         const std::shared_ptr<ConflictTreeNode> &b) const {
    if (a->getCost() == b->getCost()) {
      return a->getNumberOfConflicts() < b->getNumberOfConflicts();
    }
    return a->getCost() < b->getCost();
  }
};

#endif // TFE_MAPF_CONFLICTTREENODE_H
