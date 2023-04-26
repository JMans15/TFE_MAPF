#include <utility>

//
// Created by Arthur Mahy on 09/11/2022.
//

#ifndef TFE_MAPF_CONFLICTTREENODE_H
#define TFE_MAPF_CONFLICTTREENODE_H

#include "../ConflictConstraints/Conflict.h"

class ConflictTreeNode {
public:

    ConflictTreeNode(std::set<VertexConstraint> setOfVertexConstraints, std::set<EdgeConstraint> setOfEdgeConstraints, std::unordered_map<int, std::vector<int>> solution, std::unordered_map<int,int> costs, int cost, std::shared_ptr<ConflictTreeNode> parent = nullptr, std::set<Conflict> setOfConflicts = std::set<Conflict>())
        : setOfVertexConstraints(std::move(setOfVertexConstraints))
        , setOfEdgeConstraints(std::move(setOfEdgeConstraints))
        , solution(std::move(solution))
        , costs(std::move(costs))
        , cost(cost)
        , parent(std::move(parent))
        , setOfConflicts(setOfConflicts)
    {}
    ~ConflictTreeNode() = default;

    inline std::shared_ptr<ConflictTreeNode> getParent() const {
        return parent;
    }

    std::set<VertexConstraint> getSetOfVertexConstraints(){
        return setOfVertexConstraints;
    }

    std::set<EdgeConstraint> getSetOfEdgeConstraints(){
        return setOfEdgeConstraints;
    }

    std::unordered_map<int, std::vector<int>> getSolution() const {
        return solution;
    }

    std::unordered_map<int,int> getCosts() const {
        return costs;
    }

    inline int getCost() const {
        return cost;
    }

    inline int getNumberOfConflicts() const {
        return setOfConflicts.size();
    }

    std::set<Conflict> getSetOfConflicts() const {
        return setOfConflicts;
    }

private:
    std::shared_ptr<ConflictTreeNode> parent; // parent node
    std::set<VertexConstraint> setOfVertexConstraints; // constraint added in this node
    std::set<EdgeConstraint> setOfEdgeConstraints; // constraint added in this node

    // solution and costs only contains the path (and its path) of the agent that just has been replanned
    // but they contain the paths of all agents in the root node
    std::unordered_map<int, std::vector<int>> solution; // key = id of the agent, value = path of this agent
    std::unordered_map<int,int> costs; // key = id of the agent, value = cost of the path for this agent

    int cost; // cost of the current solution, the f-value of the node
    std::set<Conflict> setOfConflicts; // set of conflicts (between the paths) in the current solution
};

class ConflictTreeNodeComparator {
public:
    inline bool operator() (const std::shared_ptr<ConflictTreeNode> &a, const std::shared_ptr<ConflictTreeNode> &b) const {
        if (a->getCost() == b->getCost()) {
            return a->getNumberOfConflicts() < b->getNumberOfConflicts();
        }
        return a->getCost() < b->getCost();
    }
};

#endif //TFE_MAPF_CONFLICTTREENODE_H
