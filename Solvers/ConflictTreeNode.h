#include <utility>

//
// Created by Arthur Mahy on 09/11/2022.
//

#ifndef TFE_MAPF_CONFLICTTREENODE_H
#define TFE_MAPF_CONFLICTTREENODE_H

class ConflictTreeNode {
public:

    ConflictTreeNode(std::set<VertexConstraint> setOfVertexConstraints, std::set<EdgeConstraint> setOfEdgeConstraints, std::unordered_map<int, std::vector<int>> solution, const std::unordered_map<int,int>& costs, int cost, int numberOfConflicts = 0)
        : setOfVertexConstraints(std::move(setOfVertexConstraints))
        , setOfEdgeConstraints(std::move(setOfEdgeConstraints))
        , solution(std::move(solution))
        , costs(costs)
        , cost(cost)
        , numberOfConflicts(numberOfConflicts)
    {}
    ~ConflictTreeNode() = default;

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
        return numberOfConflicts;
    }

private:
    std::set<VertexConstraint> setOfVertexConstraints;
    std::set<EdgeConstraint> setOfEdgeConstraints;
    std::unordered_map<int, std::vector<int>> solution; // key = id of the agent, value = path of this agent
    std::unordered_map<int,int> costs; // key = id of the agent, value = cost of the path for this agent
    int cost; // cost of the current solution, the f-value of the node
    int numberOfConflicts; // number of soft constraints that have been violated on the path leading up to this node
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
