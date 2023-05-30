//
// Created by Arthur Mahy on 10/04/2023.
//

#ifndef TFE_MAPF_CONFLICTBASEDSEARCH_H
#define TFE_MAPF_CONFLICTBASEDSEARCH_H

#include "../../Problems/MultiAgentProblemWithConstraints.h"
#include "../AStar/AStar.h"
#include "ConflictTreeNode.h"

// Conflict Based Search
// - high level of the algorithm - Conflict Tree (CT) as a best first search
//
// Only for multi agent problem
//
// typeOfHeuristic is the heuristic for the A* searches
// If CAT is true, we use a Conflict Avoidance Table (CAT) when replanning to avoid planned paths (if possible with optimal cost)
// If disjointSplitting is true, we use a disjoint splitting strategy (branches on 1 positive and 1 negative constraint instead of 2 negative constraints)
class ConflictBasedSearch {
public:
    ConflictBasedSearch(std::shared_ptr<MultiAgentProblemWithConstraints> problem, TypeOfHeuristic typeOfHeuristic, bool CAT = true, bool disjointSplitting = false);
    std::shared_ptr<Solution> solve();

protected:
    std::shared_ptr<MultiAgentProblemWithConstraints> problem;
    TypeOfHeuristic typeOfHeuristic;
    std::multiset<std::shared_ptr<ConflictTreeNode>, ConflictTreeNodeComparator> fringe; // the open list
    int numberOfVisitedNodes; // number of nodes for which we tested if the set of conflicts is empty or not
    int numberOfNodesLeftInTheFringe;
    bool CAT;
    bool disjointSplitting;

    // Plans a path for each agent
    // Returns a tuple {solutions, cost, costs} where
    // - solutions is a map from the id of an agent to the path of this agent
    // - cost is the cost of all the paths (-1 if it didn't find a solution for one agent)
    // - costs is a map from the id an agent to the cost of the path of this agent
    std::tuple<std::unordered_map<int, std::vector<int>>,int, std::unordered_map<int, int>> planIndividualPaths();

    // Returns the set of conflicts between the paths of solutions
    std::set<AgentConflict> calculateSetOfConflicts(const std::unordered_map<int, std::vector<int>>& solutions);

    // Returns an updated set of conflicts after having replanned agent agentId
    // - fullSolutions contains the paths of the agents
    // - setOfConflicts is the old set of conflicts
    // - agentId is the id of the replanned agent
    // - newPath is the new path of agentId
    std::set<AgentConflict> updateSetOfConflicts(const std::unordered_map<int, std::vector<int>>& fullSolutions, const std::set<AgentConflict>& setOfConflicts, int agentId, std::vector<int> newPath);

    // Retrieves information from the parent nodes
    // Returns a tuple {fullSetOfVertexConstraints, fullSetOfEdgeConstraints, fullCosts, fullSolutions}
    // - fullSetOfVertexConstraints and fullSetOfEdgeConstraints are the sets of constraints from the root node to this node
    // - fullCosts is a map from the id of an agent to the cost of the latest path of this agent
    // - fullSolutions is a map from the id of an agent to the latest path of this agent
    std::tuple<std::set<VertexConstraint>, std::set<EdgeConstraint>, std::unordered_map<int, int>, std::unordered_map<int, vector<int>>> retrieveSetsOfConstraintsAndCostsAndSolutions(std::shared_ptr<ConflictTreeNode> node);

    // Returns a tuple {fullSetOfVertexConstraints, fullSetOfEdgeConstraints, fullCosts, fullSolutions, setOfPositiveConstraints}
    // - setOfPositiveConstraints is the set of positive constraints from the root node to this node
    std::tuple<std::set<VertexConstraint>, std::set<EdgeConstraint>, std::unordered_map<int, int>, std::unordered_map<int, vector<int>>, std::set<VertexConstraint>> retrieveSetsOfConstraintsAndCostsAndSolutionsDisjointSplitting(std::shared_ptr<ConflictTreeNode> node);

    // Retrieves the paths from the parent nodes
    // Returns fullSolutions
    // - fullSolutions is a map from the id of an agent to the latest path of this agent
    std::unordered_map<int, vector<int>> retrieveSolutions(std::shared_ptr<ConflictTreeNode> node);

    // Returns the solution in node with the right format (Solution class and same size for all paths)
    std::shared_ptr<Solution> combineSolutions(const std::shared_ptr<ConflictTreeNode>& node);

    std::shared_ptr<Solution> disjointSplittingSolve();
};


#endif //TFE_MAPF_CONFLICTBASEDSEARCH_H
