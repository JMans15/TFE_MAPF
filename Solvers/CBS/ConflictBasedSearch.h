//
// Created by Arthur Mahy on 10/04/2023.
//

#ifndef TFE_MAPF_CONFLICTBASEDSEARCH_H
#define TFE_MAPF_CONFLICTBASEDSEARCH_H

#include "../../Problems/MultiAgentProblem.h"
#include "../AStar/GeneralAStar.h"
#include "ConflictTreeNode.h"

//! Conflict Based Search
//! - high level of the algorithm - Conflict Tree (CT) as a best first search
//!
//! Only takes in account negative constraints of problem
//!
//! typeOfHeuristic is the heuristic for the A* searches
//! If CAT is true, we use a Conflict Avoidance Table (CAT) when replanning to
//! avoid planned paths (if possible with optimal cost)
//! If disjointSplitting is true, we use a disjoint splitting strategy
//! (branches on 1 positive and 1 negative constraint instead of 2 negative constraints)
//! (DS is a technique designed to ensure that expanding a CT node N creates a
//! disjoint partition of the space of solutions that satisfy the constraints of N. That is, every
//! solution that satisfies the constraints of N is in exactly one of its children.)
//!
//! CBS with disjoint splitting is not optimal with the Fuel objective function, but well
//! with the other objective functions
class ConflictBasedSearch {
public:
  //! Constructor for ConflictBasedSearch solver for a specified Problem
  //! @param [in] typeOfHeuristic Type of heuristic to use
  //! @param [in] CAT true if CAT is to be used
  //! @param [in] disjointSplitting true if DS is to be used
  //! @param [in] problem Problem to solve
  ConflictBasedSearch(std::shared_ptr<MultiAgentProblem> problem,
                      TypeOfHeuristic typeOfHeuristic, bool CAT = true,
                      bool disjointSplitting = false);
  std::shared_ptr<Solution> solve();

  //! Constructor for ConflictBasedSearch solver, no problem specified yet
  //! @param [in] typeOfHeuristic Type of heuristic to use
  //! @param [in] CAT true if CAT is to be used
  //! @param [in] disjointSplitting true if DS is to be used
  explicit ConflictBasedSearch(TypeOfHeuristic typeOfHeuristic, bool CAT = true,
                               bool disjointSplitting = false);

  //! Solve a particular Problem
  //! @return found Solution
  //! @param [in] problem Problem to solve
  std::shared_ptr<Solution> solve(std::shared_ptr<MultiAgentProblem> problem);

protected:
  std::shared_ptr<MultiAgentProblem>
      problem;                     /**< Multi agent problem to solve */
  TypeOfHeuristic typeOfHeuristic; /**< Type of heuristic to use */
  std::multiset<std::shared_ptr<ConflictTreeNode>, ConflictTreeNodeComparator>
      fringe;               /**< The open list */
  int numberOfVisitedNodes; /**< number of nodes for which we tested if the set
                             of conflicts is empty or not */
  int numberOfNodesLeftInTheFringe; /**< Number of nodes currently in the fringe
                                     */
  bool CAT;                         /**< true if CAT is to be used */
  bool disjointSplitting;           /**< true if DS is to be used */

  //! Plans a path for each agent individually
  //! @return a tuple {solutions, cost, costs} where
  //! - solutions is a map from the id of an agent to the path of this agent
  //! - cost is the cost of all the paths (-1 if it didn't find a solution for
  //! one agent)
  //! - costs is a map from the id an agent to the cost of the path of this
  //! agent
  std::tuple<std::unordered_map<int, std::vector<int>>, int,
             std::unordered_map<int, int>>
  planIndividualPaths();

  //! @param [in] Solution found for each agent
  //! @return the set of conflicts between the paths of solutions
  std::set<AgentConflict> calculateSetOfConflicts(
      const std::unordered_map<int, std::vector<int>> &solutions);

  //! @return an updated set of conflicts after having replanned agent agentId
  //! @param [in] fullSolutions contains the paths of the agents
  //! @param [in] setOfConflicts is the old set of conflicts
  //! @param [in] agentId is the id of the replanned agent
  //! @param [in] newPath is the new path of agentId
  std::set<AgentConflict> updateSetOfConflicts(
      const std::unordered_map<int, std::vector<int>> &fullSolutions,
      const std::set<AgentConflict> &setOfConflicts, int agentId,
      std::vector<int> newPath);

  //! Retrieves information from the parent nodes
  //! @return a tuple {fullSetOfVertexConstraints, fullSetOfEdgeConstraints,
  //! fullCosts, fullSolutions}
  //! - fullSetOfVertexConstraints and fullSetOfEdgeConstraints are
  //! the sets of constraints from the root node to this node
  //! - fullCosts is a map from the id of an agent to the cost of the
  //! latest path of this agent
  //! - fullSolutions is a map from the id of an agent to the latest
  //! path of this agent
  //! @param [in] node node to retrieve informations about
  std::tuple<HardVertexConstraintsSet, HardEdgeConstraintsSet,
             std::unordered_map<int, int>, std::unordered_map<int, vector<int>>>
  retrieveSetsOfConstraintsAndCostsAndSolutions(
      std::shared_ptr<ConflictTreeNode> node);

  //! @return a tuple {fullSetOfVertexConstraints, fullSetOfEdgeConstraints,
  //! fullCosts, fullSolutions, setOfPositiveConstraints}
  //! - setOfPositiveConstraints is the set of positive constraints from the
  //! root node to this node
  //! @param [in] node node to retrieve informations about
  std::tuple<HardVertexConstraintsSet, HardEdgeConstraintsSet,
             std::unordered_map<int, int>, std::unordered_map<int, vector<int>>,
             std::set<VertexConstraint>>
  retrieveSetsOfConstraintsAndCostsAndSolutionsDisjointSplitting(
      std::shared_ptr<ConflictTreeNode> node);

  //! Retrieves the paths from the parent nodes
  //! @return fullSolutions
  //! - fullSolutions is a map from the id of an agent to the latest path of
  //! this agent
  //! @param [in] node node to retrieve the solution from
  std::unordered_map<int, vector<int>>
  retrieveSolutions(std::shared_ptr<ConflictTreeNode> node);

  //! @return the solution in node with the right format (Solution class and
  //! same size for all paths)
  //! @param [in] node node to retreive the solution from
  std::shared_ptr<Solution>
  combineSolutions(const std::shared_ptr<ConflictTreeNode> &node);

  //! Runs disjoint splitting
  //! @return the solution found by disjoint splitting
  std::shared_ptr<Solution> disjointSplittingSolve();

  //! @return true if agent with id agentId can be at position between t1
  //! and t2 according to the setOfVertexConstraints
  //! @param [in] setOfVertexConstraints The set of hard constraint to be
  //! respected
  //! @param [in] t1 start of time frame to check the constraint for
  //! @param [in] t2 end of time frame to check the constraint for
  //! @param [in] position Position at which to check the constraints for
  //! @param [in] agentId Id of agent to check the constraints for
  static bool
  okForHardConstraints(HardVertexConstraintsSet setOfVertexConstraints, int t1,
                       int t2, int position, int agentId);
};

#endif // TFE_MAPF_CONFLICTBASEDSEARCH_H
