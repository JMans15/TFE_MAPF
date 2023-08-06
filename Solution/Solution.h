//! class containing the solution to a problem

#ifndef TFE_MAPF_SOLUTION_H
#define TFE_MAPF_SOLUTION_H

#include "../Constraints/VertexConstraint.h"
#include <set>
#include <string>
#include <unordered_map>
#include <vector>

class Solution {
public:
  Solution();

  //! Solutions of ID
  Solution(int numberOfTimesteps,
           std::unordered_map<int, std::vector<int>> positions,
           int numberOfResolvedConflicts, int sizeOfLargerGroup);

  //! Solutions of CBS or A*
  Solution(int cost, int numberOfVisitedNodes, int numberOfTimesteps,
           std::unordered_map<int, std::vector<int>> positions,
           int numberOfNodesLeftInTheFringe, int startTime = 0);

  int getCost() const;
  int getNumberOfVisitedNodes() const;
  int getNumberOfTimesteps() const;
  std::vector<int> getPathOfAgent(int id);
  bool getFoundPath();
  void print();
  void write(const std::string &filename, int w);

  //! Fuel : Total amount of distance traveled by all agents
  int getFuelCost();

  //! Makespan : Total time for the last agent to reach its goal
  int getMakespanCost();

  //! SumOfCosts : The sum of the time steps required for every agent to reach
  //! its goal
  int getSumOfCostsCost();

  //! @return true if all paths don't have any conflict (vertex and edge
  //! conflicts)
  bool isValid();

  //! Map containing the paths of each agent
  std::unordered_map<int, std::vector<int>> getPositions() const;

  //! Lengthens the positions matrix so that the length of a path is length
  void lengthenPositions(int length);

  //! @return true if all paths are consistent according to the positive
  // constraints
  bool isConsistent(const std::set<VertexConstraint> &setOfPositiveConstraints);

private:
  //! Cost of the solution / value of the objective function for this solution
  //! (if solution of a single joint A* or conflict based search)
  int cost;

  //! For solutions of CBS or A* :
  //! The number of visited nodes
  //! - goal tested states if solution of a single joint A*
  //! - conflict tree nodes if solution of conflict based search
  int numberOfVisitedNodes;

  //! For solutions of CBS or A* :
  //! The number of nodes left in the fringe/open list at the end of the search
  int numberOfNodesLeftInTheFringe;

  //! For solutions of ID :
  //! The number of solved conflicts (merging or replanning) until a solution
  //! without any conflict is found
  int numberOfResolvedConflicts;

  //! For solutions of ID :
  //! The size of the largest group of agents (solved by a single low-level
  //! search)
  int sizeOfLargerGroup;

  //! The number of needed timesteps from all agents to reach their target
  //! position
  int numberOfTimesteps;

  //! Map containing the paths
  //! The key is the id of an agent and the value is the path of the agent
  //! positions[a][t] is the position of agent a at time t in this solution
  std::unordered_map<int, std::vector<int>> positions;

  //! Whether a path was found or not
  //! @return true if this solution contains a path
  bool foundPath;

  //! true if solution of ID, false if solution of CBS or A*
  bool solutionOfId;

  //! The time at which the search started (useful for sub-searches in dynamic
  //! environments)
  int startTime;
};

//! Computes the fuel cost of a path
//! @param [in] path The path on which to compute the cost
//! @return The fuel cost of the path
int fuelCost(std::vector<int> path);

//! Computes the makespan cost of a path
//! @param [in] path The path on which to compute the cost
//! @return The makespan cost of the path
int makespanCost(const std::vector<int> &path);

#endif // TFE_MAPF_SOLUTION_H
