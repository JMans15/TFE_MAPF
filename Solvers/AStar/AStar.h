//! Generic A*

#ifndef TFE_MAPF_ASTAR_H
#define TFE_MAPF_ASTAR_H

#include "../../AStarProblems/AStarProblem.h"
#include "../../Heuristics/HeuristicOptimal.h"
#include "../../Solution/Solution.h"

#include <algorithm>
#include <unordered_map>

// Uses an instance of the abstract class AStarProblem
// Uses an instance of the abstract class Heuristic
template <class P, class S> class AStar {
public:
  //! Constructor for A* solver
  //! @param [in] problem Problem to be solved by A* (can be single agent or
  //! multi agent)
  //! @param [in] typeOfHeuristic Type of heuristic to use
  AStar(std::shared_ptr<P> problem, TypeOfHeuristic typeOfHeuristic)
      : problem(problem),
        heuristic(getHeuristic<P, S>(
            problem, std::make_shared<TypeOfHeuristic>(typeOfHeuristic))),
        numberOfVisitedStates(0) {}

  //! Solve problem and return Solution
  std::shared_ptr<Solution> solve() {
    LOG("===== A* Search ====");

    LOG("Beginning the A* search. ");

    auto start = problem->getStartState();
    fringe.insert(std::make_shared<Node<S>>(
        start, 0, heuristic->heuristicFunction(start)));
    distance[start] = 0;

    while (!fringe.empty()) {
      auto it = fringe.begin();
      const auto node = *it;

      fringe.erase(it);

      auto nodeState = node->getState();
      if (node->getCost() > distance[nodeState]) {
        continue;
      }

      numberOfVisitedStates += 1;

      if (problem->isGoalState(nodeState)) {
        numberOfNodesLeftInTheFringe = (int)fringe.size();
        return retrieveSolution(node);
      }

      auto successors = problem->getSuccessors(nodeState);
      for (auto &[successor, edgeCost, edgeNumberOfViolations] : successors) {
        auto successorCost = node->getCost() + edgeCost;
        if (successorCost <= problem->getMaxCost()) {
          auto it = distance.find(successor);
          if (it == distance.end() || successorCost < it->second) {
            distance[successor] = successorCost;
            auto h = heuristic->heuristicFunction(successor);
            auto successorViolationCount =
                node->getViolationCount() + edgeNumberOfViolations;
            fringe.insert(std::make_shared<Node<S>>(
                successor, successorCost, h, node, successorViolationCount));
          }
        }
      }
    }

    LOG("No path has been found.");
    return std::make_shared<Solution>();
  }

private:
  std::shared_ptr<P>
      problem; //!< Problem to be solved, multi agebt or single agent
  std::shared_ptr<Heuristic<S>>
      heuristic; //!< Heuristic to use (which definition of State)
  std::multiset<std::shared_ptr<Node<S>>, NodeComparator<S>>
      fringe; //!< the open list / frontier
  std::unordered_map<std::shared_ptr<S>, int, StateHasher<S>, StateEquality<S>>
      distance;                     //!< the closed list (-> graph search)
  int numberOfVisitedStates;        //!< Number of visited states
  int numberOfNodesLeftInTheFringe; //!< Number of nodes left in the fringe
  //! Goes from the Node node to the root and extract the Solution
  std::shared_ptr<Solution> retrieveSolution(std::shared_ptr<Node<S>> node) {
    int cost = node->getCost();

    std::vector<std::shared_ptr<S>> states;
    while (node) {
      states.push_back(node->getState());
      node = node->getParent();
    }
    std::reverse(states.begin(), states.end());

    auto positions = problem->getPositions(states);
    int numberOfTimesteps = positions.begin()->second.size();

    LOG("The search is over. We found an optimal solution. ");
    return std::make_shared<Solution>(
        cost, numberOfVisitedStates, numberOfTimesteps, positions,
        numberOfNodesLeftInTheFringe, problem->getStartTime());
  }
};

#endif // TFE_MAPF_ASTAR_H
