//
// Created by Arthur Mahy on 09/11/2022.
//

#ifndef TFE_MAPF_ASTARPROBLEM_H
#define TFE_MAPF_ASTARPROBLEM_H

#include "unordered_map"
#include <iostream>
#include <memory>
#include <vector>

template <class S> class AStarProblem {
public:
  AStarProblem() = default;
  ~AStarProblem() = default;

  // Returns the start state for the search problem
  virtual std::shared_ptr<S> getStartState() const = 0;

  // Returns true if the state is a valid goal state
  virtual bool isGoalState(std::shared_ptr<S> state) const = 0;

  // For a given state, getSuccessors returns a list of pairs (successor,
  // stepcost, numberOfViolations) where successor is a successor state to the
  // current state stepcost is the cost to go from state to successor
  // numberOfViolations is the number of soft constraints that have been
  // violated to go from state to successor
  virtual std::vector<std::tuple<std::shared_ptr<S>, int, int>>
  getSuccessors(std::shared_ptr<S> state) const = 0;

  virtual std::unordered_map<int, std::vector<int>>
  getPositions(std::vector<std::shared_ptr<S>> states) const = 0;

  virtual int getMaxCost() const = 0;

  virtual int getStartTime() const = 0;
};

#endif // TFE_MAPF_ASTARPROBLEM_H
