//
// Created by Arthur Mahy on 23/11/2022.
//

#include "Solution.h"

#include <algorithm>
#include <fstream>
#include <iostream>
#include <utility>

Solution::Solution() { foundPath = false; }

Solution::Solution(int numberOfTimesteps,
                   std::unordered_map<int, std::vector<int>> positions,
                   int numberOfResolvedConflicts, int sizeOfLargerGroup)
    : foundPath(true), numberOfTimesteps(numberOfTimesteps),
      positions(positions), startTime(0),
      numberOfResolvedConflicts(numberOfResolvedConflicts),
      sizeOfLargerGroup(sizeOfLargerGroup), solutionOfId(true) {}

Solution::Solution(int cost, int numberOfVisitedNodes, int numberOfTimesteps,
                   std::unordered_map<int, std::vector<int>> positions,
                   int numberOfNodesLeftInTheFringe, int startTime)
    : foundPath(true), cost(cost), numberOfVisitedNodes(numberOfVisitedNodes),
      numberOfTimesteps(numberOfTimesteps), positions(positions),
      numberOfNodesLeftInTheFringe(numberOfNodesLeftInTheFringe),
      startTime(startTime), solutionOfId(false) {
  /*if (not isValid()){
      std::cout << "This solution is not valid ! There are conflicts between the
  paths. " << std::endl;
  }*/
}

int Solution::getCost() const { return cost; }

int Solution::getNumberOfVisitedNodes() const { return numberOfVisitedNodes; }

int Solution::getNumberOfTimesteps() const { return numberOfTimesteps; }

std::vector<int> Solution::getPathOfAgent(int id) { return positions[id]; }

void Solution::write(const std::string &filename, int width) {
  std::ofstream file(filename.c_str());
  if (foundPath) {
    int A = static_cast<int>(positions.size());
    int T = numberOfTimesteps;
    file << A << " " << T << std::endl;
    int x, y, p;
    for (int t = 0; t < T; t++) {
      for (auto a : positions) {
        p = a.second[t];
        // The compiler will likely reuse the result of the division, no need to
        // bother optimizing ourselves
        y = p / width;
        x = p % width;
        file << x << "," << y << std::endl;
      }
    }
  }
  file.close();
}

void Solution::print() {
  if (foundPath) {
    std::cout << " " << std::endl;
    std::cout << "==== Solution ====" << std::endl;
    if (not solutionOfId) {
      std::cout << numberOfVisitedNodes << " visited nodes " << std::endl;
      std::cout << " ( = goal tested states if solution of a single joint A*"
                << std::endl;
      std::cout
          << "  or conflict tree nodes if solution of conflict based search)"
          << std::endl;
      std::cout
          << numberOfNodesLeftInTheFringe
          << " nodes left at the end of the search in the fringe/open list "
          << std::endl;
      std::cout << "Cost of the solution = " << cost
                << " (value of the objective function for this solution)"
                << std::endl;
    } else {
      std::cout << numberOfResolvedConflicts
                << " solved conflicts (merging or replanning) until a solution "
                   "without any conflict is found"
                << std::endl;
      std::cout << sizeOfLargerGroup
                << " is the size of the largest group of agents (solved by a "
                   "single low-level search) "
                << std::endl;
    }
    std::cout << " - SumOfCosts cost of the solution = " << getSumOfCostsCost()
              << std::endl;
    std::cout << " - Fuel cost of the solution = " << getFuelCost()
              << std::endl;
    std::cout << " - Makespan cost of the solution = " << getMakespanCost()
              << std::endl;
    std::cout << " " << std::endl;
    std::cout << " -> Position of every agent at each time : " << std::endl;
    int realTime = startTime;
    for (int t = 0; t < positions.begin()->second.size(); t++) {
      std::cout << " -- Time " << realTime << " : " << std::endl;
      for (auto i : positions) {
        std::cout << " --- Agent " << i.first << " : position " << i.second[t]
                  << std::endl;
      }
      realTime++;
    }
    std::cout << " " << std::endl;
    std::cout << " -> Path of each agent : " << std::endl;
    for (auto i : positions) {
      std::cout << " -- Agent " << i.first << " : " << std::endl;
      realTime = startTime;
      for (int t = 0; t < i.second.size(); t++) {
        std::cout << " --- Time " << realTime << " : position " << i.second[t]
                  << std::endl;
        realTime++;
      }
    }
  }
}

bool Solution::getFoundPath() { return foundPath; }

int Solution::getFuelCost() {
  int cost = 0;
  for (auto a : positions) {
    for (int t = 1; t < numberOfTimesteps; t++) {
      if (a.second[t] != a.second[t - 1]) {
        cost += 1;
      }
    }
  }
  return cost;
}

int Solution::getMakespanCost() { return numberOfTimesteps - 1; }

int Solution::getSumOfCostsCost() {
  int cost = 0;
  for (auto a : positions) {
    for (int t = numberOfTimesteps - 1; t >= 0; t--) {
      if (a.second[t] != a.second[numberOfTimesteps - 1]) {
        cost += t + 1;
        break;
      }
    }
  }
  return cost;
}

bool Solution::isValid() {
  for (auto [agentA, pathA] : positions) {
    for (auto [agentB, pathB] : positions) {
      if (agentA < agentB) {
        // Vertex conflict
        for (int t = 0; t < numberOfTimesteps; t++) {
          if (pathA[t] == pathB[t]) {
            return false;
          }
        }
        // Edge conflict
        for (int t = 0; t < numberOfTimesteps - 1; t++) {
          if (pathA[t] == pathB[t + 1] and pathA[t + 1] == pathB[t]) {
            return false;
          }
        }
      }
    }
  }
  return true;
}

bool Solution::isConsistent(
    const std::set<VertexConstraint> &setOfPositiveConstraints) {
  for (const auto &constraint : setOfPositiveConstraints) {
    if (!positions[constraint.getAgent()].empty()) {
      if (positions[constraint.getAgent()][constraint.getTime()] !=
          constraint.getPosition()) {
        // std::cout << "solution inconsistent with the positive constraints" <<
        // std::endl;
        return false;
      }
    }
  }
  return true;
}

std::unordered_map<int, std::vector<int>> Solution::getPositions() const {
  return positions;
}

void Solution::lengthenPositions(int length) {
  int actualLength = positions.begin()->second.size();
  if (actualLength < length) {
    for (auto i : positions) {
      positions[i.first].resize(length, i.second[actualLength - 1]);
    }
  }
}

int fuelCost(std::vector<int> path) {
  int cost = 0;
  for (int t = 1; t < (int)path.size(); t++) {
    if (path[t] != path[t - 1]) {
      cost += 1;
    }
  }
  return cost;
}

int makespanCost(const std::vector<int> &path) { return (int)path.size() - 1; }
