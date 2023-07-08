//
// Created by Arthur Mahy on 11/06/2023.
//

#include "ODMultiAgentAStarProblemWithConstraints.h"

#include <utility>

ODMultiAgentAStarProblemWithConstraints::
    ODMultiAgentAStarProblemWithConstraints(
        std::shared_ptr<MultiAgentProblem> problem)
    : AStarProblem<ODMultiAgentSpaceTimeState>(), problem(std::move(problem)) {}

std::shared_ptr<ODMultiAgentSpaceTimeState>
ODMultiAgentAStarProblemWithConstraints::getStartState() const {
  return std::make_shared<ODMultiAgentSpaceTimeState>(
      problem->getStarts(), problem->getStarts(), problem->getStartTime(), 0,
      true);
}

bool ODMultiAgentAStarProblemWithConstraints::isGoalState(
    std::shared_ptr<ODMultiAgentSpaceTimeState> state) const {
  auto positions = state->getPrePositions();
  for (int i = 0; i < problem->getNumberOfAgents(); i++) {
    if (positions[i] != problem->getTargets()[i]) {
      return false;
    }
  }
  return true;
}

// Returns true if position is not already occupied by assigned agents
bool ODMultiAgentAStarProblemWithConstraints::notAlreadyOccupiedPosition(
    int position, std::vector<int> &positions, int agentToAssign) const {
  for (int i = 0; i < agentToAssign; i++) {
    if (positions[i] == position) {
      return false;
    }
  }
  return true;
}

// Returns true if the edge (position, positions[agentToAssign]) is not already
// occupied by assigned agents
bool ODMultiAgentAStarProblemWithConstraints::notAlreadyOccupiedEdge(
    int position, const std::vector<int> &positions, int agentToAssign,
    const std::vector<int> &prePositions) const {
  for (int i = 0; i < agentToAssign; i++) {
    if (prePositions[i] == position &&
        positions[i] == positions[agentToAssign]) {
      return false;
    }
  }
  return true;
}

std::vector<std::tuple<std::shared_ptr<ODMultiAgentSpaceTimeState>, int, int>>
ODMultiAgentAStarProblemWithConstraints::getSuccessors(
    std::shared_ptr<ODMultiAgentSpaceTimeState> state) const {
  std::vector<std::tuple<std::shared_ptr<ODMultiAgentSpaceTimeState>, int, int>>
      successors;
  auto positions = state->getPositions();
  auto prePositions = state->getPrePositions();
  int agentToAssign = state->getAgentToAssign();
  int t = state->getTimestep();
  int nextAgentToAssign;
  int nextT;
  int costMovement;
  int costWait;
  bool isStandard;
  if (problem->getObjFunction() == Fuel) {
    costMovement = 1;
    costWait = 0;
  } else if (problem->getObjFunction() == Makespan) {
    if (agentToAssign == 0) {
      // we are assigning a position to the first agent,
      // we know that we will need another timestep
      costMovement = 1;
      costWait = 1;
    } else {
      costMovement = 0;
      costWait = 0;
    }
  } else { // obj_function=="SumOfCosts"
    costMovement = 1;
  }
  if (agentToAssign == 0) {
    nextT = t + 1;
  } else {
    nextT = t;
  }
  if (agentToAssign == problem->getNumberOfAgents() - 1) {
    // we are assigning a position to the last agent,
    // the next state will be standard
    nextAgentToAssign = 0;
    isStandard = true;
  } else {
    nextAgentToAssign = agentToAssign + 1;
    isStandard = false;
  }

  if (problem->getObjFunction() != SumOfCosts) {

    // Move
    for (int j : problem->getGraph()->getNeighbors(positions[agentToAssign])) {
      vector<int> newpositions(positions);
      newpositions[agentToAssign] = j;
      if (notAlreadyOccupiedPosition(j, positions, agentToAssign) &&
          notAlreadyOccupiedEdge(j, positions, agentToAssign, prePositions) &&
          problem->okForConstraints(agentToAssign, positions[agentToAssign], j,
                                    nextT)) {
        auto successor = std::make_shared<ODMultiAgentSpaceTimeState>(
            newpositions, prePositions, nextT, nextAgentToAssign, isStandard);
        successors.emplace_back(
            successor, costMovement,
            problem->numberOfViolations(agentToAssign, positions[agentToAssign],
                                        j, nextT));
      }
    }

    // Wait
    if (notAlreadyOccupiedPosition(positions[agentToAssign], positions,
                                   agentToAssign) &&
        problem->okForConstraints(agentToAssign, positions[agentToAssign],
                                  nextT)) {
      auto successor = std::make_shared<ODMultiAgentSpaceTimeState>(
          positions, prePositions, nextT, nextAgentToAssign, isStandard);
      successors.emplace_back(
          successor, costWait,
          problem->numberOfViolations(agentToAssign, positions[agentToAssign],
                                      nextT));
    }

  } else { // obj_function=="SumOfCosts"
    auto cannotMove = state->getCannotMove();
    if (state->canMove(agentToAssign)) { // agentToAssign is allowed to move

      // Move
      for (int j :
           problem->getGraph()->getNeighbors(positions[agentToAssign])) {
        vector<int> newpositions(positions);
        newpositions[agentToAssign] = j;
        if (notAlreadyOccupiedPosition(j, positions, agentToAssign) &&
            notAlreadyOccupiedEdge(j, positions, agentToAssign, prePositions) &&
            problem->okForConstraints(agentToAssign, positions[agentToAssign],
                                      j, nextT)) {
          auto successor = std::make_shared<ODMultiAgentSpaceTimeState>(
              newpositions, prePositions, nextT, nextAgentToAssign, isStandard,
              cannotMove);
          successors.emplace_back(
              successor, costMovement,
              problem->numberOfViolations(agentToAssign,
                                          positions[agentToAssign], j, nextT));
        }
      }

      // Wait
      if (positions[agentToAssign] !=
          problem->getTargets()[agentToAssign]) { // agentToAssign not at his
                                                  // target position
        costWait = 1;
        if (notAlreadyOccupiedPosition(positions[agentToAssign], positions,
                                       agentToAssign) &&
            problem->okForConstraints(agentToAssign, positions[agentToAssign],
                                      nextT)) {
          auto successor = std::make_shared<ODMultiAgentSpaceTimeState>(
              positions, prePositions, nextT, nextAgentToAssign, isStandard,
              cannotMove);
          successors.emplace_back(
              successor, costWait,
              problem->numberOfViolations(agentToAssign,
                                          positions[agentToAssign], nextT));
        }
      } else { // agentToAssign is at his target position
        // agentToAssign can still move in the future
        costWait = 1;
        if (notAlreadyOccupiedPosition(positions[agentToAssign], positions,
                                       agentToAssign) &&
            problem->okForConstraints(agentToAssign, positions[agentToAssign],
                                      nextT)) {
          auto successor = std::make_shared<ODMultiAgentSpaceTimeState>(
              positions, prePositions, nextT, nextAgentToAssign, isStandard,
              cannotMove);
          successors.emplace_back(
              successor, costWait,
              problem->numberOfViolations(agentToAssign,
                                          positions[agentToAssign], nextT));
        }

        // we are forcing agentToAssign to not move in the future
        costWait = 0;
        cannotMove[agentToAssign] = true;
        if (notAlreadyOccupiedPosition(positions[agentToAssign], positions,
                                       agentToAssign) &&
            problem->okForConstraints(agentToAssign, positions[agentToAssign],
                                      nextT)) {
          auto successor = std::make_shared<ODMultiAgentSpaceTimeState>(
              positions, prePositions, nextT, nextAgentToAssign, isStandard,
              cannotMove);
          successors.emplace_back(
              successor, costWait,
              problem->numberOfViolations(agentToAssign,
                                          positions[agentToAssign], nextT));
        }
      }

    } else { // agentToAssign is not allowed to move
      costWait = 0;
      if (notAlreadyOccupiedPosition(positions[agentToAssign], positions,
                                     agentToAssign) &&
          problem->okForConstraints(agentToAssign, positions[agentToAssign],
                                    nextT)) {
        auto successor = std::make_shared<ODMultiAgentSpaceTimeState>(
            positions, prePositions, nextT, nextAgentToAssign, isStandard,
            cannotMove);
        successors.emplace_back(
            successor, costWait,
            problem->numberOfViolations(agentToAssign, positions[agentToAssign],
                                        nextT));
      }
    }
  }
  return successors;
}

std::unordered_map<int, std::vector<int>>
ODMultiAgentAStarProblemWithConstraints::getPositions(
    std::vector<std::shared_ptr<ODMultiAgentSpaceTimeState>> states) const {
  std::unordered_map<int, std::vector<int>> positions;

  for (const auto &state : states) {
    if (state->isStandard()) {
      for (int agent = 0; agent < problem->getNumberOfAgents(); agent++) {
        positions[problem->getAgentIds()[agent]].push_back(
            state->getPositions()[agent]);
      }
    }
  }

  return positions;
}

int ODMultiAgentAStarProblemWithConstraints::getMaxCost() const {
  return problem->getMaxCost();
}

int ODMultiAgentAStarProblemWithConstraints::getStartTime() const {
  return problem->getStartTime();
}

std::shared_ptr<MultiAgentProblem>
ODMultiAgentAStarProblemWithConstraints::getProblem() {
  return problem;
}
