//
// Created by Arthur Mahy on 11/06/2023.
//

#include "StandardMultiAgentAStarProblem.h"

#include <utility>

StandardMultiAgentAStarProblem::StandardMultiAgentAStarProblem(
    std::shared_ptr<MultiAgentProblem> problem)
    : AStarProblem<StandardMultiAgentState>(), problem(std::move(problem)) {}

std::shared_ptr<StandardMultiAgentState>
StandardMultiAgentAStarProblem::getStartState() const {
  return std::make_shared<StandardMultiAgentState>(problem->getStarts());
}

bool StandardMultiAgentAStarProblem::isGoalState(
    std::shared_ptr<StandardMultiAgentState> state) const {
  auto positions = state->getPositions();
  for (int i = 0; i < problem->getNumberOfAgents(); i++) {
    if (positions[i] != problem->getTargets()[i]) {
      return false;
    }
  }
  return true;
}

// Returns true if position is not already occupied by assigned agents
bool StandardMultiAgentAStarProblem::notAlreadyOccupiedPosition(
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
bool StandardMultiAgentAStarProblem::notAlreadyOccupiedEdge(
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

void StandardMultiAgentAStarProblem::recursiveAssignAMoveToAnAgent(
    int agentToAssign,
    std::vector<std::tuple<std::shared_ptr<StandardMultiAgentState>, int, int>>
        *successors,
    int cost, std::vector<int> positions, const std::vector<int> &prePositions,
    std::vector<u_int8_t> cannotMove) const {
  int costMovement;
  int costWait;
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
  if (problem->getObjFunction() != SumOfCosts) {
    if (agentToAssign == problem->getNumberOfAgents() - 1) {
      // Move
      for (int j :
           problem->getGraph()->getNeighbors(positions[agentToAssign])) {
        vector<int> newpositions(positions);
        newpositions[agentToAssign] = j;
        if (notAlreadyOccupiedPosition(j, positions, agentToAssign) &&
            notAlreadyOccupiedEdge(j, positions, agentToAssign, prePositions)) {
          auto successor =
              std::make_shared<StandardMultiAgentState>(newpositions);
          successors->emplace_back(successor, cost + costMovement, 0);
        }
      }
      // Wait
      if (notAlreadyOccupiedPosition(positions[agentToAssign], positions,
                                     agentToAssign)) {
        auto successor = std::make_shared<StandardMultiAgentState>(positions);
        successors->emplace_back(successor, cost + costWait, 0);
      }
    } else {
      // Move
      for (int j :
           problem->getGraph()->getNeighbors(positions[agentToAssign])) {
        vector<int> newpositions(positions);
        newpositions[agentToAssign] = j;
        if (notAlreadyOccupiedPosition(j, positions, agentToAssign) &&
            notAlreadyOccupiedEdge(j, positions, agentToAssign, prePositions)) {
          recursiveAssignAMoveToAnAgent(agentToAssign + 1, successors,
                                        cost + costMovement, newpositions,
                                        prePositions);
        }
      }
      // Wait
      if (notAlreadyOccupiedPosition(positions[agentToAssign], positions,
                                     agentToAssign)) {
        recursiveAssignAMoveToAnAgent(agentToAssign + 1, successors,
                                      cost + costWait, positions, prePositions);
      }
    }
  } else { // obj_function=="SumOfCosts"
    if (agentToAssign == problem->getNumberOfAgents() - 1) {
      if (not cannotMove[agentToAssign]) { // agentToAssign is allowed to move

        // Move
        for (int j :
             problem->getGraph()->getNeighbors(positions[agentToAssign])) {
          vector<int> newpositions(positions);
          newpositions[agentToAssign] = j;
          if (notAlreadyOccupiedPosition(j, positions, agentToAssign) &&
              notAlreadyOccupiedEdge(j, positions, agentToAssign,
                                     prePositions)) {
            auto successor = std::make_shared<StandardMultiAgentState>(
                newpositions, cannotMove);
            successors->emplace_back(successor, cost + costMovement, 0);
          }
        }

        // Wait
        if (positions[agentToAssign] !=
            problem->getTargets()[agentToAssign]) { // agentToAssign not at his
                                                    // target position
          costWait = 1;
          if (notAlreadyOccupiedPosition(positions[agentToAssign], positions,
                                         agentToAssign)) {
            auto successor = std::make_shared<StandardMultiAgentState>(
                positions, cannotMove);
            successors->emplace_back(successor, cost + costWait, 0);
          }
        } else { // agentToAssign is at his target position
          // agentToAssign can still move in the future
          costWait = 1;
          if (notAlreadyOccupiedPosition(positions[agentToAssign], positions,
                                         agentToAssign)) {
            auto successor = std::make_shared<StandardMultiAgentState>(
                positions, cannotMove);
            successors->emplace_back(successor, cost + costWait, 0);
          }

          // we are forcing agentToAssign to not move in the future
          costWait = 0;
          cannotMove[agentToAssign] = true;
          if (notAlreadyOccupiedPosition(positions[agentToAssign], positions,
                                         agentToAssign)) {
            auto successor = std::make_shared<StandardMultiAgentState>(
                positions, cannotMove);
            successors->emplace_back(successor, cost + costWait, 0);
          }
        }

      } else { // agentToAssign is not allowed to move
        costWait = 0;
        if (notAlreadyOccupiedPosition(positions[agentToAssign], positions,
                                       agentToAssign)) {
          auto successor =
              std::make_shared<StandardMultiAgentState>(positions, cannotMove);
          successors->emplace_back(successor, cost + costWait, 0);
        }
      }
    } else {
      if (not cannotMove[agentToAssign]) { // agentToAssign is allowed to move

        // Move
        for (int j :
             problem->getGraph()->getNeighbors(positions[agentToAssign])) {
          vector<int> newpositions(positions);
          newpositions[agentToAssign] = j;
          if (notAlreadyOccupiedPosition(j, positions, agentToAssign) &&
              notAlreadyOccupiedEdge(j, positions, agentToAssign,
                                     prePositions)) {
            recursiveAssignAMoveToAnAgent(agentToAssign + 1, successors,
                                          cost + costMovement, newpositions,
                                          prePositions, cannotMove);
          }
        }

        // Wait
        if (positions[agentToAssign] !=
            problem->getTargets()[agentToAssign]) { // agentToAssign not at his
                                                    // target position
          costWait = 1;
          if (notAlreadyOccupiedPosition(positions[agentToAssign], positions,
                                         agentToAssign)) {
            recursiveAssignAMoveToAnAgent(agentToAssign + 1, successors,
                                          cost + costWait, positions,
                                          prePositions, cannotMove);
          }
        } else { // agentToAssign is at his target position
          // agentToAssign can still move in the future
          costWait = 1;
          if (notAlreadyOccupiedPosition(positions[agentToAssign], positions,
                                         agentToAssign)) {
            recursiveAssignAMoveToAnAgent(agentToAssign + 1, successors,
                                          cost + costWait, positions,
                                          prePositions, cannotMove);
          }

          // we are forcing agentToAssign to not move in the future
          costWait = 0;
          cannotMove[agentToAssign] = true;
          if (notAlreadyOccupiedPosition(positions[agentToAssign], positions,
                                         agentToAssign)) {
            recursiveAssignAMoveToAnAgent(agentToAssign + 1, successors,
                                          cost + costWait, positions,
                                          prePositions, cannotMove);
          }
        }

      } else { // agentToAssign is not allowed to move
        costWait = 0;
        if (notAlreadyOccupiedPosition(positions[agentToAssign], positions,
                                       agentToAssign)) {
          recursiveAssignAMoveToAnAgent(agentToAssign + 1, successors,
                                        cost + costWait, positions,
                                        prePositions, cannotMove);
        }
      }
    }
  }
}

std::vector<std::tuple<std::shared_ptr<StandardMultiAgentState>, int, int>>
StandardMultiAgentAStarProblem::getSuccessors(
    std::shared_ptr<StandardMultiAgentState> state) const {
  std::vector<std::tuple<std::shared_ptr<StandardMultiAgentState>, int, int>>
      successors;
  auto positions = state->getPositions();

  recursiveAssignAMoveToAnAgent(0, &successors, 0, positions, positions,
                                state->getCannotMove());

  return successors;
}

std::unordered_map<int, std::vector<int>>
StandardMultiAgentAStarProblem::getPositions(
    std::vector<std::shared_ptr<StandardMultiAgentState>> states) const {
  std::unordered_map<int, std::vector<int>> positions;

  for (const auto &state : states) {
    for (int agent = 0; agent < problem->getNumberOfAgents(); agent++) {
      positions[problem->getAgentIds()[agent]].push_back(
          state->getPositions()[agent]);
    }
  }

  return positions;
}

int StandardMultiAgentAStarProblem::getMaxCost() const {
  return problem->getMaxCost();
}

int StandardMultiAgentAStarProblem::getStartTime() const {
  return problem->getStartTime();
}

std::shared_ptr<MultiAgentProblem>
StandardMultiAgentAStarProblem::getProblem() {
  return problem;
}
