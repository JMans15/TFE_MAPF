//
// Created by Arthur Mahy on 04/01/2023.
//

#include "MultiAgentProblem.h"
#include <algorithm>

MultiAgentProblem::MultiAgentProblem(
    const std::shared_ptr<Graph> &graph, std::vector<int> starts,
    std::vector<int> targets, ObjectiveFunction objective,
    const std::vector<int> &m_agentIds,
    const HardVertexConstraintsSet &setOfHardVertexConstraints,
    const HardEdgeConstraintsSet &setOfHardEdgeConstraints, int maxCost,
    const SoftVertexConstraintsMultiSet &setOfSoftVertexConstraints,
    const SoftEdgeConstraintsMultiSet &setOfSoftEdgeConstraints, int startTime)
    : graph(graph), numberOfAgents((int)starts.size()), starts(starts),
      targets(targets), agentIds(m_agentIds), objective(objective),
      setOfHardVertexConstraints(setOfHardVertexConstraints),
      setOfHardEdgeConstraints(setOfHardEdgeConstraints),
      setOfSoftVertexConstraints(setOfSoftVertexConstraints),
      setOfSoftEdgeConstraints(setOfSoftEdgeConstraints), maxCost(maxCost),
      startTime(startTime) {
  if (objective != SumOfCosts && objective != Makespan && objective != Fuel) {
    objective = Makespan;
  }
  impossible = false;
  if ((not m_agentIds.empty()) and ((int)m_agentIds.size() != numberOfAgents)) {
    impossible = true;
  }
  if (numberOfAgents != (int)targets.size()) {
    impossible = true;
  }
  if (m_agentIds.empty()) {
    for (int a = 0; a < numberOfAgents; a++) {
      agentIds.emplace_back(a);
    }
  }
  for (int i = 0; i < numberOfAgents; i++) {
    idToIndex[agentIds[i]] = i;
  }
  for (int agentId : agentIds) {
    if (std::count(agentIds.begin(), agentIds.end(), agentId) > 1) {
      impossible = true;
      break;
    }
  }
  for (int i = 0; i < numberOfAgents; i++) {
    if (graph->getNeighbors(starts[i]).empty()) {
      impossible = true;
    }
  }
  for (int i = 0; i < numberOfAgents; i++) {
    if (graph->getNeighbors(targets[i]).empty()) {
      impossible = true;
    }
  }
  for (int start : starts) {
    if (std::count(starts.begin(), starts.end(), start) > 1) {
      impossible = true;
      break;
    }
  }
  for (int target : targets) {
    if (std::count(targets.begin(), targets.end(), target) > 1) {
      impossible = true;
      break;
    }
  }
  externalConstraints = false;
  if (not setOfHardVertexConstraints.empty()) {
    externalConstraints = true;
  }
  if (not setOfHardEdgeConstraints.empty()) {
    externalConstraints = true;
  }
  if (not setOfSoftVertexConstraints.empty()) {
    externalConstraints = true;
  }
  if (not setOfSoftEdgeConstraints.empty()) {
    externalConstraints = true;
  }
}

bool MultiAgentProblem::okForConstraints(int agent, int position,
                                         int newPosition, int time) const {
  if (setOfHardVertexConstraints.find({agentIds[agent], newPosition, time}) ==
      setOfHardVertexConstraints.end()) {
    return setOfHardEdgeConstraints.find(
               {agentIds[agent], position, newPosition, time}) ==
           setOfHardEdgeConstraints.end();
  }
  return false;
}

bool MultiAgentProblem::okForConstraints(int agent, int newPosition,
                                         int time) const {
  return setOfHardVertexConstraints.find(
             {agentIds[agent], newPosition, time}) ==
         setOfHardVertexConstraints.end();
}

int MultiAgentProblem::numberOfViolations(int agent, int position,
                                          int newPosition, int time) const {
  int count = 0;
  auto range = setOfSoftVertexConstraints.equal_range({0, newPosition, time});
  for (auto it = range.first; it != range.second; ++it) {
    if (it->getAgent() != agentIds[agent]) {
      count += 1;
    }
  }
  auto range2 =
      setOfSoftEdgeConstraints.equal_range({0, position, newPosition, time});
  for (auto it = range2.first; it != range2.second; ++it) {
    if (it->getAgent() != agentIds[agent]) {
      count += 1;
    }
  }
  return count;
}

int MultiAgentProblem::numberOfViolations(int agent, int newPosition,
                                          int time) const {
  int count = 0;
  auto range = setOfSoftVertexConstraints.equal_range({0, newPosition, time});
  for (auto it = range.first; it != range.second; ++it) {
    if (it->getAgent() != agentIds[agent]) {
      count += 1;
    }
  }
  return count;
}

std::vector<int> MultiAgentProblem::getStarts() const { return starts; }

std::vector<int> MultiAgentProblem::getTargets() const { return targets; }

ObjectiveFunction MultiAgentProblem::getObjFunction() { return objective; }

HardVertexConstraintsSet
MultiAgentProblem::getSetOfHardVertexConstraints() const {
  return setOfHardVertexConstraints;
}

HardEdgeConstraintsSet MultiAgentProblem::getSetOfHardEdgeConstraints() const {
  return setOfHardEdgeConstraints;
}

std::vector<int> MultiAgentProblem::getAgentIds() const { return agentIds; }

int MultiAgentProblem::getStartOf(int id) { return starts[idToIndex[id]]; }

int MultiAgentProblem::getTargetOf(int id) { return targets[idToIndex[id]]; }

bool MultiAgentProblem::isImpossible() const { return impossible; }

SoftVertexConstraintsMultiSet
MultiAgentProblem::getSetOfSoftVertexConstraints() const {
  return setOfSoftVertexConstraints;
}

SoftEdgeConstraintsMultiSet
MultiAgentProblem::getSetOfSoftEdgeConstraints() const {
  return setOfSoftEdgeConstraints;
}

std::shared_ptr<Graph> MultiAgentProblem::getGraph() const { return graph; }

int MultiAgentProblem::getNumberOfAgents() const { return numberOfAgents; }

int MultiAgentProblem::getMaxCost() const { return maxCost; }

int MultiAgentProblem::getStartTime() const { return startTime; }

bool MultiAgentProblem::hasExternalConstraints() const {
  return externalConstraints;
}

void MultiAgentProblem::print() const {
  std::cout << "==== Multi Agent Problem ====" << std::endl;
  std::cout << "Number of agents : " << numberOfAgents << std::endl;
  if (objective == SumOfCosts) {
    std::cout << "Objective function : SumOfCosts" << std::endl;
  } else if (objective == Makespan) {
    std::cout << "Objective function : Makespan" << std::endl;
  } else {
    std::cout << "Objective function : Fuel" << std::endl;
  }
  for (int agentId : agentIds) {
    if (std::count(agentIds.begin(), agentIds.end(), agentId) > 1) {
      std::cout << "2 or more agents have the same id." << std::endl;
    }
  }
  std::cout << "Start position of each agent :" << std::endl;
  for (int i = 0; i < numberOfAgents; i++) {
    std::cout << " - Agent " << agentIds[i] << " : " << starts[i] << std::endl;
    if (graph->getNeighbors(starts[i]).empty()) {
      std::cout << "   The start position of agent " << agentIds[i]
                << " is unreachable." << std::endl;
    }
  }
  std::cout << "Target position of each agent :" << std::endl;
  for (int i = 0; i < numberOfAgents; i++) {
    std::cout << " - Agent " << agentIds[i] << " : " << targets[i] << std::endl;
    if (graph->getNeighbors(targets[i]).empty()) {
      std::cout << "   The target position of agent " << agentIds[i]
                << " is unreachable." << std::endl;
    }
  }
  for (int start : starts) {
    if (std::count(starts.begin(), starts.end(), start) > 1) {
      std::cout << "   2 or more agents have the same start position."
                << std::endl;
    }
  }
  for (int target : targets) {
    if (std::count(targets.begin(), targets.end(), target) > 1) {
      std::cout << "   2 or more agents have the same target position."
                << std::endl;
    }
  }
  if (!setOfHardVertexConstraints.empty()) {
    std::cout << "The problem has the following hard vertex constraints :"
              << std::endl;
    for (const auto &constraint : setOfHardVertexConstraints) {
      std::cout << "   Agent " << constraint.getAgent()
                << " cannot be at position " << constraint.getPosition()
                << " at time " << constraint.getTime() << "." << std::endl;
    }
  }
  if (!setOfHardEdgeConstraints.empty()) {
    std::cout << "The problem has the following hard edge constraints :"
              << std::endl;
    for (const auto &constraint : setOfHardEdgeConstraints) {
      std::cout << "   Agent " << constraint.getAgent()
                << " cannot go from position " << constraint.getPosition1()
                << " to position " << constraint.getPosition2() << " between "
                << constraint.getTime() - 1 << " and time "
                << constraint.getTime() << "." << std::endl;
    }
  }
  if (!setOfSoftVertexConstraints.empty()) {
    std::cout << "The problem has the following soft vertex constraints :"
              << std::endl;
    for (const auto &constraint : setOfSoftVertexConstraints) {
      std::cout << "   Agent " << constraint.getAgent() << " is at position "
                << constraint.getPosition() << " at time "
                << constraint.getTime() << "." << std::endl;
    }
  }
  if (!setOfSoftEdgeConstraints.empty()) {
    std::cout << "The problem has the following soft edge constraints :"
              << std::endl;
    for (const auto &constraint : setOfSoftEdgeConstraints) {
      std::cout << "   Agent " << constraint.getAgent()
                << " is occupying the edge (" << constraint.getPosition1()
                << ", " << constraint.getPosition2() << ") between time "
                << constraint.getTime() - 1 << " and time "
                << constraint.getTime() << "." << std::endl;
    }
  }
  if (maxCost != INT_MAX) {
    std::cout
        << "The solution of this problem must have a cost inferior or equal to "
        << maxCost << std::endl;
  }
  if (startTime != 0) {
    std::cout << "The start time of the problem is " << startTime << std::endl;
  }
}
