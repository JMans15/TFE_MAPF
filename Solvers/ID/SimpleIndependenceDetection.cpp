//
// Created by Arthur Mahy on 15/05/2023.
//

#include "SimpleIndependenceDetection.h"
#include <unordered_map>
#include <utility>

template <class MultiAgentSolver>
SimpleIndependenceDetection<MultiAgentSolver>::SimpleIndependenceDetection(
    std::shared_ptr<MultiAgentProblem> problem,
    std::shared_ptr<MultiAgentSolver> lowLevelSearch, bool CAT)
    : problem(std::move(problem)), CAT(CAT), lowLevelSearch(lowLevelSearch),
      numberOfResolvedConflicts(0), sizeOfLargerGroup(1) {}

template <class MultiAgentSolver>
SimpleIndependenceDetection<MultiAgentSolver>::SimpleIndependenceDetection(
    std::shared_ptr<MultiAgentSolver> lowLevelSearch, bool CAT)
    : problem(nullptr), CAT(CAT), lowLevelSearch(lowLevelSearch),
      numberOfResolvedConflicts(0), sizeOfLargerGroup(1) {}

template <class MultiAgentSolver>
std::shared_ptr<Solution> SimpleIndependenceDetection<MultiAgentSolver>::solve(
    std::shared_ptr<MultiAgentProblem> m_problem) {
  problem = std::move(m_problem);
  groups =
      std::unordered_set<std::shared_ptr<Group>, GroupHasher, GroupEquality>();
  setOfConflicts = std::set<GroupConflict>();
  vertexConflictAvoidanceTable = problem->getSetOfSoftVertexConstraints();
  edgeConflictAvoidanceTable = problem->getSetOfSoftEdgeConstraints();
  numberOfResolvedConflicts = 0;
  return solve();
}

template <class MultiAgentSolver>
bool SimpleIndependenceDetection<MultiAgentSolver>::planSingletonGroups() {
  for (const std::shared_ptr<Group> &group : groups) {
    int agentId = *group->getAgents().begin();
    auto prob = std::make_shared<SingleAgentProblem>(
        problem->getGraph(), problem->getStartOf(agentId),
        problem->getTargetOf(agentId), problem->getObjFunction(), agentId,
        problem->getSetOfHardVertexConstraints(),
        problem->getSetOfHardEdgeConstraints(), INT_MAX,
        vertexConflictAvoidanceTable, edgeConflictAvoidanceTable);
    auto solution =
        AStar<SingleAgentAStarProblemWithConstraints,
              SingleAgentSpaceTimeState>(
            std::make_shared<SingleAgentAStarProblemWithConstraints>(prob),
            OptimalDistance)
            .solve();
    if (not solution->getFoundPath()) {
      return false;
    }
    group->putSolution(solution);
    if (CAT) {
      vector<int> pathOfAgent = solution->getPositions().begin()->second;
      for (int t = 0; t < pathOfAgent.size(); t++) {
        vertexConflictAvoidanceTable.insert({agentId, pathOfAgent[t], t});
      }
      for (int t = 1; t < pathOfAgent.size(); t++) {
        edgeConflictAvoidanceTable.insert(
            {agentId, pathOfAgent[t], pathOfAgent[t - 1], t});
        // edgeConflictAvoidanceTable.insert({agentId, pathOfAgent[t-1],
        // pathOfAgent[t], t});
      }
    }
  }
  return true;
}

template <class MultiAgentSolver>
void SimpleIndependenceDetection<MultiAgentSolver>::calculateSetOfConflicts() {
  for (const auto &groupA : groups) {
    for (const auto &groupB : groups) {
      if (*groupA != *groupB) {
        for (auto [agentA, pathA] : groupA->getSolution()->getPositions()) {
          for (auto [agentB, pathB] : groupB->getSolution()->getPositions()) {
            if (agentA != agentB and pathA.size() <= pathB.size()) {
              // Vertex conflict
              for (int t = 0; t < pathA.size(); t++) {
                if (pathA[t] == pathB[t]) {
                  setOfConflicts.insert({groupA, groupB, t});
                }
              }
              int targetA = problem->getTargetOf(agentA);
              for (int t = pathA.size(); t < pathB.size(); t++) {
                if (targetA == pathB[t]) {
                  setOfConflicts.insert({groupA, groupB, t});
                }
              }
              // Edge conflict
              for (int t = 0; t < pathA.size() - 1; t++) {
                // We only add an edge conflict if 2 agents are traversing the
                // edge with different directions The other case is already
                // token into account with 2 vertex conflicts
                if (pathA[t] == pathB[t + 1] and pathA[t + 1] == pathB[t]) {
                  setOfConflicts.insert({groupA, groupB, t + 1});
                }
              }
            }
          }
        }
      }
    }
  }
}

template <class MultiAgentSolver>
bool SimpleIndependenceDetection<MultiAgentSolver>::mergeGroupsAndPlanNewGroup(
    std::shared_ptr<Group> groupA, std::shared_ptr<Group> groupB) {
  groups.erase(groupA);
  groups.erase(groupB);
  std::set<int> agents(groupA->getAgents());
  for (int i : groupB->getAgents()) {
    agents.insert(i);
  }
  auto newGroup = std::make_shared<Group>(agents);
  groups.insert(newGroup);
  vector<int> starts;
  vector<int> targets;
  vector<int> agentIds;
  for (int agentId : newGroup->getAgents()) {
    starts.push_back(problem->getStartOf(agentId));
    targets.push_back(problem->getTargetOf(agentId));
    agentIds.push_back(agentId);
  }
  if (CAT) {
    auto setOfAgentsToReplan = newGroup->getAgents();
    auto it = vertexConflictAvoidanceTable.begin();
    while (it != vertexConflictAvoidanceTable.end()) {
      if (setOfAgentsToReplan.find(it->getAgent()) !=
          setOfAgentsToReplan.end()) {
        it = vertexConflictAvoidanceTable.erase(it);
      } else {
        ++it;
      }
    }
    auto it2 = edgeConflictAvoidanceTable.begin();
    while (it2 != edgeConflictAvoidanceTable.end()) {
      if (setOfAgentsToReplan.find(it2->getAgent()) !=
          setOfAgentsToReplan.end()) {
        it2 = edgeConflictAvoidanceTable.erase(it2);
      } else {
        ++it2;
      }
    }
  }
  auto prob = std::make_shared<MultiAgentProblem>(
      problem->getGraph(), starts, targets, problem->getObjFunction(), agentIds,
      problem->getSetOfHardVertexConstraints(),
      problem->getSetOfHardEdgeConstraints(), INT_MAX,
      vertexConflictAvoidanceTable, edgeConflictAvoidanceTable);
  auto solution = lowLevelSearch->solve(prob);
  if (not solution->getFoundPath()) {
    return false;
  }
  newGroup->putSolution(solution);
  if (CAT) {
    for (auto [agentId, pathOfAgent] : solution->getPositions()) {
      for (int t = 0; t < pathOfAgent.size(); t++) {
        vertexConflictAvoidanceTable.insert({agentId, pathOfAgent[t], t});
      }
      for (int t = 1; t < pathOfAgent.size(); t++) {
        edgeConflictAvoidanceTable.insert(
            {agentId, pathOfAgent[t], pathOfAgent[t - 1], t});
        // edgeConflictAvoidanceTable.insert({agentId, pathOfAgent[t-1],
        // pathOfAgent[t], t});
      }
    }
  }

  // update the set of conflicts
  std::set<GroupConflict> newSetOfConflicts;
  for (const auto &conflict : setOfConflicts) {
    if (*conflict.getGroupA() == *groupA or *conflict.getGroupB() == *groupA) {
      continue;
    }
    if (*conflict.getGroupA() == *groupB or *conflict.getGroupB() == *groupB) {
      continue;
    }
    newSetOfConflicts.insert(conflict);
  }
  setOfConflicts = newSetOfConflicts;
  for (const auto &group : groups) {
    if (*group != *newGroup) {
      for (auto [agentA, pathA] : group->getSolution()->getPositions()) {
        for (auto [agentB, pathB] : newGroup->getSolution()->getPositions()) {
          // Vertex conflict
          if (pathA.size() < pathB.size()) {
            for (int t = 0; t < pathA.size(); t++) {
              if (pathA[t] == pathB[t]) {
                setOfConflicts.insert({group, newGroup, t});
              }
            }
            int targetA = problem->getTargetOf(agentA);
            for (int t = pathA.size(); t < pathB.size(); t++) {
              if (targetA == pathB[t]) {
                setOfConflicts.insert({group, newGroup, t});
              }
            }
          } else {
            for (int t = 0; t < pathB.size(); t++) {
              if (pathA[t] == pathB[t]) {
                setOfConflicts.insert({group, newGroup, t});
              }
            }
            int targetB = problem->getTargetOf(agentB);
            for (int t = pathB.size(); t < pathA.size(); t++) {
              if (pathA[t] == targetB) {
                setOfConflicts.insert({group, newGroup, t});
              }
            }
          }
          // Edge conflict
          for (int t = 0; t < std::min(pathA.size(), pathB.size()) - 1; t++) {
            if (pathA[t] == pathB[t + 1] and pathA[t + 1] == pathB[t]) {
              setOfConflicts.insert({group, newGroup, t + 1});
            }
          }
        }
      }
    }
  }
  return true;
}

template <class MultiAgentSolver>
std::shared_ptr<Solution>
SimpleIndependenceDetection<MultiAgentSolver>::combineSolutions() {
  int numberOfTimesteps = 0;
  for (const std::shared_ptr<Group> &group : groups) {
    for (int agent : group->getAgents()) {
      numberOfTimesteps =
          std::max((int)group->getSolution()->getPositions()[agent].size(),
                   numberOfTimesteps);
    }
  }
  for (const std::shared_ptr<Group> &group : groups) {
    group->getSolution()->lengthenPositions(numberOfTimesteps);
  }
  std::unordered_map<int, vector<int>> positions;
  vector<int> ids = problem->getAgentIds();
  for (const std::shared_ptr<Group> &group : groups) {
    for (int agentId : group->getAgents()) {
      positions[agentId] = group->getSolution()->getPathOfAgent(agentId);
    }
  }
  for (const auto &group : groups) {
    sizeOfLargerGroup =
        std::max(sizeOfLargerGroup, (int)group->getAgents().size());
  }
  auto solution =
      std::make_shared<Solution>(numberOfTimesteps, positions,
                                 numberOfResolvedConflicts, sizeOfLargerGroup);
  if (problem->getMaxCost() != INT_MAX) {
    if (problem->getObjFunction() == Makespan) {
      if (solution->getMakespanCost() > problem->getMaxCost()) {
        return std::make_shared<Solution>();
      }
    } else if (problem->getObjFunction() == SumOfCosts) {
      if (solution->getSumOfCostsCost() > problem->getMaxCost()) {
        return std::make_shared<Solution>();
      }
    } else {
      if (solution->getFuelCost() > problem->getMaxCost()) {
        return std::make_shared<Solution>();
      }
    }
  }
  return solution;
}

template <class MultiAgentSolver>
std::shared_ptr<Solution>
SimpleIndependenceDetection<MultiAgentSolver>::solve() {

  LOG("===== Simple Independent Detection Search (with set of conflicts) ====");

  if (problem == nullptr) {
    return std::make_shared<Solution>();
  }

  if (problem->isImpossible()) {
    return std::make_shared<Solution>();
  }

  // Assign each agent to a singleton group
  for (int agent : problem->getAgentIds()) {
    groups.insert(std::make_shared<Group>(std::set<int>{agent}));
  }

  // Plan a path for each group
  if (not planSingletonGroups()) {
    return std::make_shared<Solution>();
  }
  calculateSetOfConflicts();

  while (not setOfConflicts.empty()) {
    auto conflict = setOfConflicts.begin();

    if (not mergeGroupsAndPlanNewGroup(conflict->getGroupA(),
                                       conflict->getGroupB())) {
      return std::make_shared<Solution>();
    }
    numberOfResolvedConflicts += 1;
  }
  return combineSolutions();
}

template class SimpleIndependenceDetection<GeneralAStar>;
template class SimpleIndependenceDetection<ConflictBasedSearch>;
