//
// Created by Arthur Mahy on 16/05/2023.
//

#include "IndependenceDetection.h"

template <class MultiAgentSolver>
IndependenceDetection<MultiAgentSolver>::IndependenceDetection(
    std::shared_ptr<MultiAgentProblem> problem,
    std::shared_ptr<MultiAgentSolver> lowLevelSearch, bool CAT)
    : SimpleIndependenceDetection<MultiAgentSolver>(problem, lowLevelSearch,
                                                    CAT) {}

template <class MultiAgentSolver>
IndependenceDetection<MultiAgentSolver>::IndependenceDetection(
    std::shared_ptr<MultiAgentSolver> lowLevelSearch, bool CAT)
    : SimpleIndependenceDetection<MultiAgentSolver>(nullptr, lowLevelSearch,
                                                    CAT) {}

template <class MultiAgentSolver>
std::shared_ptr<Solution> IndependenceDetection<MultiAgentSolver>::solve(
    std::shared_ptr<MultiAgentProblem> m_problem) {
  problem = std::move(m_problem);
  groups =
      std::unordered_set<std::shared_ptr<Group>, GroupHasher, GroupEquality>();
  setOfConflicts = std::set<GroupConflict>();
  vertexConflictAvoidanceTable = problem->getSetOfSoftVertexConstraints();
  edgeConflictAvoidanceTable = problem->getSetOfSoftEdgeConstraints();
  numberOfResolvedConflicts = 0;
  alreadyConflictedBefore =
      std::unordered_set<std::set<std::shared_ptr<Group>, PointerGroupEquality>,
                         SetOfPointersHasher, SetOfPointersEquality>();
  return solve();
}

template <class MultiAgentSolver>
bool IndependenceDetection<MultiAgentSolver>::replanGroupAAvoidingGroupB(
    const std::shared_ptr<Group> &groupA,
    const std::shared_ptr<Group> &groupB) {
  HardVertexConstraintsSet vertexIllegalTable =
      problem->getSetOfHardVertexConstraints();
  HardEdgeConstraintsSet edgeIllegalTable =
      problem->getSetOfHardEdgeConstraints();
  for (auto a : groupB->getSolution()->getPositions()) {
    vector<int> pathOfAgent = a.second;
    for (int agentA : groupA->getAgents()) {
      for (int t = 0; t < pathOfAgent.size(); t++) {
        // agent cannot go at position pathofagent[t] at time t
        vertexIllegalTable.insert({agentA, pathOfAgent[t], t});
      }
      for (int t = 1; t < pathOfAgent.size(); t++) {
        // to avoid edge conflict
        edgeIllegalTable.insert(
            {agentA, pathOfAgent[t], pathOfAgent[t - 1], t});
      }
    }
  }
  vector<int> starts;
  vector<int> targets;
  vector<int> agentIds;
  for (int agentId : groupA->getAgents()) {
    starts.push_back(problem->getStartOf(agentId));
    targets.push_back(problem->getTargetOf(agentId));
    agentIds.push_back(agentId);
  }
  std::shared_ptr<Solution> solution;
  if (groupA->getAgents().size() == 1) {
    auto prob = std::make_shared<SingleAgentProblem>(
        problem->getGraph(), starts[0], targets[0], problem->getObjFunction(),
        agentIds[0], vertexIllegalTable, edgeIllegalTable,
        groupA->getSolution()->getCost(), vertexConflictAvoidanceTable,
        edgeConflictAvoidanceTable);
    solution =
        AStar<SingleAgentAStarProblemWithConstraints,
              SingleAgentSpaceTimeState>(
            std::make_shared<SingleAgentAStarProblemWithConstraints>(prob),
            OptimalDistance)
            .solve();
  } else {
    auto prob = std::make_shared<MultiAgentProblem>(
        problem->getGraph(), starts, targets, problem->getObjFunction(),
        agentIds, vertexIllegalTable, edgeIllegalTable,
        groupA->getSolution()->getCost(), vertexConflictAvoidanceTable,
        edgeConflictAvoidanceTable);
    solution = lowLevelSearch->solve(prob);
  }
  if (solution->getFoundPath()) {
    groupA->putSolution(solution);
    if (CAT) {
      auto setOfAgentsToReplan = groupA->getAgents();
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
      if (*conflict.getGroupA() == *groupA or
          *conflict.getGroupB() == *groupA) {
        continue;
      }
      newSetOfConflicts.insert(conflict);
    }
    setOfConflicts = newSetOfConflicts;
    for (const auto &group : groups) {
      if (*group != *groupA) {
        for (auto [agent, path] : group->getSolution()->getPositions()) {
          for (auto [agentA, pathA] : groupA->getSolution()->getPositions()) {
            // Vertex conflict
            if (pathA.size() < path.size()) {
              for (int t = 0; t < pathA.size(); t++) {
                if (pathA[t] == path[t]) {
                  setOfConflicts.insert({group, groupA, t});
                }
              }
              int targetA = problem->getTargetOf(agentA);
              for (int t = pathA.size(); t < path.size(); t++) {
                if (targetA == path[t]) {
                  setOfConflicts.insert({group, groupA, t});
                }
              }
            } else {
              for (int t = 0; t < path.size(); t++) {
                if (pathA[t] == path[t]) {
                  setOfConflicts.insert({group, groupA, t});
                }
              }
              int target = problem->getTargetOf(agent);
              for (int t = path.size(); t < pathA.size(); t++) {
                if (pathA[t] == target) {
                  setOfConflicts.insert({group, groupA, t});
                }
              }
            }
            // Edge conflict
            for (int t = 0; t < std::min(path.size(), pathA.size()) - 1; t++) {
              if (path[t] == pathA[t + 1] and path[t + 1] == pathA[t]) {
                setOfConflicts.insert({group, groupA, t + 1});
              }
            }
          }
        }
      }
    }

    return true;
  }
  return false;
}

template <class MultiAgentSolver>
std::shared_ptr<Solution> IndependenceDetection<MultiAgentSolver>::solve() {

  LOG("===== Independent Detection Search (with set of conflicts) ====");

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

  /*std::cout << "The groups are the following:" << std::endl;
  for (auto group : groups){
      std::cout << "group:" << std::endl;
      for (auto agent : group->getAgents()){
          std::cout << agent << std::endl;
      }
  }*/

  calculateSetOfConflicts();

  while (not setOfConflicts.empty()) {
    auto conflict = setOfConflicts.begin();

    auto groupA = conflict->getGroupA();
    auto groupB = conflict->getGroupB();

    /*std::cout << "These 2 groups are in conflict : " << std::endl;
    std::cout << "groupA:" << std::endl;
    for (auto agent : groupA->getAgents()){
        std::cout << agent << std::endl;
    }
    std::cout << "groupB:" << std::endl;
    for (auto agent : groupB->getAgents()){
        std::cout << agent << std::endl;
    }*/

    if (alreadyConflictedBefore.count({groupA, groupB}) == 0) {
      alreadyConflictedBefore.insert({groupA, groupB});
      if (not replanGroupAAvoidingGroupB(groupA, groupB)) {
        if (not replanGroupAAvoidingGroupB(groupB, groupA)) {
          if (not mergeGroupsAndPlanNewGroup(groupA, groupB)) {
            return std::make_shared<Solution>();
          }
          // std::cout << "We couldn't find an alternate optimal solution for
          // group A and group B -> we merge." << std::endl;
        } else {
          /*std::cout << "We replanned this group : " << std::endl;
          std::cout << "group:" << std::endl;
          for (auto agent : groupB->getAgents()){
              std::cout << agent << std::endl;
          }*/
        }
      } else {
        /*std::cout << "We replanned this group : " << std::endl;
        std::cout << "group:" << std::endl;
        for (auto agent : groupA->getAgents()){
            std::cout << agent << std::endl;
        }*/
      }
    } else {
      // std::cout << "Group A and group B already conflicted before -> we
      // merge." << std::endl;
      if (not mergeGroupsAndPlanNewGroup(groupA, groupB)) {
        return std::make_shared<Solution>();
      }
    }
    numberOfResolvedConflicts += 1;

    /*std::cout << "The groups are the following:" << std::endl;
    for (auto group : groups){
        std::cout << "group:" << std::endl;
        for (auto agent : group->getAgents()){
            std::cout << agent << std::endl;
        }
    }*/
  }
  return combineSolutions();
}

template class IndependenceDetection<GeneralAStar>;
template class IndependenceDetection<ConflictBasedSearch>;
