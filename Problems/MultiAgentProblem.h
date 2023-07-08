//
// Created by Arthur Mahy on 04/01/2023.
//

#ifndef TFE_MAPF_MULTIAGENTPROBLEM_H
#define TFE_MAPF_MULTIAGENTPROBLEM_H

#include "Problem.h"

class MultiAgentProblem : public Problem {
public:
  // Multi-agent pathfinding with external constraints
  MultiAgentProblem(
      const std::shared_ptr<Graph> &graph, std::vector<int> starts,
      std::vector<int> targets, ObjectiveFunction objective,
      const std::vector<int> &agentIds = std::vector<int>(),
      const HardVertexConstraintsSet &setOfHardVertexConstraints =
          HardVertexConstraintsSet(),
      const HardEdgeConstraintsSet &setOfHardEdgeConstraints =
          HardEdgeConstraintsSet(),
      int maxCost = INT_MAX,
      const SoftVertexConstraintsMultiSet &setOfSoftVertexConstraints =
          SoftVertexConstraintsMultiSet(),
      const SoftEdgeConstraintsMultiSet &setOfSoftEdgeConstraints =
          SoftEdgeConstraintsMultiSet(),
      int startTime = 0);

  std::shared_ptr<Graph> getGraph() const;
  std::vector<int> getStarts() const;
  std::vector<int> getTargets() const;
  int getStartOf(int id);
  int getTargetOf(int id);
  ObjectiveFunction getObjFunction();
  std::vector<int> getAgentIds() const;
  int getMaxCost() const;
  void print() const;

  int getNumberOfAgents() const override;
  bool isImpossible() const override;
  bool hasExternalConstraints() const override;

  HardVertexConstraintsSet getSetOfHardVertexConstraints() const;
  HardEdgeConstraintsSet getSetOfHardEdgeConstraints() const;
  SoftVertexConstraintsMultiSet getSetOfSoftVertexConstraints() const;
  SoftEdgeConstraintsMultiSet getSetOfSoftEdgeConstraints() const;
  int getStartTime() const;

  // Returns true if the agent is allowed to go from position to newPosition
  // between time-1 and time (according to the hard vertex constraints and the
  // hard edge constraints of the problem)
  bool okForConstraints(int agent, int position, int newPosition,
                        int time) const;

  // Returns true if the agent is allowed to be at newPosition at time
  // (according to the hard vertex constraints of the problem)
  bool okForConstraints(int agent, int newPosition, int time) const;

  // The number of violated soft constraints if agent go from position to
  // newPosition between time-1 and time (according to the soft vertex
  // constraints and the soft edge constraints of the problem)
  int numberOfViolations(int agent, int position, int newPosition,
                         int time) const;

  // The number of violated soft constraints if agent is at newPosition at time
  // (according to the soft vertex constraints of the problem)
  int numberOfViolations(int agent, int newPosition, int time) const;

private:
  // Graph with the possible positions and transitions for the agents
  std::shared_ptr<Graph> graph;

  // Number of agents in this problem
  int numberOfAgents;

  // starts is a list of length numberOfAgents with the start position of each
  // agent starts[index] is the start position of the agent at index
  std::vector<int> starts;

  // target is a list of length numberOfAgents with the target position of each
  // agent targets[index] is the target position of the agent at index
  std::vector<int> targets;

  // The objective function to minimize : Fuel or Makespan or SumOfCosts
  // - Fuel : Total amount of distance traveled by all agents
  // - Makespan : Total time for the last agent to reach its goal
  // - SumOfCosts : The sum of the time steps required for every agent to reach
  // its goal and remain at the goal position without leaving it again
  ObjectiveFunction objective;

  // agentIds[index] is the id of the agent at index
  std::vector<int> agentIds;              // Index to id
  std::unordered_map<int, int> idToIndex; // Id to index

  // The solution of this problem must have a cost inferior or equal to maxCost
  int maxCost;

  // true if a start or a target position is unreachable
  // or if 2 agents have the same start position or the same target position (or
  // the same id) or if the starts, targets and agentIds vectors doesn't have
  // the same size
  bool impossible;

  // true if the problem has external constraints
  bool externalConstraints;

  // Hard constraints like (a, p, t) meaning agent a can't be at position p at
  // time t When planning agent agentId, the agent won't go at position p at
  // time t if a==agentId
  //
  // Set of hard vertex constraints
  HardVertexConstraintsSet setOfHardVertexConstraints;
  // Set of hard edge constraints
  HardEdgeConstraintsSet setOfHardEdgeConstraints;

  // Soft constraints like (a, p, t) meaning agent a is occupying position p at
  // time t When planning agent agentId, the agent will try to avoid position p
  // at time t if a!=agentId
  //
  // Set of soft vertex constraints
  SoftVertexConstraintsMultiSet setOfSoftVertexConstraints;
  // Set of soft edge constraints
  SoftEdgeConstraintsMultiSet setOfSoftEdgeConstraints;

  // Start time of the paths. Agent is at position start at time startTime.
  int startTime;
};

#endif // TFE_MAPF_MULTIAGENTPROBLEM_H
