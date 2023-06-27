//
// Created by Arthur Mahy on 13/02/2023.
//

#ifndef TFE_MAPF_SINGLEAGENTPROBLEM_H
#define TFE_MAPF_SINGLEAGENTPROBLEM_H

#include "Problem.h"

class SingleAgentProblem : public Problem {
public:

    // Simplest single-agent pathfinding problem
    SingleAgentProblem(const std::shared_ptr<Graph>& graph, int start, int target, int agentId = 0, int maxCost = INT_MAX);

    // Single-agent pathfinding with constraints
    SingleAgentProblem(const std::shared_ptr<Graph>& graph, int start, int target, ObjectiveFunction objective,
                                      int agentId = 0, const HardVertexConstraintsSet &setOfHardVertexConstraints = HardVertexConstraintsSet(),
                                      const HardEdgeConstraintsSet &setOfHardEdgeConstraints = HardEdgeConstraintsSet(), int maxCost = INT_MAX,
                                      const SoftVertexConstraintsMultiSet& setOfSoftVertexConstraints = SoftVertexConstraintsMultiSet(),
                                      const SoftEdgeConstraintsMultiSet& setOfSoftEdgeConstraints = SoftEdgeConstraintsMultiSet(), int startTime = 0);

    std::shared_ptr<Graph> getGraph() const;
    int getStart() const;
    int getTarget() const;
    int getAgentId() const;
    int getMaxCost() const;
    void print() const;

    int getNumberOfAgents() const override;
    bool isImpossible() const override;
    bool hasExternalConstraints() const override;

    ObjectiveFunction getObjFunction();
    HardVertexConstraintsSet getSetOfHardVertexConstraints() const;
    HardEdgeConstraintsSet getSetOfHardEdgeConstraints() const;
    SoftVertexConstraintsMultiSet getSetOfSoftVertexConstraints() const;
    SoftEdgeConstraintsMultiSet getSetOfSoftEdgeConstraints() const;
    int getStartTime() const;

    // Returns true if the agent is allowed to go from position to newPosition between time-1 and time
    // (according to the hard vertex constraints and the hard edge constraints of the problem)
    bool okForConstraints(int position, int newPosition, int time) const;

    // Returns true if the agent is allowed to be at newPosition at time
    // (according to the hard vertex constraints of the problem)
    bool okForConstraints(int newPosition, int time) const;

    // The number of violated soft constraints if agent go from position to newPosition between time-1 and time
    // (according to the soft vertex constraints and the soft edge constraints of the problem)
    int numberOfViolations(int position, int newPosition, int time) const;

    // The number of violated soft constraints if agent is at newPosition at time
    // (according to the soft vertex constraints of the problem)
    int numberOfViolations(int newPosition, int time) const;

private:

    // Graph with the possible positions and transitions for the agent
    std::shared_ptr<Graph> graph;

    // Start position of the agent
    int start;

    // Target position of the agent
    int target;

    // Id of the agent
    int agentId;

    // The solution of this problem must have a cost inferior or equal to maxCost
    int maxCost;

    // true if the start or the target position is unreachable
    bool impossible;

    // true if the problem has external constraints
    bool externalConstraints;

    // The objective function to minimize : Fuel or Makespan
    // - Fuel : Total amount of distance traveled by the agent (costWait = 0)
    // - Makespan (or SumOfCosts) : Total time for the agent to reach its goal (costWait = 1)
    ObjectiveFunction objective;

    // Hard constraints like (a, p, t) meaning agent a can't be at position p at time t
    // When planning agent agentId, the agent won't go at position p at time t if a==agentId
    //
    // Set of hard vertex constraints
    HardVertexConstraintsSet setOfHardVertexConstraints;
    // Set of hard edge constraints
    HardEdgeConstraintsSet setOfHardEdgeConstraints;

    // Soft constraints like (a, p, t) meaning agent a is occupying position p at time t
    // When planning agent agentId, the agent will try to avoid position p at time t if a!=agentId
    //
    // Set of soft vertex constraints
    SoftVertexConstraintsMultiSet setOfSoftVertexConstraints;
    // Set of soft edge constraints
    SoftEdgeConstraintsMultiSet setOfSoftEdgeConstraints;

    // Start time of the path. Agent is at position start at time startTime.
    int startTime;
};

#endif //TFE_MAPF_SINGLEAGENTPROBLEM_H
