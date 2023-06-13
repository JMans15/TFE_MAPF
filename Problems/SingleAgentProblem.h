//
// Created by Arthur Mahy on 13/02/2023.
//

#ifndef TFE_MAPF_SINGLEAGENTPROBLEM_H
#define TFE_MAPF_SINGLEAGENTPROBLEM_H

#include "Problem.h"

class SingleAgentProblem : public Problem {
public:
    SingleAgentProblem(std::shared_ptr<Graph> graph, int start, int target, ObjectiveFunction objective = Fuel,
                                      int agentId = 0, const HardVertexConstraintsSet &setOfHardVertexConstraints = HardVertexConstraintsSet(),
                                      const HardEdgeConstraintsSet &setOfHardEdgeConstraints = HardEdgeConstraintsSet(), int maxCost = INT_MAX,
                                      const SoftVertexConstraintsMultiSet& setOfSoftVertexConstraints = SoftVertexConstraintsMultiSet(),
                                      const SoftEdgeConstraintsMultiSet& setOfSoftEdgeConstraints = SoftEdgeConstraintsMultiSet(), int startTime = 0);

    SingleAgentProblem(std::shared_ptr<Graph> graph, int start, int target, int agentId, int maxCost);

    int getAgentId() const;

    const int getStart() const;
    const int getTarget() const;
    ObjectiveFunction getObjFunction();
    HardVertexConstraintsSet getSetOfHardVertexConstraints() const;
    HardEdgeConstraintsSet getSetOfHardEdgeConstraints() const;
    SoftVertexConstraintsMultiSet getSetOfSoftVertexConstraints() const;
    SoftEdgeConstraintsMultiSet getSetOfSoftEdgeConstraints() const;
    std::shared_ptr<Graph> getGraph() const;
    int getNumberOfAgents() const;
    int getMaxCost() const;
    int getStartTime() const;
    bool hasTimeConstraints() const;
    bool isMultiAgentProblem() const;
    bool isImpossible() const;

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

    // Graph with the possible positions and transitions for the agents
    std::shared_ptr<Graph> graph;

    int numberOfAgents;

    // start position of the agent
    int start;

    // target position of the agent
    int target;

    int agentId;

    // true if the start or the target position is unreachable
    bool impossible;

    // The objective function to minimize : Fuel or Makespan
    // - Fuel : Total amount of distance traveled by the agent (costWait = 0)
    // - Makespan or SumOfCosts : Total time for the agent to reach its goal (costWait = 1)
    ObjectiveFunction objective;

    // Set of hard vertex constraints like (a, p, t) meaning agent a can't be at position p at time t
    HardVertexConstraintsSet setOfHardVertexConstraints;
    // Set of hard edge constraints
    HardEdgeConstraintsSet setOfHardEdgeConstraints;

    // Set of soft vertex constraints like (a, p, t) meaning agent a is occupying position p at time t
    // We ignore constraints from agent a when planning agent a
    SoftVertexConstraintsMultiSet setOfSoftVertexConstraints;
    // Set of soft edge constraints
    SoftEdgeConstraintsMultiSet setOfSoftEdgeConstraints;

    // The solution of this problem must have a cost inferior or equal to maxCost
    int maxCost;

    int startTime;
};

#endif //TFE_MAPF_SINGLEAGENTPROBLEM_H
