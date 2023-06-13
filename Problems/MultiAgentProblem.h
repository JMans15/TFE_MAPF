//
// Created by Arthur Mahy on 04/01/2023.
//

#ifndef TFE_MAPF_MULTIAGENTPROBLEM_H
#define TFE_MAPF_MULTIAGENTPROBLEM_H

#include "Problem.h"

// Multi Agent Problem
class MultiAgentProblem : public Problem{
public:
    MultiAgentProblem(const std::shared_ptr<Graph>& graph, std::vector<int> starts, std::vector<int> targets,
                                     ObjectiveFunction objective = Fuel, const std::vector<int>& agentIds = std::vector<int>(),
                                     const HardVertexConstraintsSet& setOfHardVertexConstraints = HardVertexConstraintsSet(),
                                     const HardEdgeConstraintsSet& setOfHardEdgeConstraints = HardEdgeConstraintsSet(), int maxCost = INT_MAX,
                                     const SoftVertexConstraintsMultiSet& setOfSoftVertexConstraints = SoftVertexConstraintsMultiSet(),
                                     const SoftEdgeConstraintsMultiSet& setOfSoftEdgeConstraints = SoftEdgeConstraintsMultiSet(), int startTime = 0);

    std::vector<int> getAgentIds() const;

    const std::vector<int>& getStarts() const;
    const std::vector<int>& getTargets() const;
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

    int getStartOf(int id);
    int getTargetOf(int id);

    // Returns true if the agent is allowed to go from position to newPosition between time-1 and time
    // (according to the hard vertex constraints and the hard edge constraints of the problem)
    bool okForConstraints(int agent, int position, int newPosition, int time) const;

    // Returns true if the agent is allowed to be at newPosition at time
    // (according to the hard vertex constraints of the problem)
    bool okForConstraints(int agent, int newPosition, int time) const;

    // The number of violated soft constraints if agent go from position to newPosition between time-1 and time
    // (according to the soft vertex constraints and the soft edge constraints of the problem)
    int numberOfViolations(int agent, int position, int newPosition, int time) const;

    // The number of violated soft constraints if agent is at newPosition at time
    // (according to the soft vertex constraints of the problem)
    int numberOfViolations(int agent, int newPosition, int time) const;

private:

    // Graph with the possible positions and transitions for the agents
    std::shared_ptr<Graph> graph;

    int numberOfAgents;

    // starts is a list of length numberOfAgents with the start position of each agent
    std::vector<int> starts;

    // target is a list of length numberOfAgents with the target position of each agent
    std::vector<int> targets;

    std::vector<int> agentIds; // Index to id
    std::unordered_map<int, int> idToIndex; // Id to index

    // true if a start or a target position is unreachable
    // or if 2 agents have the same start position or the same target position (or the same id)
    // or if the starts, targets and agentIds vectors doesn't have the same size
    bool impossible;

    // The objective function to minimize : Fuel or Makespan or SumOfCosts
    // - Fuel : Total amount of distance traveled by all agents
    // - Makespan : Total time for the last agent to reach its goal
    // - SumOfCosts : The sum of the time steps required for every agent to reach its goal and never leave it again
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


#endif //TFE_MAPF_MULTIAGENTPROBLEM_H
