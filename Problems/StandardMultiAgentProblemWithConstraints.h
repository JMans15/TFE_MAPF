//
// Created by Arthur Mahy on 23/05/2023.
//

#ifndef TFE_MAPF_STANDARDMULTIAGENTPROBLEMWITHCONSTRAINTS_H
#define TFE_MAPF_STANDARDMULTIAGENTPROBLEMWITHCONSTRAINTS_H

#include "Problem.h"
#include "../States/StandardMultiAgentState.h"

#include <set>

// Multi Agent Problem solved with Standard A* (>< Operator Decomposition A*)
class StandardMultiAgentProblemWithConstraints : public Problem<StandardMultiAgentState> {
public:
    StandardMultiAgentProblemWithConstraints(const std::shared_ptr<Graph>& graph, std::vector<int> starts, std::vector<int> targets,
                                     ObjectiveFunction objective = Fuel, const std::vector<int>& agentIds = std::vector<int>(),
                                     const std::set<VertexConstraint> &setOfHardVertexConstraints = std::set<VertexConstraint>(),
                                     const std::set<EdgeConstraint> &setOfHardEdgeConstraints = std::set<EdgeConstraint>(), int maxCost = INT_MAX,
                                     const SoftVertexConstraintsMultiSet& setOfSoftVertexConstraints = SoftVertexConstraintsMultiSet(),
                                     const SoftEdgeConstraintsMultiSet& setOfSoftEdgeConstraints = SoftEdgeConstraintsMultiSet(), int startTime = 0);

    std::shared_ptr<StandardMultiAgentState> getStartState() const override;
    bool isGoalState(std::shared_ptr<StandardMultiAgentState> state) const override;
    std::vector<std::tuple<std::shared_ptr<StandardMultiAgentState>, int, int>> getSuccessors(std::shared_ptr<StandardMultiAgentState> state) const override;
    std::unordered_map<int, std::vector<int>> getPositions(std::vector<std::shared_ptr<StandardMultiAgentState>> states) const override;
    std::vector<int> getAgentIds() const override;
    bool isImpossible() const override;

    const std::vector<int>& getStarts() const;
    const std::vector<int>& getTargets() const;
    ObjectiveFunction getObjFunction();
    std::set<VertexConstraint> getSetOfHardVertexConstraints() const;
    std::set<EdgeConstraint> getSetOfHardEdgeConstraints() const;
    SoftVertexConstraintsMultiSet getSetOfSoftVertexConstraints() const;
    SoftEdgeConstraintsMultiSet getSetOfSoftEdgeConstraints() const;

    int getStartOf(int id);
    int getTargetOf(int id);

private:
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
    std::set<VertexConstraint> setOfHardVertexConstraints;
    // Set of hard edge constraints
    std::set<EdgeConstraint> setOfHardEdgeConstraints;

    // Set of soft vertex constraints like (a, p, t) meaning agent a is occupying position p at time t
    // This problem will try to avoid these position-timestep points. So, it's better to not put any (a,p,t) constraint when planning agent a.
    SoftVertexConstraintsMultiSet setOfSoftVertexConstraints;
    // Set of soft edge constraints
    SoftEdgeConstraintsMultiSet setOfSoftEdgeConstraints;

    // Returns true if the agent is allowed to go from position to newPosition between time-1 and time
    // (according to the hard vertex constraints and the hard edge constraints of the problem)
    bool okForConstraints(int agent, int position, int newPosition, int time) const;

    // Returns true if the agent is allowed to be at newPosition at time
    // (according to the hard vertex constraints of the problem)
    bool okForConstraints(int agent, int newPosition, int time) const;

    // The number of violated soft constraints if agent go from position to newPosition between time-1 and time
    // (according to the soft vertex constraints and the soft edge constraints of the problem)
    int numberOfViolations(int position, int newPosition, int time) const;

    // The number of violated soft constraints if agent is at newPosition at time
    // (according to the soft vertex constraints of the problem)
    int numberOfViolations(int newPosition, int time) const;

    // Returns true if position is not already occupied by assigned agents
    bool notAlreadyOccupiedPosition(int position, std::vector<int> &positions, int agentToAssign) const;

    // Returns true if the edge (position, positions[agentToAssign]) is not already occupied by assigned agents
    bool notAlreadyOccupiedEdge(int position, const std::vector<int> &positions, int agentToAssign, const std::vector<int> &prePositions) const;

    // Recursive function used in the getSuccessors(state) method
    // Branches on all possibles moves for agentToAssign (from 0 to numberOfAgents-1)
    // If agentToAssign is the last agent, we add a successor to the list of successors for every possible move for agentToAssign
    // If agentToAssign isn't the last agent, we call recursiveAssignAMoveToAnAgent for every possible move for agentToAssign
    //
    // positions[:agentToAssign] are the assigned positions
    // positions[agentToAssign:] are the not yet assigned positions
    // positions[agentToAssign] is not yet assigned but will be in this function
    //
    // prePositions is the positions of the agents in state
    // t is the timestep in state (when we add a successor to the list of successors, its timestep is t+1)
    // cost and violations are the cost and the number of violated soft constraints by assigning the agents from 0 to agentToAssign
    // cannotMove is the list of agents which are at their target positions and cannot move anymore (for the SumOfCosts objective function)
    void recursiveAssignAMoveToAnAgent(int agentToAssign, std::vector<std::tuple<std::shared_ptr<StandardMultiAgentState>, int, int>>* successors, int cost, std::vector<int> positions, const std::vector<int>& prePositions, int t, int violations, std::vector<int> cannotMove = std::vector<int>()) const ;

};


#endif //TFE_MAPF_STANDARDMULTIAGENTPROBLEMWITHCONSTRAINTS_H
