//! Specific problem class for multi-agent problems

#ifndef TFE_MAPF_MULTIAGENTPROBLEM_H
#define TFE_MAPF_MULTIAGENTPROBLEM_H

#include "Problem.h"

class MultiAgentProblem : public Problem {
public:
  /**
   * Constructor for a multi-agent problem
   * It can handle hard and soft constraints, soft constraints being flexible
   * and hard constraints being enforced
   * @param [in] &graph A pointer to the graph corresponding to the map on which
   * the problem takes place
   * @param [in] starts A vector containing the indices of the nodes where the
   * starting points of each agent is
   * @param [in] targets A vector containing the indices of the nodes where the
   * target of each agent is
   * @param [in] objective A member of enum ObjectiveFunction specifying what
   * objective function to use
   * @param [in] &agentsIds the ids of the agents that are part of the problem,
   * useful e.g. for ID
   * @param [in] &setOfHardVertexConstraints The set of hard vertex constraints
   * @param [in] &setOfHardEdgeConstraints The set of hard edge constraints
   * @param [in] maxCost An upper bound of the cost, useful for EID
   * @param [in] &setOfSoftVertexConstraints The set of soft vertex constraints
   * @param [in] &setOfSoftEdgeConstraints The set of soft edge constraints
   * @param [in] startTime The time at which the problem started
   */
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

  std::shared_ptr<Graph>
  getGraph() const; /**< Getter for the graph corresponding to the map on which
                       the problem takes place */
  std::vector<int>
  getStarts() const; /**< Getter for the vector containing the indices of the
                        nodes where the starting points of each agent is */
  std::vector<int>
  getTargets() const; /**< Getter for the vector containing the indices of the
                         nodes where the target points of each agent is */

  int getStartOf(int id);  /**< Getter for the starting point of agent with
                              specific id @param id of the agent */
  int getTargetOf(int id); /**< Getter for the target point of agent with
                              specific id @param id of the agent */
  ObjectiveFunction
  getObjFunction(); /**< Getter for the used objective function */
  std::vector<int>
  getAgentIds() const; /**< Getter for the vector of concerned agents id */
  int getMaxCost()
      const; /**< Getter for the upper bound on the solution cost */
  void
  print() const; /**< Print function to represent the problem as a string */

  int getNumberOfAgents() const override;
  bool isImpossible() const override;
  bool hasExternalConstraints() const override;

  HardVertexConstraintsSet getSetOfHardVertexConstraints()
      const; /**< Getter for the set of harde vertex constraints */
  HardEdgeConstraintsSet getSetOfHardEdgeConstraints()
      const; /**< Getter for the set of hard edge constraints */
  SoftVertexConstraintsMultiSet getSetOfSoftVertexConstraints()
      const; /**< Getter for the set of soft vertex constraints */
  SoftEdgeConstraintsMultiSet getSetOfSoftEdgeConstraints()
      const; /**< Getter for the set of soft edge constraints */
  int getStartTime()
      const; /**< Getter for the time at which the (sub)problem starts */

  //! @param [in] agent an agent of the problem
  //! @param [in] position The current position
  //! @param [in] newPosition The desired position
  //! @param [in] time At what time should we check for constraints?
  //! @return true if the agent is allowed to go from position to newPosition
  //! between time-1 and time (according to the hard vertex constraints and the
  //! hard edge constraints of the problem)
  bool okForConstraints(int agent, int position, int newPosition,
                        int time) const;

  //! @param [in] agent an agent of the problem
  //! @param [in] newPosition The desired position
  //! @param [in] time At what time should we check for constraints?
  //! @return true if the agent is allowed to be at newPosition
  //! at time (according to the hard vertex constraints of the
  //! problem)
  bool okForConstraints(int agent, int newPosition, int time) const;

  //! @param [in] agent an agent of the problem
  //! @param [in] position The current position
  //! @param [in] newPosition The desired position
  //! @param [in] time At what time should we check for constraints?
  //! @return The number of violated soft constraints if agent go from position
  //! to newPosition between time-1 and time (according to the soft vertex
  //! constraints and the soft edge constraints of the problem)
  int numberOfViolations(int agent, int position, int newPosition,
                         int time) const;

  //! @param [in] agent an agent of the problem
  //! @param [in] newPosition The desired position
  //! @param [in] time At what time should we check for constraints?
  //! @return The number of violated soft constraints if agent is at
  //! newPosition at time (according to the soft vertex
  //! constraints of the problem)
  int numberOfViolations(int agent, int newPosition, int time) const;

private:
  //! Graph with the possible positions and transitions for the agents
  std::shared_ptr<Graph> graph;

  //! Number of agents in this problem
  int numberOfAgents;

  //! starts is a list of length numberOfAgents with the start position of each
  //! agent starts[index] is the start position of the agent at index
  std::vector<int> starts;

  //! target is a list of length numberOfAgents with the target position of each
  //! agent targets[index] is the target position of the agent at index
  std::vector<int> targets;

  //! The objective function to minimize : Fuel or Makespan or SumOfCosts
  //! - Fuel : Total amount of distance traveled by all agents
  //! - Makespan : Total time for the last agent to reach its goal
  //! - SumOfCosts : The sum of the time steps required for every agent to reach
  //! its goal and remain at the goal position without leaving it again
  ObjectiveFunction objective;

  std::vector<int> agentIds; /**< Id of each agent at each index */
  std::unordered_map<int, int>
      idToIndex; /**< Index of each agent with each id */

  //! The solution of this problem must have a cost inferior or equal to maxCost
  int maxCost;

  //! true if a start or a target position is unreachable
  //! or if 2 agents have the same start position or the same target position
  //! (or the same id) or if the starts, targets and agentIds vectors doesn't
  //! have the same size
  bool impossible;

  //! true if the problem has external constraints
  bool externalConstraints;

  //! Set of hard vertex constraints like (a, p, t) meaning agent a can't be at
  //! position p at time t
  //! When planning agent agentId, the agent won't go at
  //! position p at time t if a==agentId
  HardVertexConstraintsSet setOfHardVertexConstraints;
  //! Set of hard edge constraints (a, p, q, t)
  HardEdgeConstraintsSet setOfHardEdgeConstraints;

  //! Set of soft vertex constraints like (a, p, t) meaning agent a is occupying
  //! position p at time t
  //! When planning agent agentId, the agent will try to
  //! avoid position p at time t if a!=agentId
  SoftVertexConstraintsMultiSet setOfSoftVertexConstraints;
  //! Set of soft edge constraints (a, p, q, t)
  SoftEdgeConstraintsMultiSet setOfSoftEdgeConstraints;

  //! Start time of the paths. Agent is at position start at time startTime.
  int startTime;
};

#endif // TFE_MAPF_MULTIAGENTPROBLEM_H
