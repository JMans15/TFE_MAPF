//! Specific class for single-agent problems

#ifndef TFE_MAPF_SINGLEAGENTPROBLEM_H
#define TFE_MAPF_SINGLEAGENTPROBLEM_H

#include "Problem.h"

class SingleAgentProblem : public Problem {
public:
  /**
   * Constructor for a single agent problem
   * @param [in] &graph The graph representing the map on which the problem
   * takes place
   * @param [in] start The starting point of the agent
   * @param [in] target The target of the agent
   * @param [in] agentId The id of the agent (useful for subproblems used in
   * other algorithms)
   * @param [in] maxCost An upper bound on the cost of the path (used in EID for
   * replaning)
   */
  SingleAgentProblem(const std::shared_ptr<Graph> &graph, int start, int target,
                     int agentId = 0, int maxCost = INT_MAX);

  /**
   * Constructor for a single agent problem with constraints
   * @param [in] &graph The graph representing the map on which the problem
   * takes place
   * @param [in] start The starting point of the agent
   * @param [in] target The target of the agent
   * @param [in] objective The objective function to use
   * @param [in] agentId The id of the agent (useful for subproblems used in
   * other algorithms)
   * @param [in] &setOfHardVertexConstraints the set of hard vertex constraints
   * @param [in] &setOfHardEdgeConstraints the set of hard vertex constraints
   * @param [in] maxCost An upper bound on the cost of the path (used in EID for
   * replaning)
   * @param [in] &setOfSoftVertexConstraints the set of soft vertex constraints
   * @param [in] &setOfSoftEdgeConstraints the set of soft edge constraints
   * @param [in] startTime the time at which the problem starts
   */
  SingleAgentProblem(
      const std::shared_ptr<Graph> &graph, int start, int target,
      ObjectiveFunction objective, int agentId = 0,
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
  getGraph() const; /**< Getter for the graph representing the map on which the
                       problem takes place */
  int getStart() const;   /**< Getter for the starting point of the agent */
  int getTarget() const;  /**< Getter for the target of the agent */
  int getAgentId() const; /**< Getter for the agent's ID */
  int getMaxCost() const; /**< Getter for the max cost of the path */
  void print()
      const; /**< Print function to print the problem as a string to stdout */

  int getNumberOfAgents() const override;
  bool isImpossible() const override;
  bool hasExternalConstraints() const override;

  ObjectiveFunction getObjFunction(); /**< Getter for the objective function
                                         used in the problem */
  HardVertexConstraintsSet getSetOfHardVertexConstraints()
      const; /**< Getter for the set of hard vertex constraints */
  HardEdgeConstraintsSet getSetOfHardEdgeConstraints()
      const; /**< Getter for the set of hard edge constraints */
  SoftVertexConstraintsMultiSet getSetOfSoftVertexConstraints()
      const; /**< Getter for the set of soft vertex constraints */
  SoftEdgeConstraintsMultiSet getSetOfSoftEdgeConstraints()
      const;                /**< Getter for the set of soft edge constraints */
  int getStartTime() const; /**< Getter for the starting time of the problem */

  //! @param [in] position The current position
  //! @param [in] newPosition The desired position
  //! @param [in] time At what time should we check for constraints?
  //! @return true if the agent is allowed to go from position to newPosition
  //! between time-1 and time (according to the hard vertex constraints and the
  //! hard edge constraints of the problem)
  bool okForConstraints(int position, int newPosition, int time) const;

  //! @param [in] newPosition The desired position
  //! @param [in] time At what time should we check for constraints?
  //! @return true if the agent is allowed to go to newPosition
  //! between time-1 and time (according to the hard vertex constraints of the
  //! problem)
  bool okForConstraints(int newPosition, int time) const;

  //! @param [in] position The current position
  //! @param [in] newPosition The desired position
  //! @param [in] time At what time should we check for constraints?
  //! @return The number of violated soft constraints if agent go from position
  //! to newPosition between time-1 and time (according to the soft vertex
  //! constraints and the soft edge constraints of the problem)
  int numberOfViolations(int position, int newPosition, int time) const;

  //! @param [in] newPosition The desired position
  //! @param [in] time At what time should we check for constraints?
  //! @return The number of violated soft constraints if agent is at
  //! newPosition between time-1 and time (according to the soft vertex
  //! constraints of the problem)
  int numberOfViolations(int newPosition, int time) const;

private:
  //! Graph with the possible positions and transitions for the agent
  std::shared_ptr<Graph> graph;

  //! Start position of the agent
  int start;

  //! Target position of the agent
  int target;

  //! Id of the agent
  int agentId;

  //! The solution of this problem must have a cost inferior or equal to maxCost
  int maxCost;

  //! true if the start or the target position is unreachable
  bool impossible;

  //! true if the problem has external constraints
  bool externalConstraints;

  //! The objective function to minimize : Fuel or Makespan
  //! - Fuel : Total amount of distance traveled by the agent (costWait = 0)
  //! - Makespan (or SumOfCosts) : Total time for the agent to reach its goal
  //! (costWait = 1)
  ObjectiveFunction objective;

  //! Set of hard constraints like (a, p, t) meaning agent a can't be at
  //! position p at time t When planning agent agentId, the agent won't go at
  //! position p at time t if a==agentId
  HardVertexConstraintsSet setOfHardVertexConstraints;
  //! Set of hard edge constraints (a, p, q, t)
  HardEdgeConstraintsSet setOfHardEdgeConstraints;

  //! Set of soft constraints like (a, p, t) meaning agent a is occupying
  //! position p at time t When planning agent agentId, the agent will try to
  //! avoid position p at time t if a!=agentId
  SoftVertexConstraintsMultiSet setOfSoftVertexConstraints;
  //! Set of soft edge constraints (a, p, q, t)
  SoftEdgeConstraintsMultiSet setOfSoftEdgeConstraints;

  //! Start time of the path. Agent is at position start at time startTime.
  int startTime;
};

#endif // TFE_MAPF_SINGLEAGENTPROBLEM_H
