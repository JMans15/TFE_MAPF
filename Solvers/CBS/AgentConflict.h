//
// Created by Arthur Mahy on 22/04/2023.
//

#ifndef TFE_MAPF_AGENTCONFLICT_H
#define TFE_MAPF_AGENTCONFLICT_H

#include <iostream>
class AgentConflict {
public:
  //! Constructor for an edge conflict
  //! @param [in] agent1 First agent involved in the conflict
  //! @param [in] agent2 Second agent involved in the conflict
  //! @param [in] position Conflicting position of the two agents
  //! @param [in] time time at which the conflict occurs
  AgentConflict(int agent1, int agent2, int position, int time)
      : agent1(agent1), agent2(agent2), position1(position), time(time) {
    vertexConflict = true;
  }
  //! Constructor for an edge conflict
  //! @param [in] agent1 First agent involved in the conflict
  //! @param [in] agent2 Second agent involved in the conflict
  //! @param [in] position1 start position of agent1, end position of agent2
  //! @param [in] position2 start position of agent2, end position of agent1
  //! @param [in] time time at which the conflict occurs (from time-1 to time)
  AgentConflict(int agent1, int agent2, int position1, int position2, int time)
      : agent1(agent1), agent2(agent2), position1(position1),
        position2(position2), time(time) {
    vertexConflict = false;
  }
  ~AgentConflict() = default;

  //! Getter for vertexConflict
  bool isVertexConflict() const { return vertexConflict; }
  //! Getter for agent1
  int getAgent1() const { return agent1; }
  //! Getter for agent2
  int getAgent2() const { return agent2; }
  //! Getter vor position1
  int getPosition1() const { return position1; }
  //! Getter for position2
  int getPosition2() const { return position2; }
  //! getter for time
  int getTime() const { return time; }
  //! Formats informations and print
  void print() const {
    if (vertexConflict) {
      std::cout << "Vertex conflict : agent " << agent1 << " and agent "
                << agent2 << " are at position " << position1 << " at time "
                << time << "." << std::endl;
    } else {
      std::cout << "Edge conflict : agent " << agent1 << " and agent " << agent2
                << " are occupying the edge {" << position1 << ", " << position2
                << "} from time " << time - 1 << " to time " << time << "."
                << std::endl;
    }
  }

  // TODO : think about what to put here
  // first the conflict with the lower time
  // and then ??

  //! Defines comparison between conflicts as comparison between times
  bool operator<(const AgentConflict &other) const {
    if (time == other.time) {
      return true;
    }
    return time < other.time;
  }

private:
  bool vertexConflict; /**< True if vertex conflict, false if edge conflict */
  int agent1;          /**< One of the agents involved in the conflict */
  int agent2;          /**< The other agent involved in the conflict */
  int position1; /**< start position of an edge conflict or position of a vertex
                    conflict */
  int position2; /**< end position of an edge conflict */
  int time;      /**< Timestep at wchich the conflict occurs */
};

#endif // TFE_MAPF_AGENTCONFLICT_H
