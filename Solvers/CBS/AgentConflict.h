//
// Created by Arthur Mahy on 22/04/2023.
//

#ifndef TFE_MAPF_AGENTCONFLICT_H
#define TFE_MAPF_AGENTCONFLICT_H

#include <iostream>
class AgentConflict {
public:
  // vertex conflict : agent1 and agent2 are at position at time
  AgentConflict(int agent1, int agent2, int position, int time)
      : agent1(agent1), agent2(agent2), position1(position), time(time) {
    vertexConflict = true;
  }
  // edge conflict : agent1 and agent2 are occupying the edge {position1,
  // position2} from time-1 to time agent1 goes from position1 to position2 and
  // agent2 goes from position2 to position1
  AgentConflict(int agent1, int agent2, int position1, int position2, int time)
      : agent1(agent1), agent2(agent2), position1(position1),
        position2(position2), time(time) {
    vertexConflict = false;
  }
  ~AgentConflict() = default;

  bool isVertexConflict() const { return vertexConflict; }
  int getAgent1() const { return agent1; }
  int getAgent2() const { return agent2; }
  int getPosition1() const { return position1; }
  int getPosition2() const { return position2; }
  int getTime() const { return time; }
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
  bool operator<(const AgentConflict &other) const {
    if (time == other.time) {
      return true;
    }
    return time < other.time;
  }

private:
  bool vertexConflict;
  int agent1;
  int agent2;
  int position1;
  int position2;
  int time;
};

#endif // TFE_MAPF_AGENTCONFLICT_H
