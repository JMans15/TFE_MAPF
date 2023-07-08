//
// Created by Arthur Mahy on 07/04/2023.
//

#ifndef TFE_MAPF_EDGECONSTRAINT_H
#define TFE_MAPF_EDGECONSTRAINT_H

#include <iostream>

class EdgeConstraint {
public:
  // IF IN A SET OF HARD CONSTRAINTS :
  // agent cannot go from position1 to position2 between time-1 and time (if
  // positive is false) agent has to go from position1 to position2 between
  // time-1 and time (otherwise)
  //
  // IF IN A SET OF SOFT CONSTRAINTS :
  // agent is occupying the edge (position1, position2) between time-1 and time
  EdgeConstraint(int agent, int position1, int position2, int time,
                 bool positive = false)
      : agent(agent), position1(position1), position2(position2), time(time),
        positive(positive) {}
  ~EdgeConstraint() = default;

  int getAgent() const { return agent; }
  int getPosition1() const { return position1; }
  int getPosition2() const { return position2; }
  int getTime() const { return time; }
  // print method is not accurate in a set of soft constraints
  void print() const {
    if (positive) {
      std::cout << "Positive edge constraint : agent " << agent
                << " has to go from position " << position1 << " to position "
                << position2 << " between time " << time - 1 << " and time "
                << time << "." << std::endl;
    } else {
      std::cout << "Edge constraint : agent " << agent
                << " cannot go from position " << position1 << " to position "
                << position2 << " between time " << time - 1 << " and time "
                << time << "." << std::endl;
    }
  }
  bool isPositive() const { return positive; }

private:
  int agent;
  int position1;
  int position2;
  int time;
  bool positive;
};

#endif // TFE_MAPF_EDGECONSTRAINT_H
