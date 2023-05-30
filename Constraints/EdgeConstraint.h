//
// Created by Arthur Mahy on 07/04/2023.
//

#ifndef TFE_MAPF_EDGECONSTRAINT_H
#define TFE_MAPF_EDGECONSTRAINT_H

#include <iostream>

class EdgeConstraint {
public:

    // agent cannot go from position1 to position2 between time-1 and time (if positive is false)
    // agent has to go from position1 to position2 between time-1 and time (otherwise)
    EdgeConstraint(int agent, int position1, int position2, int time, bool positive = false) : agent(agent), position1(position1), position2(position2), time(time), positive(positive) {}
    ~EdgeConstraint() {}

    int getAgent() const {
        return agent;
    }
    int getPosition1() const {
        return position1;
    }
    int getPosition2() const {
        return position2;
    }
    int getTime() const {
        return time;
    }
    void print() const {
        if (positive){
            std::cout << "Positive edge constraint : agent " << agent << " has to go from position " << position1 << " to position " << position2 << " between time " << time-1 << " and time " << time << "." << std::endl;
        } else {
            std::cout << "Edge constraint : agent " << agent << " cannot go from position " << position1 << " to position " << position2 << " between time " << time-1 << " and time " << time << "." << std::endl;
        }
    }
    bool isPositive(){
        return positive;
    }

    bool operator<(const EdgeConstraint &other) const {
        if (positive == other.positive){
            if (agent == other.agent) {
                if (position1 == other.position1) {
                    if (position2 == other.position2) {
                        return time < other.time;
                    }
                    return position2 < other.position2;
                }
                return position1 < other.position1;
            }
            return agent < other.agent;
        }
        return positive < other.positive;
    }

private:

    int agent;
    int position1;
    int position2;
    int time;
    bool positive;
};


#endif //TFE_MAPF_EDGECONSTRAINT_H
