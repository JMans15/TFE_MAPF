//
// Created by Arthur Mahy on 07/04/2023.
//

#ifndef TFE_MAPF_VERTEXCONSTRAINT_H
#define TFE_MAPF_VERTEXCONSTRAINT_H

#include <iostream>

class VertexConstraint {
public:

    // agent cannot be at position at time (if positive is false)
    // agent has to be at position at time (otherwise)
    VertexConstraint(int agent, int position, int time, bool positive = false) : agent(agent), position(position), time(time), positive(positive) {}
    ~VertexConstraint() {}

    int getAgent() const {
        return agent;
    }
    int getPosition() const {
        return position;
    }
    int getTime() const {
        return time;
    }
    void print() const {
        if (positive){
            std::cout << "Positive vertex constraint : agent " << agent << " has to be at position " << position << " at time " << time << "." << std::endl;
        } else {
            std::cout << "Vertex constraint : agent " << agent << " cannot be at position " << position << " at time " << time << "." << std::endl;
        }
    }
    bool isPositive(){
        return positive;
    }

    bool operator<(const VertexConstraint &other) const {
        if (agent == other.agent) {
            if (time == other.time) {
                return position < other.position;
            }
            return time < other.time;
        }
        return agent < other.agent;
    }

private:

    int agent;
    int position;
    int time;
    bool positive;
};


#endif //TFE_MAPF_VERTEXCONSTRAINT_H
