//
// Created by Arthur Mahy on 07/04/2023.
//

#ifndef TFE_MAPF_VERTEXCONSTRAINT_H
#define TFE_MAPF_VERTEXCONSTRAINT_H

#include <iostream>

class VertexConstraint {
public:

    // IF IN A SET OF HARD CONSTRAINTS :
    // agent cannot be at position at time (if positive is false)
    // agent has to be at position at time (otherwise)
    //
    // IF IN A SET OF SOFT CONSTRAINTS :
    // agent is at position at time
    VertexConstraint(int agent, int position, int time, bool positive = false) : agent(agent), position(position), time(time), positive(positive) {}
    ~VertexConstraint() = default;

    int getAgent() const {
        return agent;
    }
    int getPosition() const {
        return position;
    }
    int getTime() const {
        return time;
    }
    // print method is not accurate in a set of soft constraints
    void print() const {
        if (positive){
            std::cout << "Positive vertex constraint : agent " << agent << " has to be at position " << position << " at time " << time << "." << std::endl;
        } else {
            std::cout << "Vertex constraint : agent " << agent << " cannot be at position " << position << " at time " << time << "." << std::endl;
        }
    }
    bool isPositive() const{
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
