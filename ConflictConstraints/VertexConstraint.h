//
// Created by Arthur Mahy on 07/04/2023.
//

#ifndef TFE_MAPF_VERTEXCONSTRAINT_H
#define TFE_MAPF_VERTEXCONSTRAINT_H

#include <iostream>

class VertexConstraint {
public:

    // agent cannot be at position at time
    VertexConstraint(int agent, int position, int time) : agent(agent), position(position), time(time) {}
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
        std::cout << "Vertex constraint : agent " << agent << " cannot be at position " << position << " at time " << time << "." << std::endl;
    }

    bool operator<(const VertexConstraint &other) const {
        if (agent == other.agent) {
            if (position == other.position) {
                return time < other.time;
            }
            return position < other.position;
        }
        return agent < other.agent;
    }

private:

    int agent;
    int position;
    int time;
};


#endif //TFE_MAPF_VERTEXCONSTRAINT_H
