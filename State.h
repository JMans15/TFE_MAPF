//
// Created by Arthur Mahy on 09/11/2022.
//

#ifndef TFE_MAPF_STATE_H
#define TFE_MAPF_STATE_H

#include <utility>
#include <vector>
using namespace std;

class State {

public:

    State(int m_timestep);

    virtual vector<int> getPositions() = 0;
    int getTimestep() const;

    virtual bool operator==(const State& rhs) const = 0;
    virtual size_t hash() const = 0;

protected:

    // the timestep of this state
    int timestep;
};


#endif //TFE_MAPF_STATE_H
