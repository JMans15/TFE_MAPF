//
// Created by Arthur Mahy on 27/02/2023.
//

#ifndef TFE_MAPF_REVERSERESUMABLEASTAR_H
#define TFE_MAPF_REVERSERESUMABLEASTAR_H
#include "SingleAgentProblem.h"
#include "unordered_map"
#include <queue>
#include "Heuristic.h"
#include "Node.h"

typedef tuple<int, Node> Tuple;

struct StatePtrHash {
    size_t operator()(const shared_ptr<State>& ptr) const {
        return ptr->hash();
    }
};

struct StatePtrEqual {
    size_t operator()(const shared_ptr<State>& ptr, const shared_ptr<State>& other) const {
        return *ptr == *other;
    }
};

typedef unordered_map<shared_ptr<State>, int, StatePtrHash, StatePtrEqual> myUMap;

struct CompareF {
    bool operator()(const Tuple& lhs, const Tuple& rhs) const
    {
        return get<0>(lhs) > get<0>(rhs);
    }
};

class Heuristic;

// Reverse Resumable A* search
// Only for single agent problem
// Consists of a single agent search where the beginning of the search is the goal state (target position) of the problem
// Possibility to continue the search (with the Resume method) even when the start state of the problem is goal tested
// (https://www.davidsilver.uk/wp-content/uploads/2020/03/coop-path-AIWisdom.pdf)
class ReverseResumableAStar {
public :
    ReverseResumableAStar(SingleAgentProblem* m_problem);

    // Continues the search from target to start until state is met
    // Returns the optimal cost between goal state and state
    int Resume(const shared_ptr<State>& state);

    // Returns the optimal distance (walls are taken into account) between position and target
    int OptimalDistance(int position);

    // Returns the map where (key, value) = (shared pointer of a state, optimal distance from state to target)
    myUMap getExplored();
private:
    SingleAgentProblem* problem;
    Heuristic* heuristic;
    myUMap explored; // the closed list
    priority_queue<Tuple, vector<Tuple>, CompareF> fringe;
};


#endif //TFE_MAPF_REVERSERESUMABLEASTAR_H
