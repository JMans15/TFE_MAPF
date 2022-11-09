//
// Created by mansj on 9/11/22.
//

#ifndef TFE_MAPF_SEARCHABLE_PQUEUE_H
#define TFE_MAPF_SEARCHABLE_PQUEUE_H
#include "astar.h"
#include <queue>
#include <vector>

class searchable_pqueue : public std::priority_queue<
        astar::Node,
        std::vector<astar::Node>,
        astar::compare_heuristic> {
public:
    astar::Node* search(int, int) const;
};


#endif //TFE_MAPF_SEARCHABLE_PQUEUE_H
