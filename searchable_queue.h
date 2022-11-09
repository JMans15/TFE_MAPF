//
// Created by mansj on 9/11/22.
//

#ifndef TFE_MAPF_SEARCHABLE_QUEUE_H
#define TFE_MAPF_SEARCHABLE_QUEUE_H

#include <queue>
#include "astar.h"

class searchable_queue : public std::queue<
        astar::Node,
        std::vector<astar::Node>> {
public:
    astar::Node* search(int, int) const;
};


#endif //TFE_MAPF_SEARCHABLE_QUEUE_H
