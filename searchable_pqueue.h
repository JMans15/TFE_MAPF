//
// Created by mansj on 9/11/22.
//

#ifndef TFE_MAPF_SEARCHABLE_PQUEUE_H
#define TFE_MAPF_SEARCHABLE_PQUEUE_H
#include "astar_single.h"
#include <queue>
#include <vector>
#include "Node.h"

class searchable_pqueue : public std::priority_queue<
        Node,
        std::vector<Node>,
        Node::compare_heuristic> {
public:
    Node* search(int) const;
};


#endif //TFE_MAPF_SEARCHABLE_PQUEUE_H
