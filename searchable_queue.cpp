//
// Created by mansj on 9/11/22.
//

#include "searchable_queue.h"

astar::Node *searchable_queue::search(const int x, const int y) const {
    auto cont = this->c;
    for (const astar::Node &n : cont) {
        if (n.x == x && n.y == y) {
            return const_cast<astar::Node *>(&n);
        }
    }
    return nullptr;
}
