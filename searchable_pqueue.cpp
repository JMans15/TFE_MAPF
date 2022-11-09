//
// Created by mansj on 9/11/22.
//

#include "searchable_pqueue.h"
#include <vector>
#include "astar.h"

astar::Node *searchable_pqueue::search(const int x, const int y) const  {
    auto cont = this->c;
    for (const astar::Node &n : cont) {
        if (n.x == x && n.y == y) {
            return const_cast<astar::Node *>(&n);
        }
    }
    return nullptr;
}
