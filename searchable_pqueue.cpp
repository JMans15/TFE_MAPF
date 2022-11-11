//
// Created by mansj on 9/11/22.
//

#include "searchable_pqueue.h"
#include "Node.h"

Node *searchable_pqueue::search(const int index) const  {
    for (const Node &n : this->c) {
        if (n.index == index) {
            return const_cast<Node *>(&n);
        }
    }
    return nullptr;
}
