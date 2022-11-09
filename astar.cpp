//
// Created by mansj on 9/11/22.
//

#include "astar.h"
#include "searchable_pqueue.h"
#include "searchable_queue.h"
#include <queue>
#include <vector>
#include <cmath>

void astar::shortest_path(std::vector<int> const& grid_size, Node target, Node start) {
    searchable_pqueue openlist;
    searchable_queue closedlist;
    openlist.push(start);
    while (!openlist.empty()) {
        Node n = openlist.top();
        openlist.pop();
        if (n.x == target.x && n.y == target.y) {
            // reconstruct_path
            return; // return path
        }
        // make this for all neighbors
        Node* neigh = closedlist.search(n.x+1, n.y);
        if (neigh == nullptr)
            neigh = openlist.search(n.x+1, n.y);
        if (neigh == nullptr || neigh->cost > n.cost) {
            Node v(n.x+1, n.y, n.cost+1, n.cost+1 + distance(n.x, n.y, n.x+1, n.y));
            openlist.push(v);
        }
        closedlist.push(n);
    }
}

double astar::distance(int a, int b, int c, int d) {
    return sqrt(pow(a-c, 2)+pow(b-d, 2));
}
