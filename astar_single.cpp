//
// Created by mansj on 10/11/22.
//

// PAS UTILISÉ DANS LA VERSION MULTI-AGENT

#include "astar_single.h"
#include "searchable_pqueue.h"
#include <vector>
#include <queue>
#include <iostream>
#include <unordered_set>

using std::cout;
using std::endl;
using std::vector;
using std::abs;
using std::unordered_set;

double astar_single::distance(int a, int b, int width) {
    int ax, ay, bx, by;
    ax = (int) a / width; ay = a % width;
    bx = (int) b / width; by = b % width;
    return abs(ax-bx) + abs(ay-by);
}

int astar_single::shortest_path(Graph g, int target, int start) {
    Node startNode(start);
    searchable_pqueue openlist;
    unordered_set<int> closedset;
    openlist.push(startNode);

    Node *found; int isinclosed, isinopenwithlowercost;
    vector<int> neighbors;
    while (!openlist.empty()) {
        Node n = openlist.top();
        openlist.pop();
        if (n.index == target) {
            // reconstruct_path
            return n.cost; // return path
        }
        neighbors = g.getneighbors(n.index);
        for (int neighbor : neighbors) {
            isinclosed = closedset.find(neighbor) != closedset.end();
            found = openlist.search(neighbor);
            isinopenwithlowercost = found != nullptr && found->cost < n.cost;
            if (!(isinclosed || isinopenwithlowercost)) {
                openlist.push(Node(neighbor, n.cost + 1, n.cost + 1
                                                         + distance(neighbor, target, g.getWidth())));
            }
        }
        closedset.insert(n.index);
    }
    return -1;
}
