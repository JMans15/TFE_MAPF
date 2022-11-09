//
// Created by mansj on 9/11/22.
//

#ifndef TFE_MAPF_ASTAR_H
#define TFE_MAPF_ASTAR_H
#include <vector>

class astar {
    struct Node {
        Node(int x, int y, int cost, double heur) :
                x(x), y(y), cost(cost), heur(heur) {}
        Node(int x, int y) :
                x(x), y(y), cost(0), heur(0) {}
        int x, y, cost;
        double heur;
    };
    struct compare_heuristic {
        bool operator()(Node const& a, Node const& b) {
            return b.heur - a.heur;
        }
    };
    static double distance(int a, int b, int c, int d);
    void shortest_path(const std::vector<int>& grid_size, Node target, Node start);
    friend class searchable_pqueue;
    friend class searchable_queue;
};


#endif //TFE_MAPF_ASTAR_H
