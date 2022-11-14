//
// Created by mansj on 10/11/22.
//

#ifndef MAPF_REBORN_NODE_H
#define MAPF_REBORN_NODE_H


class Node {
public:
    int cost, index;
    double heuristic;
    Node(int index, int cost, double heuristic);
    explicit Node(int index);
    struct compare_heuristic {
        bool operator()(Node const& a, Node const& b) {
            return a.heuristic < b.heuristic;
        }
    };
};


#endif //MAPF_REBORN_NODE_H
