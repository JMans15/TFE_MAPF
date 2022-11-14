//
// Created by mansj on 10/11/22.
//

#include "Node.h"

Node::Node(int index, int cost, double heuristic) :
    cost(cost), index(index), heuristic(heuristic) {}

Node::Node(int index) :
    cost(0), index(index), heuristic(0) {}
