//
// Created by mansj on 10/11/22.
//

#ifndef MAPF_REBORN_ASTAR_SINGLE_H
#define MAPF_REBORN_ASTAR_SINGLE_H

#include "Node.h"
#include <vector>
#include "Graph.h"

using std::vector;

class astar_single {
    static double distance(int a, int b, int width);
public:
    static int shortest_path(Graph g, int target, int start);
};


#endif //MAPF_REBORN_ASTAR_SINGLE_H
