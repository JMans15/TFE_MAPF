//
// Created by mansj on 10/11/22.
//

#include "library.h"
#include "parser.h"
#include "astar_single.h"
#include <iostream>

int main() {
    Graph g = parser::parse("/home/mansj/CLionProjects/MAPF_REBORN/Benchmarks/Berlin_1_256.map");
    int len = astar_single::shortest_path(g, 0, 256*256-1);
    std::cout << "Found path of length " << len << std::endl;
}
