//
// Created by Arthur Mahy on 27/02/2023.
//

#ifndef TFE_MAPF_ASTAR_H
#define TFE_MAPF_ASTAR_H
#include "Problem.h"
#include "Heuristic.h"
#include "unordered_set"
#include "Solution.h"
#include <queue>

// Basic A* search
// Can be applied for multi agent (operator decomposition) and single agent problems
class AStar {
public :
    AStar(Problem* problem, TypeOfHeuristic typeOfHeuristic);
    Solution getSolution();
private:
    Solution solution;
};


#endif //TFE_MAPF_ASTAR_H
