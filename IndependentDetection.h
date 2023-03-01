//
// Created by Arthur Mahy on 28/02/2023.
//

#ifndef TFE_MAPF_INDEPENDENTDETECTION_H
#define TFE_MAPF_INDEPENDENTDETECTION_H
#include "MultiAgentProblem.h"
#include "AStar.h"

// Independent Detection search
// Only for multi agent problem
//
// typeOfHeuristic is the heuristic for the A* searches
class IndependentDetection {
public:
    IndependentDetection(MultiAgentProblem* problem, TypeOfHeuristic typeOfHeuristic);
    Solution getSolution();
private:
    Solution solution;
};


#endif //TFE_MAPF_INDEPENDENTDETECTION_H
