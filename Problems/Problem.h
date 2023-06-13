//
// Created by Arthur Mahy on 12/06/2023.
//

#ifndef TFE_MAPF_PROBLEM_H
#define TFE_MAPF_PROBLEM_H

#include "../GraphParser/Graph.h"
#include "../Constraints/ConstraintsSet.h"

#include <set>
#include "unordered_map"

//#define DEBUG

#ifdef DEBUG
#define LOG(str) std::cout << str << std::endl;
#else
#define LOG(str)
#endif

enum ObjectiveFunction {
    Fuel, Makespan, SumOfCosts
};

class Problem {
public:

    virtual bool hasTimeConstraints() const = 0;
    virtual bool isMultiAgentProblem() const = 0;
    virtual bool isImpossible() const = 0;

};

#endif //TFE_MAPF_PROBLEM_H
