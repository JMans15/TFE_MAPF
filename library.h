#ifndef TFE_MAPF_LIBRARY_H
#define TFE_MAPF_LIBRARY_H

#include "Problem.h"
#include <vector>
#include "State.h"
#include "Solution.h"
using namespace std;

Solution aStarSearch(Problem* problem, TypeOfHeuristic typeOfHeuristic);

#endif //TFE_MAPF_LIBRARY_H
