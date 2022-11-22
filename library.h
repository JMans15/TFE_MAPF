#ifndef TFE_MAPF_LIBRARY_H
#define TFE_MAPF_LIBRARY_H

#include "Problem.h"
#include <vector>
#include "State.h"
using namespace std;

vector<string> aStarSearch(const Problem& problem, const function<int(State, Problem)>& heuristic);

#endif //TFE_MAPF_LIBRARY_H
