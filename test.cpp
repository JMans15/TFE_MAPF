//
// Created by mansj on 10/11/22.
//

#include "library.h"
#include "parser.h"
//#include "astar_single.h"
#include <iostream>
#include <vector>

int distance(int a, int b, int width) {
    int ax, ay, bx, by;
    ax = (int) a / width; ay = a % width;
    bx = (int) b / width; by = b % width;
    return abs(ax-bx) + abs(ay-by);
}

// Sum of Individual Costs heuristic
int SICheuristic(State state, const Problem& problem){
    int sum = 0;
    vector<int> positions = state.getPositions();
    for (int i = 0; i < positions.size(); i++){
        sum += distance(positions[i], problem.getTargets()[i], problem.getGraph().getN());
    }
    return sum;
}

int main() {
    //Graph g = parser::parse("/home/mansj/CLionProjects/MAPF_REBORN/Benchmarks/Berlin_1_256.map");
    Graph g = parser::parse("/Users/arthurmahy/Desktop/memoire - LINFO2990/TFE_MAPF/mapf-map/random-32-32-10.map");
    vector<int> starts;
    starts.push_back(0);
    starts.push_back(1);
    vector<int> targets;
    targets.push_back(1);
    targets.push_back(0);
    Problem problem = Problem(g, starts, targets);
    //int len = astar_single::shortest_path(g, 0, 256*256-1);
    //std::cout << "Found path of length " << len << std::endl;
    vector<string> path = aStarSearch(problem, SICheuristic);
    for (const string& action: path){
        std::cout << action << std::endl;
    }
}
