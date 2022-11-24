//
// Created by mansj on 10/11/22.
//

#include "library.h"
#include "parser.h"
#include <vector>

int main() {
    //Graph g = parser::parse("/home/mansj/CLionProjects/MAPF_REBORN/Benchmarks/Berlin_1_256.map");

    // TEST 1
    Graph g = parser::parse("/Users/arthurmahy/Desktop/memoire - LINFO2990/TFE_MAPF/mapf-map/empty-4-4.map");
    vector<int> starts;
    starts.push_back(4);
    starts.push_back(9);
    vector<int> targets;
    targets.push_back(6);
    targets.push_back(1);
    Problem problem = Problem(g, starts, targets, "Makespan");
    Solution solution = aStarSearch(problem);
    solution.print();

    // TEST 2
    /*Graph g = parser::parse("/Users/arthurmahy/Desktop/memoire - LINFO2990/TFE_MAPF/mapf-map/AssignmentIACourse.map");
    vector<int> starts;
    starts.push_back(7);
    vector<int> targets;
    targets.push_back(17);
    Problem problem = Problem(g, starts, targets, "Fuel");
    Solution solution = aStarSearch(problem);
    solution.print();*/

    // TEST 3
    /*Graph g = parser::parse("/Users/arthurmahy/Desktop/memoire - LINFO2990/TFE_MAPF/mapf-map/empty-4-4.map");
    vector<int> starts;
    starts.push_back(0);
    starts.push_back(3);
    vector<int> targets;
    targets.push_back(3);
    targets.push_back(0);
    Problem problem = Problem(g, starts, targets, "Makespan");
    Solution solution = aStarSearch(problem);
    solution.print();*/

    // TEST 4
    /*Graph g = parser::parse("/Users/arthurmahy/Desktop/memoire - LINFO2990/TFE_MAPF/mapf-map/AssignmentIACourse.map");
    vector<int> starts;
    starts.push_back(48);
    starts.push_back(17);
    vector<int> targets;
    targets.push_back(17);
    targets.push_back(20);
    Problem problem = Problem(g, starts, targets, "Fuel");
    Solution solution = aStarSearch(problem);
    solution.print();*/

    // TEST 5
    /*Graph g = parser::parse("/Users/arthurmahy/Desktop/memoire - LINFO2990/TFE_MAPF/mapf-map/Berlin_1_256.map");
    vector<int> starts;
    starts.push_back(1);
    vector<int> targets;
    targets.push_back(256*256-1);
    Problem problem = Problem(g, starts, targets, "Fuel");
    Solution solution = aStarSearch(problem);
    solution.print();*/

    // TEST 6
    /*Graph g = parser::parse("/Users/arthurmahy/Desktop/memoire - LINFO2990/TFE_MAPF/mapf-map/empty-4-4.map");
    vector<int> starts;
    starts.push_back(4);
    vector<int> targets;
    targets.push_back(6);
    Problem problem = Problem(g, starts, targets, "Fuel", vector<Constraint>{Constraint(0,5,1)});
    Solution solution = aStarSearch(problem);
    solution.print();*/
}
