//
// Created by mansj on 10/11/22.
//

#include "library.h"
#include "parser.h"
#include <vector>

int main() {
    // Graph g = parser::parse("/home/mansj/CLionProjects/TFE_MAPF/Benchmarks/map_empty_4x4.map");

    // TEST 1 : 1 agent
    /*Graph g = parser::parse("/Users/arthurmahy/Desktop/memoire - LINFO2990/TFE_MAPF/mapf-map/AssignmentIACourse.map");
    vector<int> starts;
    starts.push_back(7);
    vector<int> targets;
    targets.push_back(17);
    Problem problem = Problem(g, starts, targets, "Fuel");
    Solution solution = aStarSearch(problem);
    // cost = 18, numberOfVisitedStates = 27
    solution.print();*/

    // TEST 2 : 1 agent
    /*Graph g = parser::parse("/Users/arthurmahy/Desktop/memoire - LINFO2990/TFE_MAPF/mapf-map/Berlin_1_256.map");
    vector<int> starts;
    starts.push_back(1);
    vector<int> targets;
    targets.push_back(256*256-1);
    Problem problem = Problem(g, starts, targets, "SumOfCosts");
    Solution solution = aStarSearch(problem);
    // cost = 509
    solution.print();*/

    // TEST 3 : 2 agents can't be at the same vertex at the same time
    /*Graph g = parser::parse("/Users/arthurmahy/Desktop/memoire - LINFO2990/TFE_MAPF/mapf-map/empty-4-4.map");
    vector<int> starts;
    starts.push_back(4);
    starts.push_back(9);
    vector<int> targets;
    targets.push_back(6);
    targets.push_back(1);
    Problem problem = Problem(g, starts, targets, "SumOfCosts");
    Solution solution = aStarSearch(problem);
    // makespan cost = 3, fuel cost = 4,  sumofcosts cost = 5
    solution.print();*/

    // TEST 4 : 2 agents can't traverse the same edge between successive time steps
    /*Graph g = parser::parse("/Users/arthurmahy/Desktop/memoire - LINFO2990/TFE_MAPF/mapf-map/empty-4-4.map");
    vector<int> starts;
    starts.push_back(0);
    starts.push_back(3);
    vector<int> targets;
    targets.push_back(3);
    targets.push_back(0);
    Problem problem = Problem(g, starts, targets, "SumOfCosts");
    Solution solution = aStarSearch(problem);
    // makespan cost = 5, fuel cost = 8,  sumofcosts cost = 8
    solution.print();*/

    // TEST 5 : 2 agents
    Graph g = parser::parse("/Users/arthurmahy/Desktop/memoire - LINFO2990/TFE_MAPF/mapf-map/AssignmentIACourse.map");
    vector<int> starts;
    starts.push_back(48);
    starts.push_back(17);
    vector<int> targets;
    targets.push_back(17);
    targets.push_back(20);
    Problem problem = Problem(g, starts, targets, "SumOfCosts");
    Solution solution = aStarSearch(problem);
    // makespan cost = 7, fuel cost = 10,  sumofcosts cost = 10
    solution.print();

    // TEST 6 : 1 agent and a constraint (a, p, t)
    /*Graph g = parser::parse("/Users/arthurmahy/Desktop/memoire - LINFO2990/TFE_MAPF/mapf-map/empty-4-4.map");
    vector<int> starts;
    starts.push_back(4);
    vector<int> targets;
    targets.push_back(6);
    Problem problem = Problem(g, starts, targets, "SumOfCosts", vector<Constraint>{Constraint(0,5,1)});
    Solution solution = aStarSearch(problem);
    // makespan cost = 3, fuel cost = 2,  sumofcosts cost = 3 (if 2 states are equal if same positions and same time)
    // makespan cost = 4, fuel cost = 4,  sumofcosts cost = 4 (if 2 states are equal if same positions) (SOUCIS HERE)
    solution.print();*/
}
