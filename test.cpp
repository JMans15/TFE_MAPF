//
// Created by mansj on 10/11/22.
//

#include "parser.h"
#include "library.h"
#include "MultiAgentProblem.h"
#include "SingleAgentProblem.h"
#include <vector>
#include <chrono>

template <typename T>
class Clock {
    chrono::high_resolution_clock clock;
    chrono::time_point<chrono::high_resolution_clock, chrono::nanoseconds> start;
    chrono::time_point<chrono::high_resolution_clock, chrono::nanoseconds> stop;
public:
    void tick() {
        this->start = this->clock.now();
    }
    void tack () {
        this->stop = this->clock.now();
    };
    void print () {
        cerr << chrono::duration_cast<T>(this->stop-this->start) << endl;
    };
};

Clock<chrono::milliseconds> timer;

int main() {
    // Graph g = parser::parse("/home/mansj/CLionProjects/TFE_MAPF/Benchmarks/map_empty_4x4.map");

    // TEST 1 : 1 agent
    /*Graph g = parser::parse("../mapf-map/AssignmentIACourse.map");
    int start = 7;
    int target = 17;
    SingleAgentProblem problem = SingleAgentProblem(g, start, target);
    Solution solution = aStarSearch(&problem, Manhattan);
    // cost = 18, numberOfVisitedStates = 27
    solution.print();*/

    // TEST 2 : 1 agent
    Graph g = parser::parse("../mapf-map/Paris/Paris_1_256.map");
    int start = 1;
    int target = 256*200-100;
    timer.tick();
    SingleAgentProblem problem = SingleAgentProblem(g, start, target);
    Solution solution = aStarSearch(&problem, Manhattan);
    timer.tack();
    // cost = 509
    solution.print();
    timer.print();

    // TEST 3 : 2 agents can't be at the same vertex at the same time
    /*Graph g = parser::parse("/Users/arthurmahy/Desktop/memoire - LINFO2990/TFE_MAPF/mapf-map/empty-4-4.map");
    vector<int> starts;
    starts.push_back(4);
    starts.push_back(9);
    vector<int> targets;
    targets.push_back(6);
    targets.push_back(1);
    MultiAgentProblem problem = MultiAgentProblem(g, starts, targets, Makespan);
    Solution solution = aStarSearch(&problem, MIC);
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
    MultiAgentProblem problem = MultiAgentProblem(g, starts, targets, SumOfCosts);
    Solution solution = aStarSearch(&problem, SIC);
    // makespan cost = 5, fuel cost = 8,  sumofcosts cost = 8
    solution.print();*/

    // TEST 5 : 2 agents
    /*Graph g = parser::parse("../mapf-map/Paris/Paris_1_256.map");
    vector<int> starts;
    starts.push_back(48);
    starts.push_back(17);
    vector<int> targets;
    targets.push_back(17);
    targets.push_back(20);
    MultiAgentProblem problem = MultiAgentProblem(g, starts, targets, SumOfCosts);
    Solution solution = aStarSearch(&problem, SIC);
    // makespan cost = 7, fuel cost = 10,  sumofcosts cost = 10
    solution.print();*/

    // TEST 6 : 1 agent and a constraint (a, p, t)
    /*Graph g = parser::parse("/Users/arthurmahy/Desktop/memoire - LINFO2990/TFE_MAPF/mapf-map/empty-4-4.map");
    vector<int> starts;
    starts.push_back(4);
    vector<int> targets;
    targets.push_back(6);
    MultiAgentProblem problem = MultiAgentProblem(g, starts, targets, Makespan, vector<Constraint>{Constraint(0,5,1)});
    Solution solution = aStarSearch(&problem, MIC);
    // makespan cost = 3, fuel cost = 2,  sumofcosts cost = 3 (if 2 states are equal if same positions and same time)
    // makespan cost = 4, fuel cost = 4,  sumofcosts cost = 4 (if 2 states are equal if same positions) (SOUCIS HERE)
    solution.print();*/
}
