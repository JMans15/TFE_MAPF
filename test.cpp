//
// Created by mansj on 10/11/22.
//

#include "AStar.h"
#include "CooperativeAStar.h"
#include "MultiAgentProblem.h"
#include "Parser.h"
#include "ReverseResumableAStar.h"
#include "SingleAgentProblem.h"
#include "SingleAgentSpaceTimeProblem.h"

#include <chrono>
#include <iostream>
#include <vector>

template <typename T>
class Clock {
    std::chrono::high_resolution_clock clock;
    std::chrono::time_point<std::chrono::high_resolution_clock, std::chrono::nanoseconds> start;
    std::chrono::time_point<std::chrono::high_resolution_clock, std::chrono::nanoseconds> stop;
public:
    void tick() {
        this->start = this->clock.now();
    }
    void tack () {
        this->stop = this->clock.now();
    };
    void print () {
        std::cerr << std::chrono::duration_cast<T>(this->stop-this->start) << std::endl;
    };
};

Clock<std::chrono::milliseconds> timer;

int main() {

    // TEST 1 : 1 agent
    /*Graph g = parser::parse("../mapf-map/AssignmentIACourse.map");
    int start = 7;
    int target = 17;
    SingleAgentProblem problem = SingleAgentProblem(g, start, target);
    Solution solution = AStar(&problem, Manhattan).getSolution();
    // cost = 18, numberOfVisitedStates = 27
    solution.print();*/

    // TEST 2 : 1 agent
    /*Graph g = parser::parse("../mapf-map/Paris_1_256.map");
    int start = 1;
    int target = 256*200-100;
    timer.tick();
    SingleAgentProblem problem = SingleAgentProblem(g, start, target);
    Solution solution = AStar(&problem, Manhattan).getSolution();
    timer.tack();
    // cost = 354
    solution.print();
    timer.print();*/

    // TEST 3 : 2 agents can't be at the same vertex at the same time
    /*Graph g = parser::parse("../mapf-map/empty-4-4.map");
    vector<int> starts;
    starts.push_back(4);
    starts.push_back(9);
    vector<int> targets;
    targets.push_back(6);
    targets.push_back(1);
    MultiAgentProblem problem = MultiAgentProblem(g, starts, targets, Makespan);
    Solution solution = AStar(&problem, Manhattan).getSolution();
    // makespan cost = 3, fuel cost = 4,  sumofcosts cost = 5
    solution.print();*/

    // TEST 4 : 2 agents can't traverse the same edge between successive time steps
    /*Graph g = parser::parse("../mapf-map/empty-4-4.map");
    vector<int> starts;
    starts.push_back(0);
    starts.push_back(3);
    vector<int> targets;
    targets.push_back(3);
    targets.push_back(0);
    MultiAgentProblem problem = MultiAgentProblem(g, starts, targets, SumOfCosts);
    Solution solution = AStar(&problem, Manhattan).getSolution();
    // makespan cost = 5, fuel cost = 8,  sumofcosts cost = 8
    solution.print();*/

    // TEST 5 : 2 agents
    /*Graph g = parser::parse("../mapf-map/AssignmentIACourse.map");
    vector<int> starts;
    starts.push_back(48);
    starts.push_back(17);
    vector<int> targets;
    targets.push_back(17);
    targets.push_back(20);
    MultiAgentProblem problem = MultiAgentProblem(g, starts, targets, SumOfCosts);
    Solution solution = AStar(&problem, Manhattan).getSolution();
    // makespan cost = 7, fuel cost = 10,  sumofcosts cost = 10
    solution.print();*/

    // TEST 6 : 1 agent and a constraint (a, p, t)
    /*Graph g = parser::parse("../mapf-map/empty-4-4.map");
    int start = 4;
    int target = 6;
    SingleAgentSpaceTimeProblem problem = SingleAgentSpaceTimeProblem(g, start, target, Makespan, set<Constraint>{Constraint(0,5,1)});
    Solution solution = AStar(&problem, Manhattan).getSolution();
    // makespan cost = 3, fuel cost = 2
    solution.print();*/

    // TEST 7 : Reverse Resumable A*
    /*Graph g = parser::parse("../mapf-map/ReverseResumableAStarExample.map");
    int start = 33;
    int target = 2;
    SingleAgentProblem problem = SingleAgentProblem(g, start, target);
    ReverseResumableAStar search = ReverseResumableAStar(&problem);
    for(const auto& key_value: search.getExplored()) {
        shared_ptr<State> key = key_value.first;
        int value = key_value.second;
        cout << key->getPositions()[0] << " - " << value << endl;
    }
    cout << search.getExplored().size() << endl; // = 14
    cout << search.OptimalDistance(0) << endl; // = 2
    cout << search.getExplored().size() << endl; // = 20
    cout << search.OptimalDistance(5) << endl; // = 9
    cout << search.getExplored().size() << endl; // = 27
    cout << search.OptimalDistance(6) << endl; // = 3
    cout << search.getExplored().size() << endl; // = 27*/

    // TEST 8 : Single agent space time search, comparaison between Manhattan distance and Optimal distance (RRA*)
    /*Graph g = parser::parse("../mapf-map/AssignmentIACourse.map");
    int start = 7;
    int target = 17;
    SingleAgentSpaceTimeProblem problem = SingleAgentSpaceTimeProblem(g, start, target, Makespan);
    Solution solution1 = AStar(&problem, Manhattan).getSolution();
    Solution solution2 = AStar(&problem, OptimalDistance).getSolution();
    // makespan cost = 18, fuel cost = 18
    solution1.print(); // numberOfVisitedStates = 179
    solution2.print(); // numberOfVisitedStates = 26*/

    // TEST 9 : Comparaison between cooperative A* (Manhattan for the single agent A*) and hierarchical cooperative A* (Optimal distance RRA* for the single agent A*)
    /*Graph g = parser::parse("../mapf-map/AssignmentIACourse.map");
    vector<int> starts;
    starts.push_back(17);
    starts.push_back(22);
    vector<int> targets;
    targets.push_back(7);
    targets.push_back(6);
    MultiAgentProblem problem = MultiAgentProblem(g, starts, targets, Makespan);
    Solution solution1 = CooperativeAStar(&problem, Manhattan).getSolution();
    Solution solution2 = CooperativeAStar(&problem, OptimalDistance).getSolution();
    solution1.print(); // numberOfVisitedStates = 395
    solution2.print(); // numberOfVisitedStates = 160*/

    // TEST 10 : 2 agents and comparaison between Manhattan and OptimalDistance
    /*Graph g = parser::parse("../mapf-map/AssignmentIACourse.map");
    vector<int> starts;
    starts.push_back(48);
    starts.push_back(17);
    vector<int> targets;
    targets.push_back(17);
    targets.push_back(20);
    MultiAgentProblem problem = MultiAgentProblem(g, starts, targets, SumOfCosts);
    Solution solution1 = AStar(&problem, Manhattan).getSolution();
    Solution solution2 = AStar(&problem, OptimalDistance).getSolution();
    solution1.print(); // numberOfVisitedStates = 23
    solution2.print(); // numberOfVisitedStates = 17*/

    // TEST 11 : 2 agents and comparaison between Manhattan and OptimalDistance
    auto g = Parser::parse("../mapf-map/Paris_1_256.map");
    std::vector<int> starts;
    starts.push_back(1);
    starts.push_back(150);
    std::vector<int> targets;
    targets.push_back(150);
    targets.push_back(1);
    auto problem = std::make_shared<MultiAgentProblem>(g, starts, targets, SumOfCosts);
    auto aStar = AStar<MultiAgentProblem, MultiAgentState>(problem, Manhattan);
    auto solution1 = aStar.solve();
    solution1.print(); // numberOfVisitedStates = 285477
    aStar = AStar<MultiAgentProblem, MultiAgentState>(problem, OptimalDistance);
    Solution solution2 = aStar.solve();
    solution2.print(); // numberOfVisitedStates = 4173
}
