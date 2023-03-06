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
    /*auto g = Parser::parse("../mapf-map/AssignmentIACourse.map");
    int start = 7;
    int target = 17;
    auto problem = std::make_shared<SingleAgentProblem>(g, start, target);
    Solution solution = AStar<SingleAgentProblem, SingleAgentState>(problem, Manhattan).solve();
    // cost = 18, numberOfVisitedStates = 25
    solution.print();*/

    // TEST 2 : 1 agent
    /*auto g = Parser::parse("../mapf-map/Paris_1_256.map");
    int start = 1;
    int target = 256*200-100;
    timer.tick();
    auto problem = std::make_shared<SingleAgentProblem>(g, start, target);
    Solution solution = AStar<SingleAgentProblem, SingleAgentState>(problem, Manhattan).solve();
    timer.tack();
    // cost = 354
    solution.print();
    timer.print();*/

    // TEST 3 : 2 agents can't be at the same vertex at the same time
    /*auto g = Parser::parse("../mapf-map/empty-4-4.map");
    vector<int> starts;
    starts.push_back(4);
    starts.push_back(9);
    vector<int> targets;
    targets.push_back(6);
    targets.push_back(1);
    auto problem = std::make_shared<MultiAgentProblem>(g, starts, targets, Makespan);
    Solution solution = AStar<MultiAgentProblem, MultiAgentState>(problem, Manhattan).solve();
    // makespan cost = 3, fuel cost = 4,  sumofcosts cost = 5
    solution.print();*/

    // TEST 4 : 2 agents can't traverse the same edge between successive time steps
    /*auto g = Parser::parse("../mapf-map/empty-4-4.map");
    vector<int> starts;
    starts.push_back(0);
    starts.push_back(3);
    vector<int> targets;
    targets.push_back(3);
    targets.push_back(0);
    auto problem = std::make_shared<MultiAgentProblem>(g, starts, targets, SumOfCosts);
    Solution solution = AStar<MultiAgentProblem, MultiAgentState>(problem, Manhattan).solve();
    // makespan cost = 5, fuel cost = 8,  sumofcosts cost = 8
    solution.print();*/

    // TEST 5 : 2 agents
    /*auto g = Parser::parse("../mapf-map/AssignmentIACourse.map");
    vector<int> starts;
    starts.push_back(48);
    starts.push_back(17);
    vector<int> targets;
    targets.push_back(17);
    targets.push_back(20);
    auto problem = std::make_shared<MultiAgentProblem>(g, starts, targets, Makespan);
    Solution solution = AStar<MultiAgentProblem, MultiAgentState>(problem, Manhattan).solve();
    // makespan cost = 7, fuel cost = 10,  sumofcosts cost = 10
    solution.print();*/

    // TEST 6 : 1 agent and a constraint (a, p, t)
    /*auto g = Parser::parse("../mapf-map/empty-4-4.map");
    int start = 4;
    int target = 6;
    auto problem = std::make_shared<SingleAgentSpaceTimeProblem>(g, start, target, Makespan, std::set<Constraint>{Constraint{0,5,1}});
    Solution solution = AStar<SingleAgentSpaceTimeProblem, SingleAgentSpaceTimeState>(problem, Manhattan).solve();
    // makespan cost = 3, fuel cost = 2
    solution.print();*/

    // TEST 7 : Reverse Resumable A*
    /*auto g = Parser::parse("../mapf-map/ReverseResumableAStarExample.map");
    int start = 33;
    int target = 2;
    auto problem = std::make_shared<SingleAgentProblem>(g, start, target);
    ReverseResumableAStar search = ReverseResumableAStar(problem);
    std::cout << search.getDistance().size() << std::endl; // = 1
    std::cout << search.optimalDistance(0) << std::endl; // = 2
    std::cout << search.getDistance().size() << std::endl; // = 24
    std::cout << search.optimalDistance(5) << std::endl; // = 9
    std::cout << search.getDistance().size() << std::endl; // = 27
    std::cout << search.optimalDistance(6) << std::endl; // = 3
    std::cout << search.getDistance().size() << std::endl; // = 27
    for(const auto& key_value: search.getDistance()) {
        std::shared_ptr<SingleAgentState> key = key_value.first;
        int value = key_value.second;
        std::cout << key->getPosition() << " - " << value << std::endl;
    }*/

    // TEST 8 : Single agent space time search, comparaison between Manhattan distance and Optimal distance (RRA*)
    /*auto g = Parser::parse("../mapf-map/AssignmentIACourse.map");
    int start = 7;
    int target = 17;
    auto problem = std::make_shared<SingleAgentSpaceTimeProblem>(g, start, target, Makespan);
    Solution solution1 = AStar<SingleAgentSpaceTimeProblem, SingleAgentSpaceTimeState>(problem, Manhattan).solve();
    Solution solution2 = AStar<SingleAgentSpaceTimeProblem, SingleAgentSpaceTimeState>(problem, OptimalDistance).solve();
    // makespan cost = 18, fuel cost = 18
    solution1.print(); // numberOfVisitedStates = 177
    solution2.print(); // numberOfVisitedStates = 19*/

    // TEST 9 : Comparaison between cooperative A* (Manhattan for the single agent A*) and hierarchical cooperative A* (Optimal distance RRA* for the single agent A*)
    /*auto g = Parser::parse("../mapf-map/AssignmentIACourse.map");
    vector<int> starts;
    starts.push_back(17);
    starts.push_back(22);
    vector<int> targets;
    targets.push_back(7);
    targets.push_back(6);
    auto problem = std::make_shared<MultiAgentProblem>(g, starts, targets, Makespan);
    Solution solution1 = CooperativeAStar(problem, Manhattan).solve();
    Solution solution2 = CooperativeAStar(problem, OptimalDistance).solve();
    solution1.print(); // numberOfVisitedStates = 389
    solution2.print(); // numberOfVisitedStates = 142*/

    // TEST 10 : 2 agents and comparaison between Manhattan and OptimalDistance
    /*auto g = Parser::parse("../mapf-map/AssignmentIACourse.map");
    vector<int> starts;
    starts.push_back(48);
    starts.push_back(17);
    vector<int> targets;
    targets.push_back(17);
    targets.push_back(20);
    auto problem = std::make_shared<MultiAgentProblem>(g, starts, targets, SumOfCosts);
    Solution solution1 = AStar<MultiAgentProblem, MultiAgentState>(problem, Manhattan).solve();
    Solution solution2 = AStar<MultiAgentProblem, MultiAgentState>(problem, OptimalDistance).solve();
    solution1.print(); // numberOfVisitedStates = 17
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
    auto solution1 = AStar<MultiAgentProblem, MultiAgentState>(problem, Manhattan).solve();
    solution1.print(); // numberOfVisitedStates = 690412
    auto solution2 = AStar<MultiAgentProblem, MultiAgentState>(problem, OptimalDistance).solve();
    solution2.print(); // numberOfVisitedStates = 331
}
