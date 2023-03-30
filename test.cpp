//
// Created by mansj on 10/11/22.
//

#include "Solvers/AStar.h"
#include "Solvers/CooperativeAStar.h"
#include "Problems/MultiAgentProblem.h"
#include "GraphParser/Parser.h"
#include "Solvers/ReverseResumableAStar.h"
#include "Problems/SingleAgentProblem.h"
#include "Problems/SingleAgentSpaceTimeProblem.h"
#include "Solvers/SimpleIndependenceDetection.h"
#include "Solvers/IndependenceDetection.h"

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

    // TEST 11 : Small Independence detection test
    /*auto g = Parser::parse("../mapf-map/AssignmentIACourse.map");
    vector<int> starts;
    starts.push_back(7);
    starts.push_back(3);
    starts.push_back(11);
    vector<int> targets;
    targets.push_back(21);
    targets.push_back(5);
    targets.push_back(4);
    auto problem = std::make_shared<MultiAgentProblem>(g, starts, targets, SumOfCosts, vector<int>{3,4,5});
    auto solution = SimpleIndependenceDetection(problem, OptimalDistance).solve();
    solution->print();
    */

    // TEST 12 : Independence Detection
    auto g = Parser::parse("../mapf-map/Paris/Paris_1_256.map");
    std::vector<int> starts;
    starts.push_back(1);
    starts.push_back(150);
    std::vector<int> targets;
    targets.push_back(150);
    targets.push_back(1);
    auto problem = std::make_shared<MultiAgentProblem>(g, starts, targets, SumOfCosts, std::vector<int>{5,10});
    std::chrono::time_point<std::chrono::system_clock> start, end;
    start = std::chrono::system_clock::now();
    auto solution1 = SimpleIndependenceDetection(problem, OptimalDistance).solve();
    end = std::chrono::system_clock::now();
    std::chrono::duration<double> elapsed_seconds = end - start;
    std::cout << elapsed_seconds.count() << std::endl;
    start = std::chrono::system_clock::now();
    auto solution2 = AStar<MultiAgentProblem,MultiAgentState>(problem, OptimalDistance).solve();
    end = std::chrono::system_clock::now();
    elapsed_seconds = end - start;
    std::cout << elapsed_seconds.count() << std::endl;
    std::cout << solution1->getMakespanCost() << std::endl;
    std::cout << solution2->getMakespanCost() << std::endl;
}
